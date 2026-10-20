"""语音 Agent 的 HTTP/SSE 服务：麦克风录音 → ASR → Agent → TTS。

这个进程同时是一个 ROS 2 节点：rclpy 在后台线程 spin，FastAPI 在事件循环里
处理浏览器请求。机器人动作由 RobotAgent 同步执行，因此 HTTP 层不需要再经过
rosbridge。
"""

from __future__ import annotations

import json
import os
import sys
import threading
from typing import Any, Dict, List, Optional

import uvicorn
from dotenv import load_dotenv
from fastapi import FastAPI, File, HTTPException, Request, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from loguru import logger
from pydantic import BaseModel

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .agent_node import RobotAgent
from .intent_parser import parse_command
from .mimo_api import (
    create_client,
    load_mimo_config,
    stream_chat,
    synthesize,
    transcribe,
)


# ---------------------------------------------------------------------------
# 环境变量
# ---------------------------------------------------------------------------
_ENV_LOADED = False


def load_env_files() -> None:
    """按优先级加载 .env；已存在的环境变量不会被覆盖。"""
    global _ENV_LOADED
    if _ENV_LOADED:
        return
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = []
    if os.getenv("VOICE_AGENT_ENV"):
        candidates.append(os.environ["VOICE_AGENT_ENV"])
    candidates.extend(
        [
            os.path.expanduser("~/.config/voice_agent/.env"),
            os.path.abspath(".env"),
            os.path.join(here, "..", ".env"),              # src/voice_agent/.env
            os.path.join(here, "..", "..", "..", ".env"),  # 工作区根目录
        ]
    )
    for path in candidates:
        path = os.path.abspath(os.path.expanduser(path))
        if os.path.isfile(path):
            load_dotenv(path, override=False)
            logger.info(f"已加载环境变量: {path}")
    _ENV_LOADED = True


load_env_files()


# ---------------------------------------------------------------------------
# 大模型工具定义
# ---------------------------------------------------------------------------
ROBOT_TOOLS: List[Dict[str, Any]] = [
    {
        "type": "function",
        "function": {
            "name": "move_robot",
            "description": (
                "让小车运动。direction 为 forward/backward 时按 distance_m 走直线；"
                "direction 为 left/right 时按 angle_deg 原地转向。差速底盘不能横移。"
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "direction": {
                        "type": "string",
                        "enum": ["forward", "backward", "left", "right"],
                        "description": "运动方向",
                    },
                    "distance_m": {
                        "type": "number",
                        "description": "前进/后退距离，单位米；省略则用默认值",
                    },
                    "angle_deg": {
                        "type": "number",
                        "description": "左转/右转角度，单位度；省略则用默认值",
                    },
                    "speed": {
                        "type": "number",
                        "description": "线速度 m/s 或角速度 rad/s；省略则用默认值",
                    },
                },
                "required": ["direction"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "stop_robot",
            "description": "立即让小车停止，并取消正在执行的导航。",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "navigate_to_point",
            "description": "让 Nav2 导航到指定点位，例如“1号点位”“point_2”。",
            "parameters": {
                "type": "object",
                "properties": {
                    "point": {
                        "type": "string",
                        "description": "点位编号或名称，例如 1号点位 / point_1",
                    }
                },
                "required": ["point"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "list_waypoints",
            "description": "列出当前可用的导航点位。",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_robot_status",
            "description": "读取小车当前状态、位姿、导航进度和传感器健康度。",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "cancel_navigation",
            "description": "取消当前 Nav2 导航目标，不改变手动运动模块。",
            "parameters": {"type": "object", "properties": {}},
        },
    },
]


def build_system_prompt(node: RobotAgent) -> str:
    base = os.getenv("SYSTEM_PROMPT") or (
        "你是运行在 BUNKER MINI 2.0 差速底盘上的中文语音助手 MiMo。"
        "回答必须口语化、简明扼要，通常 1 到 3 句话，适合直接语音朗读；"
        "不要输出 Markdown、表格或代码块。"
        "涉及小车运动或导航时，必须调用提供的工具，绝不能假装已经执行。"
        "小车只能前进、后退和原地左转/右转，不能横向平移。"
        "用户说停止、急停、取消导航时，优先保证停车。"
        "每次只执行一个动作；如果指令含糊，先用一句话确认。"
    )
    waypoints = node.waypoints.list_text()
    return (
        f"{base}\n\n"
        f"当前可用点位：{waypoints}。"
        "用户说“去1号点位”时调用 navigate_to_point(point='1号点位')。"
        "不要调用工具绕过急停；急停只能由用户明确说“急停”触发。"
    )


def execute_tool(node: RobotAgent, name: str, arguments: Dict[str, Any]) -> str:
    """执行大模型请求的工具，返回给模型看的自然语言结果。"""
    logger.info(f"LLM 工具调用: {name} {arguments}")
    try:
        if name == "move_robot":
            result = node.start_move(
                direction=str(arguments.get("direction") or ""),
                distance_m=arguments.get("distance_m"),
                angle_deg=arguments.get("angle_deg"),
                speed=arguments.get("speed"),
                source="llm",
            )
            return result["reply"]
        if name == "stop_robot":
            return node.stop_all(reason="llm")["reply"]
        if name == "navigate_to_point":
            point = arguments.get("point") or arguments.get("name") or ""
            return node.navigate_to(str(point), source="llm")["reply"]
        if name == "list_waypoints":
            return f"可用点位：{node.waypoints.list_text()}"
        if name == "get_robot_status":
            return node.describe_status()
        if name == "cancel_navigation":
            return node.cancel_navigation()["reply"]
    except Exception as exc:
        logger.exception(f"工具 {name} 执行失败")
        return f"工具 {name} 执行失败: {exc}"
    return f"未知工具: {name}"


# ---------------------------------------------------------------------------
# 请求模型
# ---------------------------------------------------------------------------
class ChatMessage(BaseModel):
    role: str
    content: str


class ChatRequest(BaseModel):
    messages: List[ChatMessage]
    robot: Optional[bool] = True


class TTSRequest(BaseModel):
    text: str
    voice: Optional[str] = None


class RobotCommandRequest(BaseModel):
    text: str


class NavigateRequest(BaseModel):
    point: str


class EstopRequest(BaseModel):
    engage: bool = True


# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------
def _sse(payload: Dict[str, Any]) -> str:
    return f"data: {json.dumps(payload, ensure_ascii=False)}\n\n"


def _sse_done() -> str:
    return "data: [DONE]\n\n"


def _resolve_web_dir() -> str:
    # --symlink-install 下 __file__ 位于 build/，而 web 文件是指向 src/ 的符号
    # 链接；Starlette 默认拒绝符号链接逃逸，所以这里优先取 realpath 后的源码目录。
    here = os.path.dirname(os.path.realpath(__file__))
    candidates = [
        os.getenv("VOICE_AGENT_WEB_DIR"),
        os.path.join(here, "..", "web"),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(os.path.join(get_package_share_directory("voice_agent"), "web"))
    except Exception:
        pass
    for path in candidates:
        if path and os.path.isdir(path):
            return os.path.abspath(path)
    return os.path.abspath(os.path.join(here, "..", "web"))


def _error_reply(node: RobotAgent, message: str) -> str:
    return (
        f"{message}。你可以说：前进、后退、左转、右转、停止，"
        "或者说去一号点位、有哪些点位。"
    )


# ---------------------------------------------------------------------------
# FastAPI 应用
# ---------------------------------------------------------------------------
def create_app(node: RobotAgent) -> FastAPI:
    app = FastAPI(title="MiMo Robot Voice Agent", version="0.1.0")

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=False,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    @app.middleware("http")
    async def token_guard(request: Request, call_next):
        token = os.getenv("VOICE_AGENT_TOKEN", "").strip()
        path = request.url.path
        if token and path.startswith("/api/") and path != "/api/info":
            supplied = request.headers.get("X-API-Token") or request.query_params.get(
                "token", ""
            )
            if supplied != token:
                return JSONResponse(
                    {"detail": "无效的 VOICE_AGENT_TOKEN"}, status_code=401
                )
        return await call_next(request)

    @app.get("/api/info")
    async def get_info():
        config = load_mimo_config()
        return {
            "status": "online",
            "model": config.model,
            "base_url": config.base_url,
            "api_configured": bool(create_client(config)),
            "voices": ["冰糖", "茉莉", "甜茶"],
            "default_voice": config.tts_voice,
            "robot": node.get_status(),
            "waypoints": node.waypoints.as_dicts(),
            "capabilities": [
                "前进/后退/左转/右转",
                "停止/急停/解除急停",
                "导航到指定点位",
                "查询状态与点位列表",
                "自由语音对话",
            ],
        }

    @app.get("/api/robot/status")
    async def robot_status():
        return node.get_status()

    @app.get("/api/waypoints")
    async def get_waypoints():
        return {
            "frame": node.waypoints.frame,
            "file": node.waypoints.path,
            "error": node.waypoints.load_error,
            "points": node.waypoints.as_dicts(),
        }

    @app.post("/api/robot/command")
    async def robot_command(req: RobotCommandRequest):
        result = node.handle_text(req.text, source="api")
        result["status"] = node.get_status()
        return result

    @app.post("/api/robot/navigate")
    async def robot_navigate(req: NavigateRequest):
        result = node.navigate_to(req.point, source="api")
        result["status"] = node.get_status()
        return result

    @app.post("/api/robot/stop")
    async def robot_stop():
        result = node.stop_all(reason="api")
        result["status"] = node.get_status()
        return result

    @app.post("/api/robot/estop")
    async def robot_estop(req: EstopRequest):
        result = node.emergency_stop() if req.engage else node.release_estop()
        result["status"] = node.get_status()
        return result

    @app.post("/api/chat")
    async def chat_endpoint(req: ChatRequest):
        user_text = next(
            (m.content for m in reversed(req.messages) if m.role == "user"), ""
        ).strip()
        if not user_text:
            raise HTTPException(status_code=400, detail="没有用户消息")

        async def event_generator():
            # 1) 确定性口令优先：停车/运动/导航不能等大模型
            if req.robot and parse_command(user_text) is not None:
                result = node.handle_text(user_text, source="chat")
                if result.get("handled"):
                    yield _sse({"delta": result.get("reply", "")})
                    yield _sse({"robot": node.get_status()})
                    yield _sse_done()
                    return

            # 2) 自由对话 + LLM 工具调用
            config = load_mimo_config()
            client = create_client(config)
            if client is None:
                yield _sse({"delta": _error_reply(node, "还没有配置 MIMO_API_KEY")})
                yield _sse_done()
                return

            history = [
                {"role": m.role, "content": m.content}
                for m in req.messages[-20:]
                if m.role in ("user", "assistant") and m.content.strip()
            ]
            messages = [{"role": "system", "content": build_system_prompt(node)}]
            messages.extend(history)

            emitted_text = False
            emitted_tool = False
            try:
                async for event in stream_chat(
                    client,
                    config,
                    messages,
                    tools=ROBOT_TOOLS if req.robot else None,
                    tool_executor=(
                        (lambda name, args: execute_tool(node, name, args))
                        if req.robot
                        else None
                    ),
                ):
                    if event["type"] == "delta":
                        emitted_text = True
                        yield _sse({"delta": event["text"]})
                    elif event["type"] == "tool_call":
                        emitted_tool = True
                        yield _sse(
                            {
                                "tool_call": {
                                    "name": event["name"],
                                    "arguments": event["arguments"],
                                }
                            }
                        )
                    elif event["type"] == "tool_result":
                        yield _sse(
                            {
                                "tool_result": {
                                    "name": event["name"],
                                    "result": event["result"],
                                }
                            }
                        )
            except Exception as exc:
                logger.exception("大模型流式请求失败")
                # 如果工具调用不被该模型/端点支持，退化为纯文本对话
                if not emitted_text and not emitted_tool:
                    try:
                        async for event in stream_chat(
                            client, config, messages, tools=None
                        ):
                            if event["type"] == "delta":
                                emitted_text = True
                                yield _sse({"delta": event["text"]})
                    except Exception as retry_exc:
                        logger.exception("大模型降级请求也失败")
                        yield _sse(
                            {
                                "error": (
                                    f"大模型请求失败：{exc}；"
                                    f"降级重试失败：{retry_exc}"
                                )
                            }
                        )
                        yield _sse_done()
                        return
                else:
                    yield _sse({"error": f"大模型请求中断：{exc}"})

            if not emitted_text and not emitted_tool:
                yield _sse({"delta": "我没太听懂，请再说一遍或者换个说法。"})
            yield _sse({"robot": node.get_status()})
            yield _sse_done()

        return StreamingResponse(
            event_generator(),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no",
            },
        )

    @app.post("/api/asr")
    async def asr_endpoint(file: UploadFile = File(...)):
        content = await file.read()
        if not content:
            raise HTTPException(status_code=400, detail="空音频文件")
        if len(content) > 25 * 1024 * 1024:
            raise HTTPException(status_code=413, detail="音频文件过大")
        config = load_mimo_config()
        client = create_client(config)
        if client is None:
            raise HTTPException(
                status_code=503, detail="未配置 MIMO_API_KEY，无法调用 MiMo ASR"
            )
        try:
            text = await transcribe(client, config, content)
        except Exception as exc:
            logger.exception("ASR 失败")
            raise HTTPException(status_code=500, detail=f"ASR 失败: {exc}") from exc
        logger.info(f"ASR 转写: {text}")
        return {"text": text}

    @app.post("/api/tts")
    async def tts_endpoint(req: TTSRequest):
        text = req.text.strip()
        if not text:
            raise HTTPException(status_code=400, detail="文本不能为空")
        config = load_mimo_config()
        client = create_client(config)
        if client is None:
            raise HTTPException(
                status_code=503, detail="未配置 MIMO_API_KEY，无法调用 MiMo TTS"
            )
        try:
            return await synthesize(client, config, text, req.voice)
        except Exception as exc:
            logger.exception("TTS 失败")
            raise HTTPException(status_code=500, detail=f"TTS 失败: {exc}") from exc

    web_dir = _resolve_web_dir()
    if os.path.isdir(web_dir):
        # 兼容 --symlink-install 与普通安装
        from inspect import signature

        kwargs: Dict[str, Any] = {"directory": web_dir, "html": True}
        if "follow_symlink" in signature(StaticFiles.__init__).parameters:
            kwargs["follow_symlink"] = True
        app.mount("/", StaticFiles(**kwargs), name="static")
    else:
        logger.warning(f"未找到网页目录: {web_dir}")

    return app


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    load_env_files()
    rclpy.init(args=sys.argv)
    node = RobotAgent()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = create_app(node)
    host = node.host
    port = node.port
    ssl_certfile = os.getenv("SSL_CERTFILE") or None
    ssl_keyfile = os.getenv("SSL_KEYFILE") or None
    scheme = "https" if ssl_certfile else "http"

    logger.info("=" * 66)
    logger.info("MiMo 小车语音 Agent 已启动")
    logger.info(f"本机访问: {scheme}://localhost:{port}")
    logger.info(f"局域网访问: {scheme}://<小车IP>:{port}")
    if not ssl_certfile:
        logger.warning(
            "浏览器只允许 localhost 或 HTTPS 使用麦克风；"
            "手机通过局域网 HTTP 访问时录音会被拒绝。"
        )
    logger.info("=" * 66)

    try:
        uvicorn.run(
            app,
            host=host,
            port=port,
            log_level="info",
            ssl_certfile=ssl_certfile,
            ssl_keyfile=ssl_keyfile,
        )
    finally:
        node.shutdown()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
