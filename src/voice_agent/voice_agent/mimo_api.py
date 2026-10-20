"""小米 MiMo Token Plan API 的最小异步封装（LLM / ASR / TTS）。

MiMo 兼容 OpenAI Chat Completions 协议，因此直接复用 openai SDK。
语音模型按 MiMo 的约定通过 `audio` 参数请求 WAV。
"""

from __future__ import annotations

import base64
import json
import os
from dataclasses import dataclass
from typing import Any, AsyncGenerator, Callable, Dict, List, Optional

from openai import AsyncOpenAI


@dataclass
class MimoConfig:
    api_key: str
    base_url: str
    model: str
    asr_model: str
    tts_model: str
    tts_voice: str


def load_mimo_config() -> MimoConfig:
    return MimoConfig(
        api_key=os.getenv("MIMO_API_KEY", "").strip(),
        base_url=os.getenv(
            "MIMO_BASE_URL", "https://token-plan-cn.xiaomimimo.com/v1"
        ).strip(),
        model=os.getenv("MIMO_MODEL", "mimo-v2.6-flash").strip(),
        asr_model=os.getenv("MIMO_ASR_MODEL", "mimo-v2.5-asr").strip(),
        tts_model=os.getenv("MIMO_TTS_MODEL", "mimo-v2.5-tts").strip(),
        tts_voice=os.getenv("MIMO_TTS_VOICE", "冰糖").strip(),
    )


def create_client(config: MimoConfig) -> Optional[AsyncOpenAI]:
    if not config.api_key or "your_token_plan_key" in config.api_key:
        return None
    return AsyncOpenAI(api_key=config.api_key, base_url=config.base_url)


ToolExecutor = Callable[[str, Dict[str, Any]], str]


async def stream_chat(
    client: AsyncOpenAI,
    config: MimoConfig,
    messages: List[Dict[str, Any]],
    tools: Optional[List[Dict[str, Any]]] = None,
    tool_executor: Optional[ToolExecutor] = None,
    max_tool_rounds: int = 3,
) -> AsyncGenerator[Dict[str, Any], None]:
    """流式对话，支持多轮 OpenAI 风格 tool calling。

    产出事件：
      {"type": "delta", "text": ...}
      {"type": "tool_call", "name": ..., "arguments": ...}
      {"type": "tool_result", "name": ..., "result": ...}
    """
    for round_index in range(max_tool_rounds + 1):
        allow_tools = bool(
            tools and tool_executor and round_index < max_tool_rounds
        )
        kwargs: Dict[str, Any] = {
            "model": config.model,
            "messages": messages,
            "stream": True,
            "temperature": 0.7,
            "max_tokens": 500,
        }
        if allow_tools:
            kwargs["tools"] = tools
            kwargs["tool_choice"] = "auto"

        stream = await client.chat.completions.create(**kwargs)
        content_parts: List[str] = []
        tool_calls: Dict[int, Dict[str, str]] = {}

        async for chunk in stream:
            if not getattr(chunk, "choices", None):
                continue
            delta = chunk.choices[0].delta
            if delta is None:
                continue
            if getattr(delta, "content", None):
                content_parts.append(delta.content)
                yield {"type": "delta", "text": delta.content}
            for call in getattr(delta, "tool_calls", None) or []:
                index = getattr(call, "index", 0) or 0
                entry = tool_calls.setdefault(
                    index, {"id": "", "name": "", "arguments": ""}
                )
                if getattr(call, "id", None):
                    entry["id"] = call.id
                function = getattr(call, "function", None)
                if function is not None:
                    if getattr(function, "name", None):
                        entry["name"] += function.name
                    if getattr(function, "arguments", None):
                        entry["arguments"] += function.arguments

        if not tool_calls:
            return

        ordered = [tool_calls[index] for index in sorted(tool_calls)]
        normalized = []
        for index, call in enumerate(ordered):
            name = call.get("name") or ""
            if not name:
                continue
            call_id = call.get("id") or f"call_{round_index}_{index}"
            normalized.append(
                {"id": call_id, "name": name, "arguments": call.get("arguments") or "{}"}
            )
        if not normalized:
            return

        messages.append(
            {
                "role": "assistant",
                "content": "".join(content_parts) or None,
                "tool_calls": [
                    {
                        "id": call["id"],
                        "type": "function",
                        "function": {
                            "name": call["name"],
                            "arguments": call["arguments"],
                        },
                    }
                    for call in normalized
                ],
            }
        )

        for call in normalized:
            yield {
                "type": "tool_call",
                "name": call["name"],
                "arguments": call["arguments"],
            }
            try:
                arguments = json.loads(call["arguments"]) if call["arguments"] else {}
                if not isinstance(arguments, dict):
                    raise ValueError("工具参数必须是 JSON 对象")
                result = tool_executor(call["name"], arguments)
            except Exception as exc:
                result = f"工具调用失败: {exc}"
            messages.append(
                {
                    "role": "tool",
                    "tool_call_id": call["id"],
                    "content": str(result),
                }
            )
            yield {
                "type": "tool_result",
                "name": call["name"],
                "result": str(result),
            }

    return


def _message_text(message: Any) -> str:
    content = getattr(message, "content", "") or ""
    if isinstance(content, str):
        return content.strip()
    if isinstance(content, list):
        parts = []
        for item in content:
            if isinstance(item, str):
                parts.append(item)
            elif isinstance(item, dict):
                parts.append(str(item.get("text") or item.get("content") or ""))
        return "".join(parts).strip()
    return str(content).strip()


def _extract_audio(response: Any) -> Optional[str]:
    choice = response.choices[0]
    message = choice.message
    audio = getattr(message, "audio", None)
    if audio is not None:
        if hasattr(audio, "data"):
            return audio.data
        if isinstance(audio, dict):
            return audio.get("data")
    content = _message_text(message)
    if not content:
        return None
    if content.startswith("data:") and "," in content:
        return content.split(",", 1)[1]
    return content


async def transcribe(client: AsyncOpenAI, config: MimoConfig, audio: bytes) -> str:
    encoded = base64.b64encode(audio).decode("utf-8")
    data_url = f"data:audio/wav;base64,{encoded}"
    response = await client.chat.completions.create(
        model=config.asr_model,
        messages=[
            {
                "role": "user",
                "content": [
                    {"type": "input_audio", "input_audio": {"data": data_url}},
                ],
            }
        ],
    )
    return _message_text(response.choices[0].message)


async def synthesize(
    client: AsyncOpenAI,
    config: MimoConfig,
    text: str,
    voice: Optional[str] = None,
) -> Dict[str, str]:
    voice = (voice or config.tts_voice or "冰糖").strip()
    audio_config = {"format": "wav", "voice": voice}
    messages = [
        {"role": "user", "content": "请用自然亲切的语调朗读"},
        {"role": "assistant", "content": text},
    ]
    try:
        response = await client.chat.completions.create(
            model=config.tts_model,
            messages=messages,
            audio=audio_config,
        )
    except TypeError:
        # 老版本 SDK 没有 audio 形参时走 extra_body
        response = await client.chat.completions.create(
            model=config.tts_model,
            messages=messages,
            extra_body={"audio": audio_config},
        )
    data = _extract_audio(response)
    if not data:
        raise RuntimeError("MiMo TTS 没有返回音频数据")
    return {"format": "wav", "voice": voice, "audio_base64": data}
