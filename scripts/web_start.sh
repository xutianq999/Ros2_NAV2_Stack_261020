#!/usr/bin/env bash
#
# 网页控制台启动脚本：rosbridge (WebSocket 9090) + 静态文件服务 (HTTP 5500)。
#
# 网页本身不启动任何导航节点 —— 底盘 CAN、FAST-LIO、Nav2 全在 nav2_real.sh 里。
# 正确顺序：先跑 scripts/nav2_real.sh 启动导航栈，再跑本脚本，最后浏览器打开提示地址。
#
# 用法：
#   ./scripts/web_start.sh
#
# 环境变量：
#   WEB_PORT  静态服务端口，默认 5500

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
WEB_DIR="$WORKSPACE_ROOT/src/web"
WEB_PORT="${WEB_PORT:-5500}"

[ -d "$WEB_DIR" ] || { echo "[web] 错误: 未找到网页目录 $WEB_DIR" >&2; exit 1; }

source /opt/ros/humble/setup.bash
# 后端需要 rclpy / NavigateToPose action，必须叠加本工作空间
set +u
source "$WORKSPACE_ROOT/install/setup.bash"
set -u

# rosbridge 未安装时明确报错退出（而不是静默失败、网页连不上 9090）
if ! ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then
  echo "[web] 错误: 未安装 ros-humble-rosbridge-suite，网页无法连接 ROS2"
  echo "       请先执行: sudo apt install ros-humble-rosbridge-suite"
  exit 1
fi

# rosbridge 未运行则另开终端启动
if ! pgrep -f "rosbridge_websocket" >/dev/null 2>&1; then
  echo ">>> 启动 rosbridge (ws://0.0.0.0:9090)"
  gnome-terminal --title="rosbridge" -- bash -c "
    source /opt/ros/humble/setup.bash;
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
  sleep 2
else
  echo ">>> rosbridge 已在运行"
fi

IP=$(hostname -I | awk '{print $1}')
echo ">>> 静态页面 + 后端 API: http://${IP}:${WEB_PORT}"
echo "    工作台  : http://${IP}:${WEB_PORT}/workbench.html   # 新版操作界面"
echo "    导航页  : http://${IP}:${WEB_PORT}/navigation.html"
echo "    座舱页  : http://${IP}:${WEB_PORT}/cockpit.html"
echo "    仪表盘  : http://${IP}:${WEB_PORT}/index.html"
echo "    任务 API: http://${IP}:${WEB_PORT}/api/mission/state"

if ! python3 -c "import fastapi, uvicorn" >/dev/null 2>&1; then
  echo "[web] 错误: 缺少 fastapi/uvicorn，后端无法启动"
  echo "       请先执行: python3 -m pip install fastapi uvicorn"
  exit 1
fi

# 后端同时托管静态页面、任务执行器、点位/任务持久化、地图保存和 PCD 转栅格
exec python3 "$WORKSPACE_ROOT/scripts/web_backend.py" --host 0.0.0.0 --port "$WEB_PORT"
