#!/usr/bin/env bash
#
# 启动小车语音 Agent（HTTP 服务 + ROS 2 节点）。
#
# 前置：
#   1. 底盘已启动：                ./scripts/chassis_start.sh
#   2. 导航栈已启动（需要导航时）：./scripts/super_lio_nav.sh relocation
#      或 ./scripts/nav2_real.sh
#   3. 已编译本包：                colcon build --packages-select voice_agent --symlink-install
#
# 用法：
#   ./scripts/voice_agent_start.sh
#   PORT=8000 ./scripts/voice_agent_start.sh
#   ./scripts/voice_agent_start.sh port:=8080
#
# 环境变量：
#   VOICE_AGENT_ENV  .env 文件路径（默认自动查找 ~/.config/voice_agent/.env、
#                    src/voice_agent/.env、工作区根目录/.env）

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

if ! ros2 pkg prefix voice_agent >/dev/null 2>&1; then
  echo "[voice_agent] 错误: 找不到 voice_agent 包，请先编译：" >&2
  echo "  colcon build --packages-select voice_agent --symlink-install" >&2
  exit 1
fi

export VOICE_AGENT_ENV="${VOICE_AGENT_ENV:-$WORKSPACE_ROOT/src/voice_agent/.env}"
if [ ! -f "$VOICE_AGENT_ENV" ]; then
  echo "[voice_agent] 提示: 未找到 $VOICE_AGENT_ENV"
  echo "             复制模板并填入 MiMo Key："
  echo "             cp src/voice_agent/.env.example src/voice_agent/.env"
fi

exec ros2 launch voice_agent voice_agent_launch.py "$@"
