#!/usr/bin/env bash
# 单独启动网页后端（静态页面 + API + 服务端任务执行）
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT"
source /opt/ros/humble/setup.bash
set +u
source "$WORKSPACE_ROOT/install/setup.bash"
set -u
exec python3 "$WORKSPACE_ROOT/scripts/web_backend.py" "$@"
