#!/usr/bin/env bash
#
# 底盘启动脚本：配置 CAN 总线并拉起 BUNKER MINI 2.0 底盘驱动。
#
# 底盘是 /cmd_vel 的实际执行者，并通过 CAN 把轮式里程计发到 /wheel_odom。
# 必须在 Nav2 / 导航栈之前启动。
#
# 用法：
#   ./chassis_start.sh              # 配置 can0 并启动底盘驱动
#   ./chassis_start.sh --can-only   # 只配置 can0，不启动驱动（供其它脚本在前台调用）
#
# 环境变量：
#   CAN_IFACE    默认 can0
#   CAN_BITRATE  默认 500000（Bunker Mini 2.0 用 500k）

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

CAN_IFACE="${CAN_IFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"
CAN_ONLY=0
[ "${1:-}" = "--can-only" ] && CAN_ONLY=1

log() { printf '[chassis] %s\n' "$*"; }
die() { printf '[chassis] 错误: %s\n' "$*" >&2; exit 1; }

# --- 1. 确保 CAN 接口存在（USB 适配器需要 gs_usb 内核模块）-----------------
if [ ! -e "/sys/class/net/${CAN_IFACE}" ]; then
  log "未发现 ${CAN_IFACE}，尝试加载 gs_usb 内核模块..."
  sudo modprobe gs_usb || true
  sleep 1
fi
[ -e "/sys/class/net/${CAN_IFACE}" ] \
  || die "仍未发现 ${CAN_IFACE}，请检查 CAN-USB 适配器是否插好。"

# --- 2. 拉起接口并设置波特率 ----------------------------------------------
# 接口若已 UP 则不重复配置（重复 set 在波特率不一致时会报 busy）。
if ip link show "$CAN_IFACE" | grep -qE '<[^>]*\bUP\b'; then
  log "${CAN_IFACE} 已处于 UP 状态，跳过配置。"
else
  log "拉起 ${CAN_IFACE}（bitrate=${CAN_BITRATE}）..."
  sudo ip link set "$CAN_IFACE" up type can bitrate "$CAN_BITRATE"
fi

# --- 3. 采样确认总线确实有报文 --------------------------------------------
if command -v candump >/dev/null 2>&1; then
  log "采样 ${CAN_IFACE} 2 秒，确认总线上有报文..."
  if timeout 2 candump -n 1 "$CAN_IFACE" >/dev/null 2>&1; then
    log "已收到 CAN 报文，总线正常。"
  else
    log "警告: 2 秒内未收到 CAN 报文，请确认底盘已上电、线缆连接正常。"
  fi
else
  log "未安装 can-utils（缺少 candump），跳过总线自检。"
fi

[ "$CAN_ONLY" = 1 ] && { log "仅配置模式（--can-only），不启动驱动。"; exit 0; }

# --- 4. 启动底盘驱动 ------------------------------------------------------
log "启动 BUNKER MINI 2.0 底盘驱动（bunker_mini2_base.launch.py）..."
# ROS 的 setup.bash 会引用未定义的 COLCON_TRACE，与 set -u 冲突，
# 故 source 期间临时关闭 nounset。
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u
exec ros2 launch me_nav2_bringup bunker_mini2_base.launch.py
