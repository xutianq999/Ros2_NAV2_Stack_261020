#!/usr/bin/env bash
#
# 清理上一次运行残留的本工程 ROS 进程。
#
# 用法：
#   ./scripts/cleanup_ros_stack.sh              # 清理本工程导航/探索链路
#   ./scripts/cleanup_ros_stack.sh --driver     # 同时清理 Livox 驱动
#   ./scripts/cleanup_ros_stack.sh --dry-run    # 只列出要结束的进程，不发信号
#
# 启动脚本（explore_record.sh / super_lio_explore.sh / super_lio_nav.sh）
# 默认会在启动前调用它；想跳过就设 SLIO_NO_CLEAN=1（explore_record.sh 用
# --no-clean）。
#
# 故意不清理：
#   - 底盘驱动（bunker / bunker_base）：由 chassis_start.sh 单独负责
#   - rosbridge / 网页服务 / rviz2：不参与控制，重复运行无害
#   - 其它用户的进程：只匹配当前用户
#
# 顺序：先 SIGINT 让 launch 自己收尾，再 SIGTERM，最后 SIGKILL。
# 当前脚本及其祖先进程永远排除，避免把正在启动的新会话杀掉。

set -uo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
INCLUDE_DRIVER=false
DRY_RUN=false

for arg in "$@"; do
  case "$arg" in
    --driver) INCLUDE_DRIVER=true ;;
    --dry-run) DRY_RUN=true ;;
    -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
    *) echo "未知参数: $arg" >&2; exit 2 ;;
  esac
done

log() { printf '[cleanup] %s\n' "$*"; }

# 当前进程和整条祖先链不能杀（例如调用本脚本的 explore_record.sh）
EXCLUDE=" $$ $PPID "
p=$PPID
while [ "${p:-0}" -gt 1 ] 2>/dev/null; do
  p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
  [ -z "$p" ] && break
  EXCLUDE+=" $p "
done

is_excluded() {
  case " $EXCLUDE " in *" $1 "*) return 0 ;; esac
  return 1
}

PATTERNS=(
  # 顶层 launch：先让它们自己收尾
  "exploration_bringup_launch.py"
  "super_lio_bringup_launch.py"
  "safe_nav_bringup_launch.py"
  "navigation_safe_launch.py"
  "safety_layer_launch.py"
  "my_nav2_launch.py"
  # 本工程节点
  "super_lio_node"
  "relocation_node"
  "super_lio_bridge"
  "tf_bridge.py"
  "small_gicp_relocalization_node"
  "global_kiss_matcher_relocalization"
  "pointcloud_to_laserscan_node"
  "grid_mapper"
  "explore_node"
  "exploration_ui"
  "record_waypoints"
  # Nav2
  "lib/nav2_behaviors/behavior_server"
  "lib/nav2_bt_navigator/bt_navigator"
  "lib/nav2_controller/controller_server"
  "lib/nav2_planner/planner_server"
  "lib/nav2_smoother/smoother_server"
  "lib/nav2_velocity_smoother/velocity_smoother"
  "lib/nav2_waypoint_follower/waypoint_follower"
  "lib/nav2_map_server/map_server"
  "lib/nav2_lifecycle_manager/lifecycle_manager"
  "lib/nav2_collision_monitor/collision_monitor"
  "twist_mux/twist_mux"
  # 本工程安全层
  "install/nav2_safety/lib/nav2_safety/"
  # 其它常见的建图/录制残留
  "slam_toolbox"
  "async_slam_toolbox_node"
  "ros2 bag record.*${WORKSPACE}"
  # 旧启动脚本本身（当前脚本在 EXCLUDE 里，不会被误杀）
  "explore_record.sh"
  "super_lio_explore.sh"
  "super_lio_nav.sh"
)

if [ "$INCLUDE_DRIVER" = true ]; then
  PATTERNS+=(
    "livox_ros_driver2_node"
    "msg_MID360_launch.py"
  )
fi

collect_pattern() {
  local pattern="$1"
  while read -r pid; do
    [ -z "$pid" ] && continue
    is_excluded "$pid" && continue
    printf '%s\n' "$pid"
  done < <(pgrep -u "$(id -u)" -f -- "$pattern" 2>/dev/null || true)
}

collect_all() {
  local pids=""
  for pat in "${PATTERNS[@]}"; do
    pids+=$'\n'$(collect_pattern "$pat")
  done
  printf '%s\n' "$pids" | awk 'NF && !seen[$1]++' | tr '\n' ' '
}

ALL="$(collect_all)"
if [ -z "${ALL// }" ]; then
  log "没有发现残留进程"
  exit 0
fi

log "将结束以下进程："
for pid in $ALL; do
  ps -o pid=,lstart=,cmd= -p "$pid" 2>/dev/null | sed 's/^/  /'
done

if [ "$DRY_RUN" = true ]; then
  log "dry-run：不发送信号"
  exit 0
fi

for sig in INT TERM KILL; do
  pids="$(collect_all)"
  [ -z "${pids// }" ] && break
  log "发送 SIG${sig} -> ${pids}"
  for pid in $pids; do
    kill -s "$sig" "$pid" 2>/dev/null || true
  done
  case "$sig" in
    INT) sleep 4 ;;
    TERM) sleep 2 ;;
  esac
done

left="$(collect_all)"
if [ -n "${left// }" ]; then
  log "警告：仍有进程未退出：${left}"
  exit 1
fi

log "残留清理完成"
