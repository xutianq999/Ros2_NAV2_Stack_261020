#!/usr/bin/env bash
#
# 自主探索启动脚本：Super-LIO + grid_mapper + Nav2 + explore_lite
#
# 用法：
#   ./scripts/super_lio_explore.sh                 # 实机：起雷达驱动，走墙钟
#   ./scripts/super_lio_explore.sh rviz            # 带 RViz
#   ./scripts/super_lio_explore.sh rviz manual     # 带 RViz，手动建图模式
#   EXPLORATION_MODE=manual ./scripts/super_lio_explore.sh rviz
#                                                  # 用环境变量指定模式
#   SLIO_NO_DRIVER=1 SLIO_USE_SIM_TIME=1 ./scripts/super_lio_explore.sh
#                                                  # 回放数据包（配 --clock，见下）
#
# 回放数据包的完整用法（两个环境变量必须成对，缺一会让 TF 时间戳错几个月）：
#   SLIO_NO_DRIVER=1 SLIO_USE_SIM_TIME=1 ./scripts/super_lio_explore.sh
#   ros2 bag play <bag> --clock --rate 1.0
#
# 启动前默认会清掉上一次运行残留的同一套 ROS 进程；想保留它们用：
#   SLIO_NO_CLEAN=1 ./scripts/super_lio_explore.sh
#
# 底盘不在这里：实机请先跑 ./scripts/chassis_start.sh。
#
# ---------------------------------------------------------------------------
# 为什么单独一个脚本，而不是给 super_lio_nav.sh 再加个模式
#
# 探索是让车自己在未知区域里开，是这套栈里风险最高的用法，而它最常见的三种失败
# 都表现为同一句话——"车不动"，从滚动的日志里几乎看不出原因：
#
#   1. map_server 残留（上个会话没退干净）：它和 grid_mapper 都在发 /map，
#      全局代价地图在两份图之间来回跳，前沿点随机失效。
#   2. 安全层的定位看门狗判定 /localization/tracking_ok 失效：它会把自动驾驶
#      锁住。此时 explore_lite 照常出目标、Nav2 照常规划，但 /cmd_vel 被掐掉。
#   3. 整链某个节点没起来（例如 /scan 因为坐标系不通而永远没有数据）。
#
# 所以本脚本在启动前后各加一道检查，把这些情况变成启动时就看得见的报错。
# 纯启动（跳过检查）走 ./scripts/super_lio_nav.sh exploration。

set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
SUPERLIO_WS="${SUPERLIO_WS:-/home/demo/superlio_ws/Super-LIO-ros2}"

RVIZ="${1:-false}"
# 默认探索模式，直接改这里即可：auto / manual
DEFAULT_EXPLORATION_MODE=auto
USE_DRIVER=true
[ "${SLIO_NO_DRIVER:-0}" = "1" ] && USE_DRIVER=false
USE_SIM_TIME=false
[ "${SLIO_USE_SIM_TIME:-0}" = "1" ] && USE_SIM_TIME=true

log()  { printf '[explore] %s\n' "$*"; }
ok()   { printf '[explore]   ✓ %s\n' "$*"; }
bad()  { printf '[explore]   ✗ %s\n' "$*"; }
die()  { printf '[explore] 错误: %s\n' "$*" >&2; exit 1; }

# auto   = 自主前沿探索
# manual = 不起前沿目标，RViz 2D Goal Pose 直接交给 Nav2 手动建图
mode_arg="${2:-$DEFAULT_EXPLORATION_MODE}"
# Also accept the mode as the first argument: ./super_lio_explore.sh manual
if [ "$RVIZ" = "auto" ] || [ "$RVIZ" = "manual" ]; then
  mode_arg="$RVIZ"
  RVIZ=false
fi
EXPLORATION_MODE="${EXPLORATION_MODE:-$mode_arg}"
case "$EXPLORATION_MODE" in
  auto|manual) ;;
  *) die "EXPLORATION_MODE 只能是 auto 或 manual（当前: $EXPLORATION_MODE）" ;;
esac

if [ "$RVIZ" != false ] && [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  log "WARNING: DISPLAY/WAYLAND_DISPLAY 未设置，RViz 可能打不开。"
  log "         请在桌面终端里运行，或先 export DISPLAY=:0。"
fi

cd "$WORKSPACE_ROOT" || exit 1

# --- 1. source 两个工作区（Super-LIO 是独立工作空间）------------------------
if [ ! -f "$SUPERLIO_WS/install/setup.bash" ]; then
  die "未找到 Super-LIO 的 install/setup.bash（期望 $SUPERLIO_WS），请先编译它"
fi
# ROS 的 setup.bash 会引用未定义的 COLCON_TRACE，与 set -u 冲突
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WORKSPACE_ROOT/install/setup.bash"
# shellcheck disable=SC1091
source "$SUPERLIO_WS/install/setup.bash"
set -u

# --- 2. 前置检查 -----------------------------------------------------------
for pkg in super_lio super_lio_bridge grid_mapper explore_lite me_nav2_bringup nav2_safety; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 \
    || die "找不到包 $pkg（先 colcon build --symlink-install，并确认两个工作区都已 source）"
done

# 默认清掉上一次运行残留的同一套进程。旧 run 留下的孤儿安全节点会让
# /safety/*、/robot/status 出现两个发布者，看起来像"状态乱跳"。
if [ "${SLIO_NO_CLEAN:-0}" != "1" ]; then
  log "自动清理上次运行残留的 ROS 进程..."
  if [ "$USE_DRIVER" = true ]; then
    "$SCRIPT_DIR/cleanup_ros_stack.sh" --driver || die "残留进程清理失败"
  else
    "$SCRIPT_DIR/cleanup_ros_stack.sh" || die "残留进程清理失败"
  fi
fi

# 残留的定位/导航进程会抢 /map 和 map->odom，且症状极难判断，宁可拒绝启动。
# /grid_mapper 和 /explore_node 也要查：重复启动的 grid_mapper 会让 /map 有两个
# 发布者（两份图交替，前沿点随机失效），而现象同样是"车走得莫名其妙"。
running_nodes="$(ros2 node list 2>/dev/null || true)"
for n in /map_server /grid_mapper /explore_node /fast_lio /super_lio \
         /super_lio_node /super_lio_bridge /relocation_node \
         /small_gicp_relocalization \
         /recovery_guard /scan_qos_relay /robot_status_monitor \
         /estop_latch /localization_watchdog /behavior_server \
         /bt_navigator /controller_server /planner_server; do
  if printf '%s\n' "$running_nodes" | grep -qx "$n"; then
    die "检测到残留节点 $n 在运行。它会和本脚本起的链抢 /map 或 map->odom，
      请先退出旧的会话（ros2 node list 看全部节点；必要时 pkill 对应进程）"
  fi
done

log "前置检查通过（包齐全、无残留定位/导航节点）"
log "模式: 驱动=$USE_DRIVER  模拟时钟=$USE_SIM_TIME  rviz=$RVIZ  探索=$EXPLORATION_MODE"
if [ "$USE_DRIVER" = false ]; then
  log "提示: 驱动已关闭，/livox/* 需要由别处提供（回放数据包时加 --clock）"
fi

# --- 3. 启动整链 -----------------------------------------------------------
LOG_DIR="$(mktemp -d)"
log "启动整链。输出同时打到本终端，并留一份在 $LOG_DIR/bringup.log"
# 用进程替换而不是直接重定向：这样 $! 仍是 launch 的 PID（便于 Ctrl-C 收尾），
# 输出也能同时留在终端上 —— 实机跑的时候看得到 launch 的日志很重要。
ros2 launch me_nav2_bringup exploration_bringup_launch.py \
  rviz:="$RVIZ" use_sim_time:="$USE_SIM_TIME" use_livox_driver:="$USE_DRIVER" \
  exploration_mode:="$EXPLORATION_MODE" \
  > >(tee "$LOG_DIR/bringup.log") 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup INT TERM

# --- 4. 启动后自检 ---------------------------------------------------------
# 整链要等 Super-LIO 收到数据并初始化（约 10~30 s），所以这里耐心等，
# 但每一项都设上限，超时就明确报出来而不是无限等。
wait_topic() {   # wait_topic <话题> <最多等几秒>
  local topic="$1" budget="${2:-60}" waited=0
  while [ "$waited" -lt "$budget" ]; do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then return 0; fi
    sleep 2; waited=$((waited + 2))
  done
  return 1
}

wait_data() {    # wait_data <话题> <最多等几秒> [<只取这个字段>]
  local topic="$1" budget="${2:-60}" field="${3:-}" waited=0
  while [ "$waited" -lt "$budget" ]; do
    if [ -n "$field" ]; then
      if [ -n "$(timeout 8 ros2 topic echo "$topic" --field "$field" --once 2>/dev/null)" ]; then
        return 0
      fi
    else
      if [ -n "$(timeout 8 ros2 topic echo "$topic" --once 2>/dev/null)" ]; then
        return 0
      fi
    fi
    waited=$((waited + 8))
  done
  return 1
}

odom_pos() {     # /lio/odom 的位置，压成一行用于比对是否变化
  timeout 10 ros2 topic echo /lio/odom --field pose.pose.position --once 2>/dev/null \
    | tr -d '\n' || true
}

grid_shape() {   # 取 /map 的 width x height，取不到打印空
  timeout 10 ros2 topic echo /map --field info --once 2>/dev/null \
    | awk '/^width:/{w=$2} /^height:/{h=$2} END{if(w&&h) print w"x"h}' || true
}

verify() {
  local problems=0

  if ! wait_topic /map 90; then
    bad "/map 一直没有出现 —— grid_mapper 没起来，或 Super-LIO 拿不到点云/里程计"
    return 1
  fi

  # 注意：话题"出现在 topic list 里"不等于"已经有数据"。grid_mapper 一启动就会
  # 声明 /map，但 Super-LIO 要到收到雷达数据、初始化完才会出里程计和点云。
  # 所以每项检查都要"等数据"，不能采样一次就下结论。

  # /map 必须只有一个发布者是 grid_mapper（map_server 若在跑会是第二个）
  local n_grid n_pub map_pub_nodes
  n_grid=$(timeout 8 ros2 topic info /map -v 2>/dev/null | grep -c 'Node name: grid_mapper' || true)
  n_pub=$(timeout 8 ros2 topic info /map 2>/dev/null | awk '/Publisher count:/{print $3}' || true)
  # 报错时点名是谁在发 —— 只说"有 2 个发布者"现场还得自己再查一遍。
  # 注意只在 Publishers 段里取（-v 输出的订阅者条目也带 "Node name:"），
  # 所以碰到 "Subscription count:" 就停。
  map_pub_nodes=$(timeout 8 ros2 topic info /map -v 2>/dev/null \
                  | awk '/^Subscription count:/{exit} /Node name:/{print $3}' \
                  | sort -u | paste -sd, - || true)
  if [ "${n_grid:-0}" -ge 1 ] && [ "${n_pub:-0}" = "1" ]; then
    ok "/map 发布者唯一：${map_pub_nodes:-grid_mapper}"
  else
    bad "/map 有 ${n_pub:-?} 个发布者：${map_pub_nodes:-取不到}
      多个发布者会让全局代价地图在两份图之间跳、前沿点随机失效。
      常见原因：残留的 map_server（上个会话没退干净），或重复启动了 grid_mapper"
    problems=$((problems + 1))
  fi

  # /lio/odom 有数据，且位姿在变（说明雷达数据真的进来了、Super-LIO 在解算）
  local p1 p2
  if wait_data /lio/odom 90 pose.pose.position; then
    p1=$(odom_pos)
    sleep 10
    p2=$(odom_pos)
    if [ -n "$p1" ] && [ "$p1" != "$p2" ]; then
      ok "/lio/odom 在更新，位姿有变化"
    else
      ok "/lio/odom 有数据（位姿 10 秒内没变 —— 车静止或数据源是静止段，属正常）"
    fi
  else
    bad "/lio/odom 一直没有数据 —— Super-LIO 没初始化。检查雷达/IMU 数据是否进来
      （实机看 /livox/lidar、/livox/imu；回放数据包时确认加了 --clock）"
    problems=$((problems + 1))
  fi

  # /map 有数据，报告尺寸（栅格大小反映已探索范围）
  if wait_data /map 30 info; then
    local s1 s2
    s1=$(grid_shape); sleep 12; s2=$(grid_shape)
    ok "/map 有数据，尺寸 $s1 -> $s2"
  else
    bad "/map 话题在但一直没有数据 —— grid_mapper 没收到点云或里程计"
    problems=$((problems + 1))
  fi

  # /scan 有数据：局部代价地图障碍层和 collision_monitor 都靠它
  if wait_data /scan 45 header; then
    ok "/scan 有数据（局部障碍层与碰撞监控有输入）"
  else
    bad "/scan 没有数据 —— 局部代价地图没有障碍物、collision_monitor 等于失效。
      多半是点云的 frame 与 TF 树不通（见 super_lio_bridge 的 _on_cloud 注释）"
    problems=$((problems + 1))
  fi

  # 安全层的锁：为 false 时自动驾驶被掐掉，车一定不会动
  local tracking
  tracking=$(timeout 10 ros2 topic echo /localization/tracking_ok --once 2>/dev/null \
             | awk '/^data:/{print $2}' || true)
  if [ "$tracking" = "true" ]; then
    ok "/localization/tracking_ok = true（自动驾驶未被安全层锁住）"
  else
    bad "/localization/tracking_ok = ${tracking:-取不到} —— 安全层会锁住自动驾驶，
      explore_lite 照常出目标但 /cmd_vel 被掐掉，表现就是车不动"
    problems=$((problems + 1))
  fi

  # 探索节点在跑
  if ros2 node list 2>/dev/null | grep -qx '/explore_node'; then
    ok "explore_node 在运行"
  else
    bad "explore_node 没起来（explore_lite 未启动或已退出）"
    problems=$((problems + 1))
  fi

  if [ "$problems" -eq 0 ]; then
    log "自检通过。探索中可用 /explore/resume 暂停/继续："
    log "  ros2 topic pub --once /explore/resume std_msgs/msg/Bool \"{data: false}\""
    log "RViz 2D Goal Pose 的 AUTO/MANUAL 切换："
    log "  手动建图: ros2 topic pub --once /exploration/mode std_msgs/msg/Bool \"{data: false}\""
    log "  恢复自主: ros2 topic pub --once /exploration/mode std_msgs/msg/Bool \"{data: true}\""
  else
    log "自检发现 $problems 项问题，详见上面 ✗ 行与 $LOG_DIR/bringup.log"
  fi
  return 0
}

# 自检放后台，不挡住 launch 的输出；主进程停掉时自检自然结束
(verify || true) &
VERIFY_PID=$!

wait "$LAUNCH_PID"
kill "$VERIFY_PID" 2>/dev/null || true
