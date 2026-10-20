#!/usr/bin/env bash
#
# TEMPORARY: Super-LIO exploration run + full reproduction snapshot.
#
# Starts the exploration stack, records a rosbag of the key topics, and saves
# configs / parameters / git state / logs into one timestamped directory.
#
# Usage:
#   ./scripts/explore_record.sh                 # real robot + RViz, starts Livox driver
#   ./scripts/explore_record.sh --no-rviz       # real robot, without RViz
#   ./scripts/explore_record.sh --bag <bagdir>  # replay a rosbag (sim time)
#   ./scripts/explore_record.sh --bag <bagdir> --rate 0.5 --duration 120
#
# Options:
#   --rviz / --no-rviz      start RViz (default) / don't start RViz
#   --manual / --auto       exploration mode (default auto); manual sends the
#                           RViz 2D Goal Pose straight to Nav2
#   --bag <path>            replay this bag instead of using the Livox driver
#   --rate <r>              bag playback rate (default 1.0)
#   --duration <sec>        stop automatically after N seconds (default: wait)
#   --raw / --no-raw        force recording of /livox/lidar + /livox/imu
#   --save-maps             save 2D grid map + 3D point cloud on exit (default)
#   --no-save-maps          don't save maps on exit
#   --output <dir>          run directory (default: <workspace>/runs/explore_<time>)
#   --no-clean              don't kill residual nodes from a previous run
#
# Real robot: run ./scripts/chassis_start.sh first (separate terminal).
# Reproduce:  the run directory contains rosbag/, config/, params_*.yaml,
#             metadata.yaml, launch.log, nodes.txt, topics.txt and README.txt.
#             If --save-maps (default), maps/ will contain:
#               - map.pgm + map.yaml (2D grid map for Nav2)
#               - global_map.pcd (Super-LIO 3D point cloud for relocalization)
#
# This script only kills the processes it started. The chassis driver in the
# other terminal is left alone.

set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(dirname -- "$SCRIPT_DIR")"
SUPERLIO_WS="${SUPERLIO_WS:-/home/demo/superlio_ws/Super-LIO-ros2}"
ORIG_ARGS="$*"

RVIZ=true
BAG=""
RATE=1.0
DURATION=0
OUTPUT=""
RECORD_RAW=""
SAVE_MAPS=false
CLEAN_RESIDUAL=true
# 默认探索模式，直接改这里即可：auto / manual。命令行 --manual/--auto 会覆盖它。
EXPLORATION_MODE=manual

usage() {
  sed -n '2,33p' "$0" | sed 's/^# \{0,1\}//'
}

while [ $# -gt 0 ]; do
  case "$1" in
    --rviz) RVIZ=true; shift ;;
    --no-rviz) RVIZ=false; shift ;;
    --manual) EXPLORATION_MODE=manual; shift ;;
    --auto) EXPLORATION_MODE=auto; shift ;;
    --bag) BAG="${2:?--bag needs a path}"; shift 2 ;;
    --rate) RATE="${2:?--rate needs a number}"; shift 2 ;;
    --duration) DURATION="${2:?--duration needs seconds}"; shift 2 ;;
    --raw) RECORD_RAW=1; shift ;;
    --no-raw) RECORD_RAW=0; shift ;;
    --save-maps) SAVE_MAPS=true; shift ;;
    --no-save-maps) SAVE_MAPS=false; shift ;;
    --no-clean) CLEAN_RESIDUAL=false; shift ;;
    --output) OUTPUT="${2:?--output needs a path}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
  esac
done

if [ -n "$BAG" ]; then
  USE_DRIVER=false
  USE_SIM_TIME=true
  [ -z "$RECORD_RAW" ] && RECORD_RAW=0
else
  USE_DRIVER=true
  USE_SIM_TIME=false
  [ -z "$RECORD_RAW" ] && RECORD_RAW=1
fi

log()  { printf '[run] %s\n' "$*"; }
die()  { printf '[run] ERROR: %s\n' "$*" >&2; exit 1; }

if [ "$RVIZ" != false ] && [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  log "WARNING: DISPLAY/WAYLAND_DISPLAY is unset; RViz may fail to open."
  log "         Run from a desktop terminal, or export DISPLAY=:0 before starting."
fi

# ------------------------------------------------------------------ environment
if [ ! -f "$SUPERLIO_WS/install/setup.bash" ]; then
  die "Super-LIO install/setup.bash not found at $SUPERLIO_WS"
fi
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WORKSPACE/install/setup.bash"
# shellcheck disable=SC1091
source "$SUPERLIO_WS/install/setup.bash"
set -u

for pkg in super_lio super_lio_bridge grid_mapper explore_lite me_nav2_bringup nav2_safety; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 \
    || die "package '$pkg' not found (build/source both workspaces first)"
done

if [ "$USE_DRIVER" = true ]; then
  if ! timeout 3 ros2 topic echo /wheel_odom --once >/dev/null 2>&1; then
    log "WARNING: /wheel_odom 没有数据；请确认 chassis_start.sh 已启动且底盘已上电。"
  fi
fi

# 默认清掉上一次运行残留的同一套进程。旧 run 留下的孤儿节点会让 /map、
# /scan、/cmd_vel 出现两个发布者，症状是"车走得莫名其妙"。--no-clean 可跳过。
if [ "$CLEAN_RESIDUAL" = true ]; then
  log "自动清理上次运行残留的 ROS 进程..."
  if [ "$USE_DRIVER" = true ]; then
    "$SCRIPT_DIR/cleanup_ros_stack.sh" --driver || die "残留进程清理失败"
  else
    "$SCRIPT_DIR/cleanup_ros_stack.sh" || die "残留进程清理失败"
  fi
fi

# 清理后仍残留就拒绝启动，避免两套栈同时抢话题。
running_nodes="$(ros2 node list 2>/dev/null || true)"
for node in /map_server /grid_mapper /explore_node /exploration_ui \
            /super_lio /super_lio_node /super_lio_bridge /relocation_node \
            /fast_lio /small_gicp_relocalization \
            /recovery_guard /scan_qos_relay /robot_status_monitor \
            /estop_latch /localization_watchdog /behavior_server \
            /bt_navigator /controller_server /planner_server; do
  if printf '%s\n' "$running_nodes" | grep -qx "$node"; then
    die "residual node $node is running; stop the old session first"
  fi
done

if [ -n "$BAG" ] && [ ! -d "$BAG" ]; then
  die "bag directory not found: $BAG"
fi

# ------------------------------------------------------------------ run directory
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${OUTPUT:-$WORKSPACE/runs/explore_$STAMP}"
mkdir -p "$RUN_DIR/config"
LAUNCH_LOG="$RUN_DIR/launch.log"
RECORD_LOG="$RUN_DIR/record.log"
PLAY_LOG="$RUN_DIR/bag_play.log"

{
  echo "date: $(date -Is)"
  echo "hostname: $(hostname)"
  echo "uname: $(uname -a)"
  echo "nproc: $(nproc)"
  echo "memory: $(awk '/MemTotal/{printf "%.1f GiB", $2/1024/1024}' /proc/meminfo 2>/dev/null)"
  echo "disk: $(df -h "$RUN_DIR" | awk 'NR==2{print $2" total, "$4" free"}')"
  echo "ros_distro: ${ROS_DISTRO:-unknown}"
  echo "ros_domain_id: ${ROS_DOMAIN_ID:-0}"
  echo "command: $0 $ORIG_ARGS"
  echo "mode: $([ -n "$BAG" ] && echo bag_replay || echo real_robot)"
  echo "use_driver: $USE_DRIVER"
  echo "use_sim_time: $USE_SIM_TIME"
  echo "rviz: $RVIZ"
  echo "record_raw: $RECORD_RAW"
  echo "save_maps: $SAVE_MAPS"
  echo "source_bag: ${BAG:-<none>}"
  for ws in "$WORKSPACE" "$SUPERLIO_WS"; do
    echo "workspace: $ws"
    if git -C "$ws" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
      echo "  commit: $(git -C "$ws" rev-parse HEAD)"
      echo "  branch: $(git -C "$ws" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
      echo "  status:"
      git -C "$ws" status --short 2>/dev/null | sed 's/^/    /'
    else
      echo "  git: not a repository"
    fi
  done
} > "$RUN_DIR/metadata.yaml"

# Snapshot the configs that determine the run.
copy_if_exists() {
  local src="$1"
  [ -f "$src" ] && cp "$src" "$RUN_DIR/config/" 2>/dev/null || true
}
copy_if_exists "$WORKSPACE/src/me_nav2_bringup/config/nav2_params.yaml"
copy_if_exists "$WORKSPACE/src/me_nav2_bringup/config/explore_params.yaml"
copy_if_exists "$WORKSPACE/src/me_nav2_bringup/config/Pointcloud2d_3d.yaml"
copy_if_exists "$WORKSPACE/src/me_nav2_bringup/config/exploration_bounds.yaml"
copy_if_exists "$WORKSPACE/src/super_lio/super_lio_bridge/config/bridge.yaml"
copy_if_exists "$WORKSPACE/src/super_lio/super_lio_bridge/config/super_lio_mid360_mapping.yaml"
copy_if_exists "$WORKSPACE/src/grid_mapper/config/grid_mapper.yaml"
copy_if_exists "$WORKSPACE/src/nav2_safety/config/collision_monitor.yaml"
copy_if_exists "$WORKSPACE/src/nav2_safety/config/twist_mux.yaml"
copy_if_exists "$WORKSPACE/src/nav2_safety/config/deadman.yaml"
copy_if_exists "$WORKSPACE/src/nav2_safety/config/estop_latch.yaml"
copy_if_exists "$WORKSPACE/src/nav2_safety/config/localization_watchdog.yaml"
copy_if_exists "$WORKSPACE/src/nav2_safety/config/recovery_guard.yaml"
copy_if_exists "$WORKSPACE/src/nav2_safety/config/robot_status_monitor.yaml"
copy_if_exists "$WORKSPACE/src/nav2_safety/config/scan_qos_relay.yaml"
copy_if_exists "$WORKSPACE/src/me_nav2_bringup/behavior_trees/navigate_w_replanning_and_exploration_recovery.xml"
copy_if_exists "$WORKSPACE/src/me_nav2_bringup/rviz/nav2.rviz"
cp "$0" "$RUN_DIR/launch_script.sh" 2>/dev/null || true

if [ -n "$BAG" ]; then
  ros2 bag info "$BAG" > "$RUN_DIR/source_bag_info.txt" 2>&1 || true
fi

cat > "$RUN_DIR/README.txt" <<'EOF'
Super-LIO exploration run snapshot
==================================

rosbag/              recorded topics (see topics.txt), zstd-compressed per message.
                     ros2 bag play/info handle it transparently; code that reads
                     the .db3 directly must zstd-decompress each message first.
config/              YAML snapshots used for this run
params_*.yaml        live ros2 param dump of key nodes
launch.log           stack output
record.log           rosbag recorder output
bag_play.log         bag player output (bag replay mode only)
metadata.yaml        date, workspaces, git commits/status, modes
nodes.txt            nodes seen after startup
topics.txt           topics seen after startup
final_bounds.yaml    exploration boundary saved at shutdown (if any)
maps/                saved maps (if --save-maps enabled):
  map.pgm + map.yaml   2D grid map for Nav2 navigation
  global_map.pcd       Super-LIO 3D point cloud for relocalization

Reproduce a bag replay:
  SLIO_NO_DRIVER=1 SLIO_USE_SIM_TIME=1 ./scripts/super_lio_explore.sh
  ros2 bag play <source_bag> --clock --rate 1.0

Replay this run's outputs only:
  ros2 bag play rosbag
EOF

# ------------------------------------------------------------------ recording
TOPICS=(
  /clock /rosout
  /tf /tf_static
  /lio/odom /lio/imu/odom /lio/robo/odom /lio/path /lio/cloud_world
  # /registered_scan 是 /lio/cloud_world 原样转发的同一份消息（实测每帧都是
  # 190.5 KB），录两份白占约 13% 体积，所以不录。
  /scan
  /odom /wheel_odom
  /plan /local_plan /received_global_plan
  /map /map_updates
  /global_costmap/costmap /global_costmap/costmap_updates
  /local_costmap/costmap /local_costmap/costmap_updates
  /cmd_vel /cmd_vel_nav /cmd_vel_nav_smoothed /cmd_vel_nav_checked
  /cmd_vel_muxed /cmd_vel_teleop
  /recovery_path /recovery_candidates /safety/recovery_active
  /cmd_vel_recovery /cmd_vel_recovery_checked
  /smart_recovery/_action/status /backup/_action/status /spin/_action/status
  /wait/_action/status /drive_on_heading/_action/status
  /follow_path/_action/status /compute_path_to_pose/_action/status
  /navigate_to_pose/_action/feedback
  /safety/recovery_blocked /safety/recovery_status /safety/watchdog_status
  /safety/deadman_status /safety/deadman_stopping /robot/status
  /navigate_to_pose/_action/status
  /explore/status /explore/frontiers /explore/resume
  /exploration/mode /exploration/enabled /exploration/input_goal
  /exploration/bounds /exploration/guide
  /exploration/bounds_markers /exploration/guide_markers
  /localization/tracking_ok /localization/tracking_status
  /safety/autonomy_blocked /safety/estop_active
)
if [ "$RECORD_RAW" = 1 ]; then
  TOPICS+=(/livox/lidar /livox/imu)
fi

log "run directory: $RUN_DIR"
log "starting rosbag recorder (raw=$RECORD_RAW)"
# 逐条消息 zstd 压缩：/map 和代价地图每帧 2 MB 以上，绝大部分是重复的栅格值，
# 实测能压到原来的 1/30 ~ 1/130；点云几乎压不动（约 1.4 倍），但开销也小。
# 用 message 而不是 file 模式：录制时边写边压，停止时不用等整包压缩，
# 目录也仍然可以直接 ros2 bag play。
#
# 压缩队列：默认只有 2，实测 /map + /lio/cloud_world 同时写入时 56 s 就丢了
# 2445 条（rosout、recovery_candidates 等），所以把队列提到 64、开 2 个压缩线程。
# max-cache-size 提到 200 MiB，给写入磁盘留出更多缓冲（双缓冲，最坏约 400 MiB）。
ros2 bag record -o "$RUN_DIR/rosbag" \
  --compression-mode message --compression-format zstd \
  --max-cache-size 209715200 \
  --compression-queue-size 64 --compression-threads 2 \
  "${TOPICS[@]}" > "$RECORD_LOG" 2>&1 &
REC_PID=$!
sleep 2

# ------------------------------------------------------------------ launch stack
log "starting exploration stack (driver=$USE_DRIVER sim_time=$USE_SIM_TIME rviz=$RVIZ)"
ros2 launch me_nav2_bringup exploration_bringup_launch.py \
  rviz:="$RVIZ" use_sim_time:="$USE_SIM_TIME" use_livox_driver:="$USE_DRIVER" \
  exploration_mode:="$EXPLORATION_MODE" \
  > >(tee "$LAUNCH_LOG") 2>&1 &
LAUNCH_PID=$!

PLAY_PID=""
if [ -n "$BAG" ]; then
  sleep 6
  log "playing bag: $BAG (rate=$RATE)"
  ros2 bag play "$BAG" --clock --rate "$RATE" > "$PLAY_LOG" 2>&1 &
  PLAY_PID=$!
fi

log "recording. Draw boundaries / send guides in RViz, then Ctrl-C to stop."
log "  commit boundary: ros2 service call /exploration_ui/commit std_srvs/srv/Trigger"
log "  clear boundary:  ros2 service call /exploration_ui/clear std_srvs/srv/Trigger"
log "  clear guide:     ros2 service call /exploration_ui/clear_guide std_srvs/srv/Trigger"

# ------------------------------------------------------------------ shutdown
CLEANED=0
cleanup() {
  [ "$CLEANED" = 1 ] && return
  CLEANED=1
  trap - INT TERM EXIT
  log "stopping; saving boundary and bag ..."

  timeout 5 ros2 service call /exploration_ui/save std_srvs/srv/Trigger \
    > /dev/null 2>&1 || true
  [ -f "$WORKSPACE/src/me_nav2_bringup/config/exploration_bounds.yaml" ] && \
    cp "$WORKSPACE/src/me_nav2_bringup/config/exploration_bounds.yaml" \
       "$RUN_DIR/final_bounds.yaml" 2>/dev/null || true

  # 保存地图（2D 栅格图 + 3D 点云图）
  if [ "$SAVE_MAPS" = true ]; then
    log "saving maps to $RUN_DIR/maps/ ..."
    mkdir -p "$RUN_DIR/maps"

    # 1. 保存 2D 栅格地图（用于 Nav2 全局规划）
    if timeout 10 ros2 topic echo /map --once > /dev/null 2>&1; then
      log "  - saving 2D grid map (map.pgm + map.yaml)"
      timeout 30 ros2 run nav2_map_server map_saver_cli \
        -f "$RUN_DIR/maps/map" \
        --ros-args -p save_map_timeout:=10000.0 \
        > "$RUN_DIR/maps/map_saver.log" 2>&1 || \
        log "    WARNING: map_saver_cli failed (see maps/map_saver.log)"
    else
      log "  - WARNING: /map topic has no data, skipping 2D map save"
    fi

    # 2. 保存 Super-LIO 的 3D 点云地图（用于重定位）
    # Super-LIO 配置了 save_map=true，退出时会自动保存到
    # <Super-LIO源码目录>/src/super_lio/map/map_explore.pcd
    SUPERLIO_MAP_DIR="${SUPERLIO_WS}/src/super_lio/map"
    SUPERLIO_MAP_FILE="map_explore.pcd"

    log "  - waiting for Super-LIO to save point cloud map..."
    sleep 3  # 给 Super-LIO 一点时间保存地图

    if [ -f "$SUPERLIO_MAP_DIR/$SUPERLIO_MAP_FILE" ]; then
      log "  - copying Super-LIO point cloud map: $SUPERLIO_MAP_FILE"
      cp "$SUPERLIO_MAP_DIR/$SUPERLIO_MAP_FILE" "$RUN_DIR/maps/global_map.pcd" && \
        log "    saved as: maps/global_map.pcd ($(du -h "$RUN_DIR/maps/global_map.pcd" | cut -f1))" || \
        log "    WARNING: failed to copy map"
    else
      log "  - WARNING: Super-LIO map not found at $SUPERLIO_MAP_DIR/$SUPERLIO_MAP_FILE"
      log "    (check lio.map.save_map and lio.map.map_name in super_lio config)"
    fi

    log "  maps saved to: $RUN_DIR/maps/"
  fi

  [ -n "$PLAY_PID" ] && kill -INT "$PLAY_PID" 2>/dev/null || true
  kill -INT "$REC_PID" 2>/dev/null || true

  # Collect the launch tree BEFORE stopping the launch, otherwise children may
  # be reparented to init and escape cleanup. Only this tree is touched; the
  # chassis driver in another terminal is left alone.
  python3 - "$LAUNCH_PID" <<'PY' 2>/dev/null || true
import os, signal, sys, time
root = int(sys.argv[1])
def children(pid):
    out = []
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        try:
            with open(f'/proc/{name}/stat', 'r') as f:
                fields = f.read().split()
            if int(fields[3]) == pid:
                out.append(int(name))
        except (OSError, ValueError, IndexError):
            pass
    return out
def tree(pid):
    result = []
    for child in children(pid):
        result.append(child)
        result.extend(tree(child))
    return result
pids = tree(root)
for pid in pids:
    try:
        os.kill(pid, signal.SIGINT)
    except OSError:
        pass
time.sleep(5)
for pid in pids + [root]:
    try:
        os.kill(pid, 0)
        os.kill(pid, signal.SIGTERM)
    except OSError:
        pass
time.sleep(2)
for pid in pids + [root]:
    try:
        os.kill(pid, signal.SIGKILL)
    except OSError:
        pass
PY

  wait "$REC_PID" 2>/dev/null || true
  [ -n "$PLAY_PID" ] && wait "$PLAY_PID" 2>/dev/null || true
  if [ -d "$RUN_DIR/rosbag" ]; then
    ros2 bag info "$RUN_DIR/rosbag" > "$RUN_DIR/bag_info.txt" 2>&1 || true
  fi
  log "run complete: $RUN_DIR"
  du -sh "$RUN_DIR" 2>/dev/null || true
}

trap cleanup INT TERM EXIT

# Register shutdown handling before the startup wait / parameter snapshots:
# Ctrl-C during this longer diagnostic phase must still stop this run's nodes.
sleep 12
for node in /super_lio_node /super_lio_bridge /grid_mapper \
            /explore_node /exploration_ui /controller_server /planner_server \
            /global_costmap/global_costmap /local_costmap/local_costmap \
            /behavior_server /bt_navigator /velocity_smoother /collision_monitor \
            /recovery_guard /twist_mux /cmd_vel_deadman /localization_watchdog; do
  name="$(printf '%s' "$node" | tr '/' '_' | sed 's/^_//')"
  timeout 8 ros2 param dump "$node" > "$RUN_DIR/params_${name}.yaml" 2>/dev/null || true
done
ros2 node list > "$RUN_DIR/nodes.txt" 2>/dev/null || true
ros2 topic list > "$RUN_DIR/topics.txt" 2>/dev/null || true

if [ "$DURATION" -gt 0 ]; then
  sleep "$DURATION"
  exit 0
fi

wait "$LAUNCH_PID"
