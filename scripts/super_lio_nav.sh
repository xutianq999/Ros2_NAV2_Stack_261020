#!/usr/bin/env bash
#
# 用 Super-LIO 跑导航栈（替代 FAST-LIO + lio_interface + sensor_scan_generation
# + small_gicp 重定位那一整条链）。
#
# 用法：
#   ./scripts/super_lio_nav.sh                 # 跟踪模式 + RViz（默认）
#   ./scripts/super_lio_nav.sh tracking        # Super-LIO 里程计 + small_gicp 持续重定位
#   ./scripts/super_lio_nav.sh tracking false  # 同上，不开 RViz
#   ./scripts/super_lio_nav.sh tracking_kiss   # KISS 全局初始化 + GICP 跟踪 + 失败自动恢复
#   ./scripts/super_lio_nav.sh relocation      # Super-LIO 自带的“只初始化一次”重定位
#   ./scripts/super_lio_nav.sh mapping         # 建图模式（退出时存地图）
#   ./scripts/super_lio_nav.sh exploration     # 自主探索（整链：见下）
#   SLIO_NO_DRIVER=1 ./scripts/super_lio_nav.sh exploration   # 无硬件，回放数据包
#
# 启动前默认会清掉上一次运行残留的同一套 ROS 进程；想保留它们用：
#   SLIO_NO_CLEAN=1 ./scripts/super_lio_nav.sh ...
# 底盘驱动（chassis_start.sh）不会被清理。
#
# exploration 模式一次起完：Super-LIO(建图) + 桥接 + 2D扫描 + Nav2(关掉 map_server)
# + 安全层 + grid_mapper(实时占据图) + explore_lite(前沿探索)。底盘仍由
# ./scripts/chassis_start.sh 单独负责。关掉 map_server 是关键 —— 探索时 /map 要由
# grid_mapper 随探索进程生长，静态先验图没有会变多的未知区域。
#
# 前置：Super-LIO 必须已经编译过（它是独立的工作空间，见 README）。
# 底盘（CAN + BUNKER 驱动）由 scripts/nav2_real.sh 负责，本脚本不管底盘 ——
# 需要底盘时就先跑 ./scripts/nav2_real.sh，但它会同时起 FAST-LIO 那套，
# 二者不能同时跑；用 Super-LIO 时请单独用 ./scripts/chassis_start.sh 起底盘。

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
SUPERLIO_WS="${SUPERLIO_WS:-/home/demo/superlio_ws/Super-LIO-ros2}"

MODE="${1:-tracking}"
# 默认打开 RViz；不需要时显式传 false: ./scripts/super_lio_nav.sh tracking false
RVIZ="${2:-true}"
# 回放数据包验证时置 1：不起 Livox 驱动，避免驱动与 bag 争 /livox/*
USE_DRIVER=true
if [ "${SLIO_NO_DRIVER:-0}" = "1" ]; then
  USE_DRIVER=false
fi
# 回放旧数据包时置 1（并给 ros2 bag play 加 --clock）：让所有节点用 bag 的时间戳。
# 否则节点走墙钟、TF 走录制时间，两者差几个月，TF 查找全部失配 —— explore_lite
# 拿不到机器人位姿、controller 转不了 map->odom、/scan 会被 message filter 丢掉。
# 实机上一律用 false（=墙钟）。
USE_SIM_TIME=false
if [ "${SLIO_USE_SIM_TIME:-0}" = "1" ]; then
  USE_SIM_TIME=true
fi

if [ "$MODE" != "mapping" ] && [ "$MODE" != "relocation" ] \
   && [ "$MODE" != "tracking" ] && [ "$MODE" != "tracking_kiss" ] \
   && [ "$MODE" != "exploration" ]; then
  echo "错误: 模式只能是 mapping / relocation / tracking / tracking_kiss / exploration，收到 '$MODE'" >&2
  exit 1
fi

# Super-LIO 是独立工作空间，必须已编译
if [ ! -f "$SUPERLIO_WS/install/setup.bash" ]; then
  echo "错误: 未找到 Super-LIO 的 install/setup.bash" >&2
  echo "      期望路径: $SUPERLIO_WS" >&2
  echo "      请先编译：" >&2
  echo "        source /opt/ros/humble/setup.bash" >&2
  echo "        source $WORKSPACE_ROOT/install/setup.bash   # 提供 livox_ros_driver2" >&2
  echo "        cd $SUPERLIO_WS && colcon build --symlink-install" >&2
  exit 1
fi

cd "$WORKSPACE_ROOT"

# ROS 的 setup.bash 会引用未定义的 COLCON_TRACE，与 set -u 冲突
set +u
source /opt/ros/humble/setup.bash
# 先本工作空间（提供 livox_ros_driver2 / nav2_safety / super_lio_bridge），
# 再叠加 Super-LIO 的工作空间
source install/setup.bash
source "$SUPERLIO_WS/install/setup.bash"
set -u

if ! ros2 pkg prefix super_lio >/dev/null 2>&1; then
  echo "错误: source 之后仍找不到 super_lio 包，请检查 $SUPERLIO_WS 是否编译成功" >&2
  exit 1
fi

# 重定位模式先确认先验地图在不在。
# 地图路径是 <super_lio 源码目录>/map/map.pcd —— Super-LIO 把 save_map_dir 参数
# 作为**后缀**拼在编译期宏 ROOT（包源码目录）之后，所以它不是绝对路径。
# 缺地图时它只会在日志里刷 "Load map failed" 然后空转，人容易看糊；这里提前报错。
SLIO_MAP="$SUPERLIO_WS/src/super_lio/map/map.pcd"
if [ "$MODE" = "relocation" ] && [ ! -f "$SLIO_MAP" ]; then
  echo "错误: 找不到先验地图 $SLIO_MAP" >&2
  echo "      重定位模式需要先建图。先跑一次：" >&2
  echo "        ./scripts/super_lio_nav.sh mapping" >&2
  echo "      建图后正常退出，地图会合并保存到上面那个路径。" >&2
  exit 1
fi

# tracking 模式的重定位由 small_gicp_relocalization 负责，它读的是
# map_config.yaml 里的 prior_pcd_file（不是 Super-LIO 的 map/map.pcd）。
# 这里提前解析并检查，避免启动后才在节点日志里看到 "Couldn't read PCD file"。
PRIOR_PCD=""
if [ "$MODE" = "tracking" ] || [ "$MODE" = "tracking_kiss" ]; then
  PRIOR_PCD="$(python3 - "$WORKSPACE_ROOT" <<'PY'
import os
import sys
import yaml

ws = sys.argv[1]
cfg = os.path.join(ws, 'src', 'me_nav2_bringup', 'config', 'map_config.yaml')
with open(cfg) as f:
    path = yaml.safe_load(f)['prior_pcd_file']
print(path if os.path.isabs(path) else
      os.path.join(os.path.dirname(os.path.dirname(cfg)), path))
PY
)"
  if [ ! -f "$PRIOR_PCD" ]; then
    echo "错误: $MODE 模式的先验点云不存在: $PRIOR_PCD" >&2
    echo "      请检查 src/me_nav2_bringup/config/map_config.yaml 的 prior_pcd_file" >&2
    exit 1
  fi
fi

echo ">>> Super-LIO 模式: $MODE  (rviz=$RVIZ, 驱动=$USE_DRIVER, 模拟时钟=$USE_SIM_TIME)"
echo ">>> 底盘需另行启动: ./scripts/chassis_start.sh"
if [ "$MODE" = "relocation" ]; then
  echo ">>> 先验地图: $SLIO_MAP"
elif [ "$MODE" = "tracking" ]; then
  echo ">>> Super-LIO 只做里程计；small_gicp 2Hz 持续修正 map->odom"
  echo ">>> 先验点云: $PRIOR_PCD"
elif [ "$MODE" = "tracking_kiss" ]; then
  echo ">>> Super-LIO 只做里程计；KISS 全局初始化 + GICP 跟踪 + 失败自动恢复"
  echo ">>> 先验点云: $PRIOR_PCD"
elif [ "$MODE" = "mapping" ]; then
  echo ">>> 建图结束后地图将写到: $SLIO_MAP"
else
  echo ">>> 探索：/map 由 grid_mapper 实时生长（不加载先验图）"
  echo ">>> 运行中可用 /explore/resume (std_msgs/Bool) 暂停/继续探索"
fi

# 默认清掉上一次运行残留的同一套进程，避免旧的安全层/Nav2 节点抢话题。
# 当前脚本及其祖先进程由 cleanup_ros_stack.sh 自动排除；底盘驱动不在清理范围。
if [ "${SLIO_NO_CLEAN:-0}" != "1" ]; then
  echo ">>> 自动清理上次运行残留的 ROS 进程..."
  if [ "$USE_DRIVER" = true ]; then
    "$SCRIPT_DIR/cleanup_ros_stack.sh" --driver || exit 1
  else
    "$SCRIPT_DIR/cleanup_ros_stack.sh" || exit 1
  fi
fi

if [ "$MODE" = "exploration" ]; then
  # exploration 要串起三个 launch（Super-LIO 那条链 + 栅格 + 探索），
  # 所以走 me_nav2_bringup 里的组合 launch，而不是直接起 super_lio_bringup。
  exec ros2 launch me_nav2_bringup exploration_bringup_launch.py \
    rviz:="$RVIZ" use_sim_time:="$USE_SIM_TIME" use_livox_driver:="$USE_DRIVER"
fi

exec ros2 launch super_lio_bridge super_lio_bringup_launch.py \
  mode:="$MODE" rviz:="$RVIZ" use_sim_time:="$USE_SIM_TIME" use_livox_driver:="$USE_DRIVER"
