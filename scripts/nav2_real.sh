#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

# 清理上次运行遗留的本栈节点，避免出现重复实例
# （重复的雷达驱动会让 FAST-LIO 收到交错的重复数据，导致 TF 断流、重定位波动）
for pattern in "ros2 launch livox_ros_driver2" "ros2 launch fast_lio" \
               "ros2 launch lio_interface" "ros2 launch sensor_scan_generation" \
               "ros2 launch me_nav2_bringup" "ros2 launch small_gicp_relocalization" \
               "ros2 launch nav2_safety"; do
  pkill -f "$pattern" 2>/dev/null
done
sleep 1

# 实机导航启动脚本

# ============================================================================
# 底盘侧：CAN 总线 + BUNKER MINI 2.0 底盘驱动
# 底盘是 /cmd_vel 的实际执行者，并通过 CAN 把轮式里程计发到 /wheel_odom。
# 必须最先启动。CAN 配置需要 root 权限，此处放在前台执行，
# 以便 sudo 密码提示能在当前终端同步处理后再往下走。
# ============================================================================
if ! bash "$SCRIPT_DIR/chassis_start.sh" --can-only; then
  echo "[nav2_real] CAN 配置失败，底盘与导航都无法工作，已中止。" >&2
  exit 1
fi

gnome-terminal --title="BUNKER 底盘驱动" -- bash -c "
bash \"$SCRIPT_DIR/chassis_start.sh\""
sleep 5   # 等待底盘驱动建立 CAN 连接并开始发布 /wheel_odom

# point_lio
# gnome-terminal --title="Livox Point-LIO 驱动" -- bash -c "
# source install/setup.bash;
# ros2 launch livox_ros_driver2 point_lio_msg_MID360_launch.py"

# gnome-terminal --title="Point-LIO 里程计" -- bash -c "
# source install/setup.bash;
# ros2 launch point_lio point_lio.launch.py use_sim_time:=false \
#   point_lio_cfg_dir:="$WORKSPACE_ROOT/src/localization/point_lio/config/mid360_real.yaml"

# gnome-terminal --title="Point-LIO lio_interface" -- bash -c "
# source install/setup.bash;
# ros2 launch lio_interface pointlio_lio_interface_launch.py use_sim_time:=false"


# Start static transforms before the LIO interface needs base_footprint -> livox_frame.
gnome-terminal --title="机器人描述" -- bash -c "
source install/setup.bash;
ros2 launch gld_robot_description gld_robot_description_launch.py rviz:=false"
sleep 2

# fast_lio
gnome-terminal --title="Livox Fast-LIO 驱动" -- bash -c "
source install/setup.bash;
ros2 launch livox_ros_driver2 fast_lio_msg_MID360_launch.py rviz:=false"
sleep 2

gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
source install/setup.bash;
ros2 launch fast_lio mapping.launch.py use_sim_time:=false rviz:=false"
sleep 3

gnome-terminal --title="Fast-LIO lio_interface" -- bash -c "
source install/setup.bash;
ros2 launch lio_interface fastlio_lio_interface_launch.py use_sim_time:=false"
sleep 2

# ---------

gnome-terminal --title="sensor_scan_generation" -- bash -c "
source install/setup.bash;
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py use_sim_time:=false"
sleep 2

gnome-terminal --title="3d点云转2d" -- bash -c "
source install/setup.bash;
ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch.py use_sim_time:=false"
sleep 2

gnome-terminal --title="small_gicp 重定位" -- bash -c "
source install/setup.bash
ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py"
sleep 3

#gnome-terminal --title="KISS + GICP 重定位" -- bash -c "
#source install/setup.bash;
#ros2 launch global_relocalization_kiss_matcher global_kiss_matcher_relocalization_launch.py"

gnome-terminal --title="Nav2 + 安全层" -- bash -c "
source install/setup.bash;
ros2 launch nav2_safety safe_nav_bringup_launch.py use_sim_time:=false"
