#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

# 实机建图启动脚本

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


# fast_lio
gnome-terminal --title="Livox Fast-LIO 驱动" -- bash -c "
source install/setup.bash;
ros2 launch livox_ros_driver2 fast_lio_msg_MID360_launch.py"

gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
source install/setup.bash;
ros2 launch fast_lio mapping.launch.py use_sim_time:=false"

gnome-terminal --title="Fast-LIO lio_interface" -- bash -c "
source install/setup.bash;
ros2 launch lio_interface fastlio_lio_interface_launch.py use_sim_time:=false"

# ---------


gnome-terminal --title="机器人描述" -- bash -c "
source install/setup.bash;
ros2 launch gld_robot_description gld_robot_description_launch.py"

gnome-terminal --title="sensor_scan_generation" -- bash -c "
source install/setup.bash;
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py use_sim_time:=false"

gnome-terminal --title="3d点云转2d" -- bash -c "
source install/setup.bash;
ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch.py use_sim_time:=false"

# 安全层：建图时没有 Nav2，所以关掉 collision_monitor，只保留仲裁/兜底/看门狗。
# 遥控话题已迁到 /cmd_vel_teleop，需要这一层来把它转到底盘订阅的 /cmd_vel。
# 注意：用遥控器驾驶时请把遥控器 SWB 拨到"遥控模式"，此时底盘由遥控器控制，
# 不参与 ROS 指令（BUNKER 手册 2.2）。
gnome-terminal --title="安全层" -- bash -c "
source install/setup.bash;
ros2 launch nav2_safety safety_layer_launch.py \
    use_sim_time:=false use_collision_monitor:=false"

# gnome-terminal --title="slam_toolbox 建图" -- bash -c "
# source install/setup.bash;
# ros2 launch slam_toolbox online_async_launch.py"

gnome-terminal --title="slam_toolbox 建图" -- bash -c "
source install/setup.bash;
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="$WORKSPACE_ROOT/src/me_nav2_bringup/config/slam_toolbox_params.yaml" \
    use_sim_time:=false"
