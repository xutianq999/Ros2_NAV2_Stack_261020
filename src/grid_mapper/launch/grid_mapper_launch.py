# 只起栅格节点本身。它把 Super-LIO 的点云变成 /map，取代 map_server。
#
# 探索模式的完整顺序：
#   1) Super-LIO（建图模式）   ./scripts/super_lio_nav.sh mapping
#   2) 底盘                    ./scripts/chassis_start.sh
#   3) 本文件                  ros2 launch grid_mapper grid_mapper_launch.py
#   4) 2D 扫描                 ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch.py
#   5) Nav2 + 安全层           Nav2 用**不带 map_server** 的方式启动（见 README）
#   6) 探索                    ros2 launch me_nav2_bringup exploration_launch.py
#
# 第 5 步是关键：map_server 和本节点都发 /map，同时跑会让全局代价地图在
# 静态先验图和实时生长图之间来回跳。探索模式必须只用本节点这一张图。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('grid_mapper'),
        'config',
        'grid_mapper.yaml',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    grid_mapper_node = Node(
        package='grid_mapper',
        executable='grid_mapper',
        name='grid_mapper',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        grid_mapper_node,
    ])
