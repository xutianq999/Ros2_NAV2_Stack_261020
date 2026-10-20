# Super-LIO -> Nav2 桥接（单独启动）
#
# 用于 Super-LIO 已在别处跑起来的情况。它会：
#   world -> imu  拆成  map -> odom  +  odom -> base_footprint
#   转发点云到 /registered_scan，并发布 /localization/tracking_ok
# 详见 super_lio_bridge/tf_bridge.py 的模块说明。
#
# 前置：必须已 source Super-LIO 所在工作空间（否则 /lio/odom 不会有数据）。
# 要改参数（比如 imu_to_base）直接编辑 config/bridge.yaml。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('super_lio_bridge')

    declare_config = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(share, 'config', 'bridge.yaml'),
        description='桥接参数文件')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='回放 bag 时必须为 true，且 bag 要用 --clock')
    # internal=Super-LIO 自己负责 map 系定位；external=只当里程计，
    # map->odom 交给外部重定位节点（small_gicp_relocalization）。
    declare_localization_mode = DeclareLaunchArgument(
        'localization_mode', default_value='internal',
        description='internal / external')

    bridge = Node(
        package='super_lio_bridge',
        executable='tf_bridge',
        name='super_lio_bridge',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'localization_mode': LaunchConfiguration('localization_mode'),
            },
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_config)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_localization_mode)
    ld.add_action(bridge)
    return ld
