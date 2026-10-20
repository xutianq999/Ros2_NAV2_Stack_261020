"""BUNKER MINI 2.0 base-driver adapter for the real-robot navigation stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bunker_launch = PythonLaunchDescriptionSource(
        [get_package_share_directory('bunker_base'), '/launch/bunker_base.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument('port_name', default_value='can0'),
        DeclareLaunchArgument('odom_topic_name', default_value='wheel_odom'),
        IncludeLaunchDescription(
            bunker_launch,
            launch_arguments={
                'use_sim_time': 'false',
                'port_name': LaunchConfiguration('port_name'),
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'odom_topic_name': LaunchConfiguration('odom_topic_name'),
                'is_bunker_mini': 'true',
                'simulated_robot': 'false',
                # FAST-LIO + sensor_scan_generation own odom -> base_footprint.
                'publish_odom_tf': 'false',
            }.items(),
        ),
    ])
