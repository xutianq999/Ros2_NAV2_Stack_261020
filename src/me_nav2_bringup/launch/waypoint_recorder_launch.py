import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _map_yaml_file():
    share = get_package_share_directory('me_nav2_bringup')
    workspace = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    source_config = os.path.join(workspace, 'src', 'me_nav2_bringup', 'config', 'map_config.yaml')
    config = source_config if os.path.isfile(source_config) else os.path.join(share, 'config', 'map_config.yaml')
    with open(config, encoding='utf-8') as stream:
        path = yaml.safe_load(stream)['map_yaml_file']
    return path


def generate_launch_description():
    share = get_package_share_directory('me_nav2_bringup')
    workspace = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    default_output = os.path.join(workspace, 'src', 'me_nav2_bringup', 'config', 'waypoints.yaml')
    if not os.path.isdir(os.path.dirname(default_output)):
        default_output = os.path.join(share, 'config', 'waypoints.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    output_file = LaunchConfiguration('output_file')
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='waypoint_map_server',
        output='screen', parameters=[{'yaml_filename': _map_yaml_file(), 'use_sim_time': use_sim_time}],
        remappings=[('map', '/waypoint_map'), ('map_updates', '/waypoint_map_updates')],
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='waypoint_map_lifecycle',
        output='screen', parameters=[{'autostart': True, 'use_sim_time': use_sim_time,
                                     'node_names': ['waypoint_map_server']}],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='waypoint_rviz', output='screen',
        arguments=['-d', os.path.join(share, 'rviz', 'waypoint_recorder.rviz')],
    )
    recorder = Node(
        package='me_nav2_bringup', executable='record_waypoints', output='screen',
        parameters=[{'output_file': output_file}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('output_file', default_value=default_output),
        map_server, lifecycle, recorder, rviz,
    ])
