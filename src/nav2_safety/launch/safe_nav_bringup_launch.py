# 带安全层的完整导航启动 —— my_nav2_launch.py 的替代品
#
# 与 me_nav2_bringup 的 my_nav2_launch.py 覆盖同样的内容（静态地图、RViz、航点
# 标记），但导航节点换成 navigation_safe_launch.py，并叠加安全层。
#
# 启用方式：把 scripts/nav2_real.sh 里
#     ros2 launch me_nav2_bringup my_nav2_launch.py use_sim_time:=false
# 改成
#     ros2 launch nav2_safety safe_nav_bringup_launch.py use_sim_time:=false
# 并把遥控发布者从 /cmd_vel 改到 /cmd_vel_teleop（见 docs/ENABLING.md）。
# 底盘和 me_nav2_bringup 都不需要改动。
#
# 地图路径复用 me_nav2_bringup 的 map_config.yaml，换地图仍然是改那一个文件。

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def load_map_path(key):
    """读取 me_nav2_bringup 的 map_config.yaml，与 my_nav2_launch.py 行为一致。"""
    me_share = get_package_share_directory('me_nav2_bringup')
    ws_root = os.path.abspath(os.path.join(me_share, '..', '..', '..', '..'))
    src_cfg = os.path.join(ws_root, 'src', 'me_nav2_bringup', 'config', 'map_config.yaml')
    cfg_file = src_cfg if os.path.isfile(src_cfg) else os.path.join(
        me_share, 'config', 'map_config.yaml')
    with open(cfg_file) as f:
        path = yaml.safe_load(f)[key]
    pkg_root = os.path.dirname(os.path.dirname(cfg_file))
    return path if os.path.isabs(path) else os.path.join(pkg_root, path)


def generate_launch_description():
    safety_share = get_package_share_directory('nav2_safety')
    me_share = get_package_share_directory('me_nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    use_map_server = LaunchConfiguration('use_map_server')
    bt_xml = LaunchConfiguration('bt_xml')

    params_file = os.path.join(me_share, 'config', 'nav2_params.yaml')
    map_yaml_file = load_map_path('map_yaml_file')
    rviz_file = os.path.join(me_share, 'rviz', 'nav2.rviz')

    ws_root = os.path.abspath(os.path.join(me_share, '..', '..', '..', '..'))
    src_waypoints = os.path.join(ws_root, 'src', 'me_nav2_bringup', 'config', 'waypoints.yaml')
    waypoints_file = src_waypoints if os.path.isfile(src_waypoints) else os.path.join(
        me_share, 'config', 'waypoints.yaml')

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(safety_share, 'launch', 'navigation_safe_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'True',
            'bt_xml': bt_xml,
        }.items(),
    )

    safety_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(safety_share, 'launch', 'safety_layer_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 探索模式用 use_map_server:=false 关掉：那时 /map 由 grid_mapper 随探索进程
    # 实时生长（前沿探索需要"未知区域会变多"），静态先验图做不到。两者都发 /map，
    # 同时跑会让全局代价地图在两份图之间来回跳。
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        condition=IfCondition(use_map_server),
        parameters=[params_file, {'yaml_filename': map_yaml_file,
                                  'use_sim_time': use_sim_time}],
    )

    lifecycle_manager_map_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        condition=IfCondition(use_map_server),
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}],
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        condition=IfCondition(rviz),
    )

    waypoint_marker_cmd = Node(
        package='me_nav2_bringup',
        executable='record_waypoints',
        name='waypoint_marker_publisher',
        output='screen',
        parameters=[{'output_file': waypoints_file, 'record_enabled': False}],
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='true', description='启动 RViz'))
    ld.add_action(DeclareLaunchArgument(
        'use_map_server', default_value='true',
        description='是否用 map_server 加载静态先验图；探索模式设 false，'
                    '改由 grid_mapper 提供实时生长的 /map'))
    ld.add_action(DeclareLaunchArgument(
        'bt_xml', default_value='',
        description='自定义行为树 XML 文件名（空 = 使用 nav2_params.yaml 的默认值）'))
    ld.add_action(navigation_cmd)
    ld.add_action(safety_cmd)
    ld.add_action(map_server_cmd)
    ld.add_action(lifecycle_manager_map_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(waypoint_marker_cmd)

    return ld
