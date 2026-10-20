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
    """读取 map_config.yaml 中的地图路径（换地图只改该文件，无需重新编译）。

    优先读取源码目录 src/me_nav2_bringup/config/map_config.yaml（改完立即生效），
    找不到时回退到 install 目录下的拷贝。相对路径以配置所在包目录为基准，
    绝对路径直接使用。
    """
    me_share = get_package_share_directory('me_nav2_bringup')
    # 由标准 install 布局 <ws>/install/<pkg>/share/<pkg> 反推工作区根目录
    ws_root = os.path.abspath(os.path.join(me_share, '..', '..', '..', '..'))
    src_cfg = os.path.join(ws_root, 'src', 'me_nav2_bringup', 'config', 'map_config.yaml')
    cfg_file = src_cfg if os.path.isfile(src_cfg) else os.path.join(
        me_share, 'config', 'map_config.yaml')
    with open(cfg_file) as f:
        path = yaml.safe_load(f)[key]
    pkg_root = os.path.dirname(os.path.dirname(cfg_file))
    return path if os.path.isabs(path) else os.path.join(pkg_root, path)


def generate_launch_description():
    # 获取共享目录路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    me_share_path = get_package_share_directory('me_nav2_bringup')
    safety_share = get_package_share_directory('nav2_safety')

    # 配置文件与地图路径
    params_file = os.path.join(me_share_path, 'config', 'nav2_params.yaml')
    map_yaml_file = load_map_path('map_yaml_file')
    rviz_file = os.path.join(me_share_path, 'rviz', 'nav2.rviz')
    workspace_root = os.path.abspath(os.path.join(me_share_path, '..', '..', '..', '..'))
    source_waypoints = os.path.join(
        workspace_root, 'src', 'me_nav2_bringup', 'config', 'waypoints.yaml')
    waypoints_file = source_waypoints if os.path.isfile(source_waypoints) else os.path.join(
        me_share_path, 'config', 'waypoints.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_map_server = LaunchConfiguration('use_map_server')

    # nav2_params.yaml 里两份 costmap 的障碍层都订阅 /scan_reliable。
    # /scan 的发布端是 Best Effort，而 costmap 用 Reliable 订阅，QoS 不兼容；
    # 这个中继把同一份消息用 Reliable 重发。安全层启动时也会起同一个节点，
    # 这里补上是为了 my_nav2_launch.py 这条回滚路径也能收到扫描数据。
    scan_relay_cmd = Node(
        package='nav2_safety',
        executable='scan_qos_relay',
        name='scan_qos_relay',
        output='screen',
        parameters=[os.path.join(safety_share, 'config', 'scan_qos_relay.yaml'),
                    {'use_sim_time': use_sim_time}],
    )

    # 只读状态聚合，Web 导航页订阅 /robot/status 显示实时状态。
    # 这条回滚路径没有安全层，安全相关字段会显示为未知，导航/探索/卡住仍可用。
    robot_status_cmd = Node(
        package='nav2_safety',
        executable='robot_status_monitor',
        name='robot_status_monitor',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[os.path.join(
            safety_share, 'config', 'robot_status_monitor.yaml'),
            {'use_sim_time': use_sim_time}],
    )

    # 启动纯导航组件，不使用AMCL
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'True'
        }.items()
    )

    # 独立启动 Map Server
    # 加载静态地图，供 Global Costmap 使用
    #
    # 探索模式下用 use_map_server:=false 关掉它：那时 /map 由 grid_mapper（或
    # slam_toolbox）按探索进程实时生长，需要"未知区域会变多"，静态先验图做不到。
    # 两者都发 /map，同时跑会让全局代价地图在两份图之间来回跳。
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        condition=IfCondition(use_map_server),
        # 同时传入你的 nav2_params.yaml 和 地图文件路径
        parameters=[params_file, {'yaml_filename': map_yaml_file, 'use_sim_time': use_sim_time}]
    )

    # 启动 Map Server 的生命周期管理器
    # 负责将 map_server 从 Unconfigured 状态推送到 Active 状态
    lifecycle_manager_map_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        condition=IfCondition(use_map_server),
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    # rviz
    rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_file], 
    )

    waypoint_marker_cmd = Node(
        package='me_nav2_bringup',
        executable='record_waypoints',
        name='waypoint_marker_publisher',
        output='screen',
        parameters=[{'output_file': waypoints_file, 'record_enabled': False}],
    )

    # 组合 LaunchDescription 返回
    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'use_map_server', default_value='true',
            description='是否用 map_server 加载静态先验图；探索模式请设 false，'
                        '改由 grid_mapper 提供实时生长的 /map'),
    ])
    ld.add_action(scan_relay_cmd)
    ld.add_action(robot_status_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(map_server_cmd)
    ld.add_action(lifecycle_manager_map_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(waypoint_marker_cmd)

    return ld
