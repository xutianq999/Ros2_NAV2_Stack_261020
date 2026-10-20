# 用 Super-LIO 跑整套导航栈（替代 FAST-LIO + lio_interface + sensor_scan_generation
# + small_gicp 重定位那一条链）。
#
# 与 scripts/nav2_real.sh 的对应关系：
#   原链路：livox 驱动 -> FAST-LIO -> lio_interface -> sensor_scan_generation
#           -> 点云转2D -> small_gicp 重定位 -> Nav2(+安全层)
#   本链路：livox 驱动 -> Super-LIO -> super_lio_bridge
#           -> 点云转2D -> Nav2(+安全层)
# 其中 FAST-LIO/lio_interface/sensor_scan_generation/small_gicp 四项，
# 由 Super-LIO 加桥接层一并取代（Super-LIO 自带重定位）。
#
# mode:
#   mapping     —— 建图，退出时把地图存到 <save_map_dir>/map.pcd
#   relocation  —— 读预先存好的地图做重定位（这是"使用它自己的重定位"）
#
# 前置：必须已 source Super-LIO 的工作空间。用 scripts/super_lio_nav.sh 会自动处理。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

MODE_EXECUTABLE = {
    'mapping': ('super_lio', 'super_lio_node', 'super_lio_mid360_mapping.yaml'),
    'relocation': ('super_lio', 'relocation_node', 'super_lio_mid360_relocation.yaml'),
    # tracking = Super-LIO 只当里程计（mapping 配置，但不存图），
    # map->odom 交给 small_gicp_relocalization 以 2Hz 持续修正。
    'tracking': ('super_lio', 'super_lio_node', 'super_lio_mid360_mapping.yaml'),
    # tracking_kiss = 同上，但重定位换成 KISS-Matcher + small_gicp：
    # 启动时无初值全局初始化；GICP 连续失败后自动切回 KISS 做全局恢复。
    'tracking_kiss': ('super_lio', 'super_lio_node', 'super_lio_mid360_mapping.yaml'),
}


def _launch_super_lio(context, *args, **kwargs):
    """按 mode 起对应的 Super-LIO 节点。

    这里直接起节点而不是 include 上游的 launch —— 上游 launch/relocation.py 加载的是
    建图那份配置（缺 lio.relocation.* 参数，仓库里明明有 relocation.yaml），
    看起来是它的笔误。我们显式指定适配好的配置，绕开这个问题。
    """
    mode = LaunchConfiguration('mode').perform(context)
    share = get_package_share_directory('super_lio_bridge')
    rviz = LaunchConfiguration('rviz')

    if mode not in MODE_EXECUTABLE:
        raise RuntimeError(
            f'mode 只能是 {sorted(MODE_EXECUTABLE)}，收到 "{mode}"')
    package, executable, config_name = MODE_EXECUTABLE[mode]

    params = [
        os.path.join(share, 'config', config_name),
        {'use_sim_time': LaunchConfiguration('use_sim_time')},
    ]
    if mode in ('tracking', 'tracking_kiss'):
        # 这两种模式用 mapping 配置，但绝不能在退出时写地图：
        # 先验 map.pcd 是重定位的基准，不能被这次导航覆盖。
        params.append({'lio.map.save_map': False})

    actions = [
        LogInfo(msg=f'Super-LIO 模式: {mode}（{executable} + {config_name}）'),
        Node(
            package=package,
            executable=executable,
            name=executable.replace('_node', '') + '_node',
            output='screen',
            parameters=params,
        ),
    ]
    if mode == 'relocation':
        actions.append(LogInfo(msg=(
            '重定位模式：先验地图取自配置里的 save_map_dir/map.pcd。'
            '若车不在原点附近，启动后用 RViz 或网页的"初始位姿"发 /initialpose 给初值。')))
    elif mode == 'tracking':
        actions.append(LogInfo(msg=(
            '跟踪模式：Super-LIO 只提供 odom 系里程计，'
            'map->odom 由 small_gicp_relocalization 持续修正。'
            '若车不在先验图原点附近，启动后用 RViz 的 "2D Pose Estimate" '
            '发 /initialpose 给初值（该模式运行中随时可重发）。')))
    elif mode == 'tracking_kiss':
        actions.append(LogInfo(msg=(
            '跟踪+恢复模式：Super-LIO 只提供 odom；KISS-Matcher 做无初值全局初始化，'
            'GICP 连续失败后自动 KISS 全局恢复。适合起点不确定或需要自动找回的场景。')))
    return actions


def generate_launch_description():
    share = get_package_share_directory('super_lio_bridge')
    me_share = get_package_share_directory('me_nav2_bringup')

    declare_mode = DeclareLaunchArgument(
        'mode', default_value='relocation',
        description='mapping=建图 / relocation=Super-LIO 自带一次性重定位 / '
                    'tracking=Super-LIO 里程计 + small_gicp 持续重定位 / '
                    'tracking_kiss=KISS 全局初始化 + GICP 跟踪 + 失败自动恢复')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='false')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    # 回放数据包做验证时设 false：否则驱动和 bag 会同时往 /livox/* 发数据，
    # 时间戳一个走墙钟一个走录制时间，IMU 会互相把时间戳拽回去，
    # Super-LIO 每帧都报 "imu loop back, clear buffer" 然后丢数据。
    declare_use_livox_driver = DeclareLaunchArgument(
        'use_livox_driver', default_value='true',
        description='是否启动 Livox 驱动；回放数据包时设 false')
    declare_use_map_server = DeclareLaunchArgument(
        'use_map_server', default_value='true',
        description='是否用 map_server 加载静态先验图；探索模式设 false')
    declare_bt_xml = DeclareLaunchArgument(
        'bt_xml', default_value='',
        description='自定义行为树 XML 文件名（探索场景可传 navigate_w_replanning_and_exploration_recovery.xml）')

    # The robot-description include below passes rviz:=false to suppress its own
    # RViz. In Humble that include's launch_arguments also shadow the parent
    # 'rviz' value, so snapshot it first and give Nav2 its own name.
    nav_rviz = LaunchConfiguration('nav_rviz')

    # Livox MID-360 驱动：Super-LIO 在 lidar_type=1 下订阅 CustomMsg，
    # 所以必须用 xfer_format=1 的这份 launch（=项目里的 msg_MID360_launch.py）。
    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('livox_ros_driver2'),
            'launch', 'msg_MID360_launch.py')),
        condition=IfCondition(LaunchConfiguration('use_livox_driver')))

    # 机器人描述：提供 base_footprint -> chassis -> livox_frame 等静态 TF
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gld_robot_description'),
            'launch', 'gld_robot_description_launch.py')),
        launch_arguments={'rviz': 'false'}.items())

    # 3D 点云 -> 2D 激光，供 Nav2 代价地图使用（桥接已把点云转到 /registered_scan）
    laserscan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            me_share, 'launch', 'pointcloud_to_laserscan_launch.py')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items())

    # Nav2 + 安全层（安全层的看门狗会盯 /localization/tracking_ok：
    # internal 模式由桥接层发布，tracking 模式由 small_gicp 发布）
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_safety'),
            'launch', 'safe_nav_bringup_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz': nav_rviz,
            'use_map_server': LaunchConfiguration('use_map_server'),
            'bt_xml': LaunchConfiguration('bt_xml'),
        }.items())

    # tracking / tracking_kiss：外部持续重定位。订阅 /registered_scan（桥接层转发，
    # external 模式下 frame_id=odom），持续发布 map->odom。
    tracking_mode = PythonExpression([
        "'", LaunchConfiguration('mode'), "' == 'tracking'"])
    tracking_kiss_mode = PythonExpression([
        "'", LaunchConfiguration('mode'), "' == 'tracking_kiss'"])
    external_mode = PythonExpression([
        "'", LaunchConfiguration('mode'), "' in ('tracking', 'tracking_kiss')"])
    localization_mode = PythonExpression([
        "'external' if '", LaunchConfiguration('mode'),
        "' in ('tracking', 'tracking_kiss') else 'internal'"])
    relocalization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('small_gicp_relocalization'),
            'launch', 'small_gicp_relocalization_launch.py')),
        condition=IfCondition(tracking_mode))
    relocalization_kiss = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('global_relocalization_kiss_matcher'),
            'launch', 'global_kiss_matcher_relocalization_launch.py')),
        condition=IfCondition(tracking_kiss_mode))

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'bridge_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'localization_mode': localization_mode,
        }.items())

    ld = LaunchDescription()
    ld.add_action(declare_mode)
    ld.add_action(declare_rviz)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_livox_driver)
    ld.add_action(declare_use_map_server)
    ld.add_action(declare_bt_xml)
    # Must run before the robot-description include (which sets rviz:=false).
    ld.add_action(SetLaunchConfiguration(
        'nav_rviz', LaunchConfiguration('rviz')))
    ld.add_action(livox)
    ld.add_action(description)
    ld.add_action(OpaqueFunction(function=_launch_super_lio))
    ld.add_action(bridge)
    ld.add_action(relocalization)
    ld.add_action(relocalization_kiss)
    ld.add_action(laserscan)
    ld.add_action(nav2)
    return ld
