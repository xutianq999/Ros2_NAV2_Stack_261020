# Nav2 导航节点（安全版）
#
# 派生自 /opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py，
# 与上游的差异只有两处 remap：
#
#   上游：velocity_smoother 输出 -> /cmd_vel          （直连底盘）
#   本文件：velocity_smoother 输出 -> /cmd_vel_nav_smoothed
#
# 这样 Nav2 的最终输出不再直接落到底盘订阅的话题上，中间才有位置插入
# 碰撞监控 -> 仲裁 -> 兜底归零 这条安全链。
#
# 另外本文件去掉了上游的 composed-nodes 分支：本项目一直以
# use_composition=False 运行，保留那条分支只会带来维护负担。
#
# 话题链：
#   controller_server  -> cmd_vel_nav
#   velocity_smoother  -> cmd_vel_nav_smoothed   （in: cmd_vel_nav）
#   collision_monitor  -> cmd_vel_nav_checked    （见 config/collision_monitor.yaml）
#   twist_mux          -> cmd_vel_muxed          （见 config/twist_mux.yaml）
#   deadman            -> /cmd_vel               （底盘订阅，无需改动）

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    me_share = get_package_share_directory('me_nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    bt_xml = LaunchConfiguration('bt_xml')

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file for all navigation nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'bt_xml', default_value='',
        description='自定义行为树 XML 文件名（空 = 使用 nav2_params.yaml 的默认值）')

    def _load_nodes(context):
        """动态构造节点，支持探索场景的自定义行为树"""
        bt_xml_value = LaunchConfiguration('bt_xml').perform(context)

        # 基础参数：从 params_file 加载
        param_substitutions = {
            'use_sim_time': use_sim_time,
            'autostart': autostart}

        # 探索场景：覆盖行为树路径。
        # Humble 的 bt_navigator 读的是 default_nav_to_pose_bt_xml；
        # default_bt_xml_filename 是旧版本的名字，写了也会被忽略（之前就栽在这）。
        # RewrittenYaml 只改 YAML 里已有的键，所以 nav2_params.yaml 里必须有这个键。
        if bt_xml_value:
            bt_xml_path = os.path.join(me_share, 'behavior_trees', bt_xml_value)
            if not os.path.isfile(bt_xml_path):
                raise RuntimeError(f'bt_xml 指定的行为树不存在: {bt_xml_path}')
            param_substitutions['default_nav_to_pose_bt_xml'] = bt_xml_path

        configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

        return [load_nodes_impl(configured_params)]

    def load_nodes_impl(configured_params):
        return GroupAction(
            actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                # 关键：behavior_server（spin/backup/drive_on_heading 等恢复行为）
                # 默认直接发布 /cmd_vel。那样会整条绕过安全链 —— 急停锁不住它，
                # collision_monitor 也管不到它，而且会和兜底级的输出在底盘上交错。
                # 没有并入导航指令流，是因为脱困需要"贴着的障碍允许保持间隙或
                # 远离"这条比 collision_monitor 更宽松的策略；后者只做严格的车身
                # 碰撞预测，嵌死时可能一步都不放行。
                # 现在走专用通道，由 recovery_guard 做方向性把关（只拦朝着障碍的
                # 分量），再进 twist_mux 的独立输入，仍受急停锁和看门狗锁约束：
                #   cmd_vel_recovery -> recovery_guard -> cmd_vel_recovery_checked
                #   -> twist_mux(priority 20) -> 兜底 -> 底盘
                # 绝不能直接发 /cmd_vel：那样会绕过整条安全链（deadman 自检会报）。
                remappings=remappings + [('cmd_vel', 'cmd_vel_recovery')]),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                [('cmd_vel', 'cmd_vel_nav'),
                 ('cmd_vel_smoothed', 'cmd_vel_nav_smoothed')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ])

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(OpaqueFunction(function=_load_nodes))

    return ld
