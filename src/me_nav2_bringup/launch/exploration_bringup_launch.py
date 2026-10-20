# 自主探索整链一键启动
#
# 组合了四部分（按数据流顺序）：
#   super_lio_bringup  Super-LIO(建图模式) + 桥接 + 2D扫描 + Nav2 + 安全层
#                      —— 其中 map_server 关掉（use_map_server:=false）
#   grid_mapper        用 Super-LIO 的点云长出实时 /map，取代 map_server
#   explore_lite       前沿探索，把目标通过 NavigateToPose 交给 Nav2
#
# 为什么不用 super_lio_nav.sh 的 mapping 模式再手动加两个节点：
#   那条路会连 map_server 一起起（它加载静态先验图），和 grid_mapper 抢 /map，
#   全局代价地图会在"先验图/实时图"之间来回跳。这里显式关掉它。
#
# 底盘不在这个 launch 里：实机请先跑 ./scripts/chassis_start.sh。
# 用数据包验证（无硬件）时加 use_livox_driver:=false，再从别处回放 bag。
#
# 前置：必须同时 source 本工作区与 Super-LIO 的工作区（scripts/super_lio_nav.sh
# 的 exploration 模式会自动处理）。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    me_share = get_package_share_directory('me_nav2_bringup')
    bridge_share = get_package_share_directory('super_lio_bridge')
    grid_share = get_package_share_directory('grid_mapper')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('exploration_rviz')
    use_livox_driver = LaunchConfiguration('use_livox_driver')

    # Super-LIO 走建图模式：探索本来就是在没有先验图的区域里长地图，
    # 不需要重定位；位姿由 LIO 提供，地图由 grid_mapper 长。
    # 使用增强恢复版行为树，应对"起点在致命空间"等探索场景常见问题。
    super_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bridge_share, 'launch', 'super_lio_bringup_launch.py')),
        launch_arguments={
            'mode': 'mapping',
            'use_map_server': 'false',
            'use_livox_driver': use_livox_driver,
            'rviz': 'false',
            'use_sim_time': use_sim_time,
            'bt_xml': 'navigate_w_replanning_and_exploration_recovery.xml',
        }.items())

    grid_mapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            grid_share, 'launch', 'grid_mapper_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            me_share, 'launch', 'exploration_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'exploration_mode': LaunchConfiguration('exploration_mode'),
        }.items())

    # RViz's 2D Goal Pose is routed through exploration_mode (started by
    # exploration_launch.py). In AUTO mode it becomes a guide; in MANUAL mode
    # it becomes a normal Nav2 goal. The tool must not publish /goal_pose
    # directly, or bt_navigator would start a second goal and abort the
    # frontier goal.
    exploration_rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', os.path.join(me_share, 'rviz', 'nav2.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/goal_pose', '/exploration/input_goal')])

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='false', description='启动 RViz'))
    ld.add_action(DeclareLaunchArgument(
        'use_livox_driver', default_value='true',
        description='是否启动 Livox 驱动；回放数据包时设 false'))
    ld.add_action(DeclareLaunchArgument(
        'exploration_mode', default_value='auto',
        description='auto: RViz goal 作为引导；manual: RViz goal 直接交给 Nav2'))
    # Nested launch files set rviz:=false; preserve the user's choice first.
    ld.add_action(SetLaunchConfiguration('exploration_rviz', LaunchConfiguration('rviz')))
    ld.add_action(super_lio)
    ld.add_action(grid_mapper)
    ld.add_action(explore)
    ld.add_action(exploration_rviz)
    return ld
