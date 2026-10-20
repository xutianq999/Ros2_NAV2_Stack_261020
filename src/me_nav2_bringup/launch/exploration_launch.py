# 前沿自主探索（explore_lite）
#
# 只起探索节点本身。它通过 NavigateToPose 动作把前沿目标交给 Nav2，
# 所以跑它之前整条链必须已经在跑，且顺序是：
#
#   1) 定位/里程计         ./scripts/super_lio_nav.sh mapping     (Super-LIO + 桥接)
#   2) 底盘                ./scripts/chassis_start.sh
#   3) 2D 扫描             ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch.py
#   4) 实时占据图          slam_toolbox（或在桥接层之外自建栅格节点）
#   5) Nav2 + 安全层        ros2 launch me_nav2_bringup my_nav2_launch.py
#   6) 本文件              ros2 launch me_nav2_bringup exploration_launch.py
#
# 注意：第 5 步的 map_server 必须让位给第 4 步的实时 /map —— 两者都发 /map，
# 同时跑会让全局代价地图在"先验图/实时图"之间跳。详见 src/super_lio/README.md。
#
# 运行中可以用话题控制：
#   ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"   # 暂停
#   ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"    # 继续
#   ros2 topic pub --once /exploration/mode std_msgs/msg/Bool "{data: false}" # 手动模式
#   ros2 topic pub --once /exploration/mode std_msgs/msg/Bool "{data: true}"  # 自主模式
# 手动模式下 exploration_mode 会停止前沿探索，RViz 2D Goal Pose 直接交给 Nav2；
# 自主模式下同一个工具作为引导，并直接成为高优先级导航目标。
# 前沿可视化在 /explore/frontiers，状态在 /explore/status。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('me_nav2_bringup')
    config_file = os.path.join(share, 'config', 'explore_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    bounds_file = LaunchConfiguration('bounds_file')
    exploration_mode = LaunchConfiguration('exploration_mode')

    # 默认把边界存到源码工作区，方便下次启动自动加载；install 只作为兜底。
    workspace = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    default_bounds = os.path.join(
        workspace, 'src', 'me_nav2_bringup', 'config', 'exploration_bounds.yaml')
    if not os.path.isdir(os.path.dirname(default_bounds)):
        default_bounds = os.path.join(share, 'config', 'exploration_bounds.yaml')

    # 命名空间重映射，理由同上游 launch：TF 话题需要随命名空间走
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # 可视化设置节点：Publish Point 点边界 + 2D Goal Pose 给方向引导。
    ui_node = Node(
        package='me_nav2_bringup',
        executable='exploration_ui',
        name='exploration_ui',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'bounds_file': bounds_file,
            'auto_load': True,
            'goal_topic': '/exploration/guide_pose',
        }],
        remappings=remappings,
    )

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[config_file, {
            'use_sim_time': use_sim_time,
            'startup_mode': exploration_mode,
        }],
        remappings=remappings,
    )

    # Routes RViz's 2D Goal Pose: AUTO -> guide, MANUAL -> Nav2 goal. It also
    # starts/stops explore_node on mode changes. Living next to explore keeps
    # the switch available even when this launch is used without the bringup.
    mode_router = Node(
        package='me_nav2_bringup',
        executable='exploration_mode',
        name='exploration_mode',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'startup_mode': exploration_mode,
        }],
        remappings=remappings,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'bounds_file', default_value=default_bounds,
            description='探索边界 YAML；不存在则从空区域开始'),
        DeclareLaunchArgument(
            'exploration_mode', default_value='auto',
            description='auto: RViz goal 作为引导；manual: RViz goal 直接交给 Nav2'),
        ui_node,
        explore_node,
        mode_router,
    ])
