# 安全层：碰撞监控 + 优先级仲裁 + 兜底归零 + 定位看门狗
#
# 只启动安全层本身，不启动 Nav2 导航节点 —— 导航节点由
# navigation_safe_launch.py 负责（或用 safe_nav_bringup_launch.py 一起拉起）。
#
# 完整话题链：
#   Nav2 controller_server      -> cmd_vel_nav
#   velocity_smoother           -> cmd_vel_nav_smoothed
#   collision_monitor           -> cmd_vel_nav_checked   （障碍停车/减速）
#   twist_mux                   -> cmd_vel_muxed         （nav / teleop / 锁优先级）
#   cmd_vel_deadman             -> /cmd_vel              （静默即归零）
#   底盘 bunker_base            订阅 /cmd_vel（无需改动）
#
# 每一级的必要性（都是实测/读码得出的结论，不是照搬范式）：
#   collision_monitor 独立于 controller_server 再做一次障碍检查；
#   twist_mux 解决 /cmd_vel 多发布者无仲裁的问题；
#   deadman 补上 twist_mux "静默不带零速" 的缺口；
#   watchdog 在 TF 断流时掐掉自动驾驶（遥控保留）。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('nav2_safety')
    config = os.path.join(share, 'config')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_collision_monitor = LaunchConfiguration('use_collision_monitor')
    use_recovery_guard = LaunchConfiguration('use_recovery_guard')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock if true')

    # 建图流程没有 Nav2，collision_monitor 没有输入可监控，用
    # use_collision_monitor:=false 关掉它，只保留仲裁/兜底/看门狗。
    declare_use_collision_monitor_cmd = DeclareLaunchArgument(
        'use_collision_monitor', default_value='true',
        description='是否启动 collision_monitor（建图流程可关掉）')

    # 守卫和 collision_monitor 是两件事：前者管恢复行为的方向性放行，后者管
    # 导航指令的无方向停车。不要共用一个开关 —— 关掉 collision_monitor 时
    # 恢复通道仍然需要守卫，否则 /cmd_vel_recovery 会没有消费者。
    declare_use_recovery_guard_cmd = DeclareLaunchArgument(
        'use_recovery_guard', default_value='true',
        description='是否启动 recovery_guard（恢复行为的方向性碰撞守卫）')

    # /scan 的发布端是 Best Effort，Nav2 costmap 的 obstacle_layer 用 Reliable
    # 订阅，两者不兼容，代价地图实际收不到扫描数据。中继成 Reliable 话题。
    scan_relay_cmd = Node(
        package='nav2_safety',
        executable='scan_qos_relay',
        name='scan_qos_relay',
        output='screen',
        parameters=[os.path.join(config, 'scan_qos_relay.yaml'),
                    {'use_sim_time': use_sim_time}],
    )

    # 优先级仲裁。输出话题固定叫 cmd_vel_out，只能靠 remap 改名（见 config 注释）
    twist_mux_cmd = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[os.path.join(config, 'twist_mux.yaml')],
        remappings=[('cmd_vel_out', 'cmd_vel_muxed')],
    )

    # 障碍停车/减速，只约束自动驾驶指令流
    # 注意：collision_monitor 是**生命周期节点**，不配 lifecycle_manager 的话它会
    # 一直停在 unconfigured，参数照收但完全不工作 —— 必须一起起 lifecycle_manager。
    collision_monitor_cmd = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[os.path.join(config, 'collision_monitor.yaml'),
                    {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_collision_monitor),
    )

    collision_monitor_lifecycle_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['collision_monitor']}],
        condition=IfCondition(use_collision_monitor),
    )

    # 恢复行为的方向性碰撞守卫：只拦朝着障碍的运动分量，让车能从贴墙处脱身。
    recovery_guard_cmd = Node(
        package='nav2_safety',
        executable='recovery_guard',
        name='recovery_guard',
        output='screen',
        parameters=[os.path.join(config, 'recovery_guard.yaml'),
                    {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_recovery_guard),
    )

    # 只读状态聚合：把 Nav2/恢复行为/安全层/定位/探索汇总成 /robot/status，
    # 供 Web 导航页和 RViz 显示“卡住/脱困中”等状态。不参与控制。
    robot_status_cmd = Node(
        package='nav2_safety',
        executable='robot_status_monitor',
        name='robot_status_monitor',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[os.path.join(config, 'robot_status_monitor.yaml'),
                    {'use_sim_time': use_sim_time}],
    )

    # 上游静默 -> 显式零速
    deadman_cmd = Node(
        package='nav2_safety',
        executable='deadman',
        name='cmd_vel_deadman',
        output='screen',
        parameters=[os.path.join(config, 'deadman.yaml')],
    )

    # 软件急停锁存：把一次性指令变成持续发布、可持久读取的状态。
    # respawn 是安全的：锁存状态会从 state_file 恢复，不会因重启而静默解除。
    estop_cmd = Node(
        package='nav2_safety',
        executable='estop_latch',
        name='estop_latch',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[os.path.join(config, 'estop_latch.yaml')],
    )

    # 定位/里程计健康 -> 锁住自动驾驶
    watchdog_cmd = Node(
        package='nav2_safety',
        executable='localization_watchdog',
        name='localization_watchdog',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[os.path.join(config, 'localization_watchdog.yaml'),
                    {'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_collision_monitor_cmd)
    ld.add_action(declare_use_recovery_guard_cmd)
    ld.add_action(LogInfo(msg=(
        'nav2_safety: 安全层启动。启用后 /cmd_vel 由兜底级独占，'
        '遥控发布者必须改到 /cmd_vel_teleop（见 docs/ENABLING.md）。')))
    ld.add_action(scan_relay_cmd)
    ld.add_action(collision_monitor_cmd)
    ld.add_action(collision_monitor_lifecycle_cmd)
    ld.add_action(recovery_guard_cmd)
    ld.add_action(robot_status_cmd)
    ld.add_action(twist_mux_cmd)
    ld.add_action(deadman_cmd)
    ld.add_action(estop_cmd)
    ld.add_action(watchdog_cmd)

    return ld
