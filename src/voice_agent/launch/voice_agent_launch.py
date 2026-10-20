"""启动语音 Agent HTTP 服务（同时是一个 ROS 2 节点）。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_waypoints_file() -> str:
    """优先使用 me_nav2_bringup 里正在维护的航点文件。"""
    try:
        me_share = get_package_share_directory("me_nav2_bringup")
        ws_root = os.path.abspath(os.path.join(me_share, "..", "..", "..", ".."))
        source = os.path.join(
            ws_root, "src", "me_nav2_bringup", "config", "waypoints.yaml"
        )
        if os.path.isfile(source):
            return source
        installed = os.path.join(me_share, "config", "waypoints.yaml")
        if os.path.isfile(installed):
            return installed
    except Exception:
        pass
    return os.path.join(
        get_package_share_directory("voice_agent"), "config", "waypoints.yaml"
    )


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("voice_agent")
    params_file = os.path.join(share, "config", "voice_agent.yaml")

    waypoints_file = LaunchConfiguration("waypoints_file")
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    use_sim_time = LaunchConfiguration("use_sim_time")

    node = Node(
        package="voice_agent",
        executable="voice_agent_server",
        name="voice_agent_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            params_file,
            {
                "waypoints_file": ParameterValue(waypoints_file, value_type=str),
                "host": ParameterValue(host, value_type=str),
                "port": ParameterValue(port, value_type=int),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=_default_waypoints_file(),
                description="航点 YAML 文件路径",
            ),
            DeclareLaunchArgument(
                "host",
                default_value=os.getenv("HOST", "0.0.0.0"),
                description="HTTP 监听地址",
            ),
            DeclareLaunchArgument(
                "port",
                default_value=os.getenv("PORT", "8000"),
                description="HTTP 监听端口",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="回放数据包时设为 true",
            ),
            node,
        ]
    )
