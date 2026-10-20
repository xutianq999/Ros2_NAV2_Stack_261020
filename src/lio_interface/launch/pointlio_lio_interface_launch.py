import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    # config = os.path.join(
    #     get_package_share_directory('lio_interface'), 'config', 'static_tf.yaml')

    lio_interface_node = Node(
        package='lio_interface',
        executable='lio_interface_node',
        namespace='',
        output='screen',
        emulate_tty=True,  # 开启提示颜色
        parameters=[{
            'use_sim_time': use_sim_time,
            'odometry_sub': '/aft_mapped_to_init',
        }],
        
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        lio_interface_node,
    ])
