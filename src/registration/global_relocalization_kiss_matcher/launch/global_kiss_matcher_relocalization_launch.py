import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    default_pcd_file = load_map_path('prior_pcd_file')
    prior_pcd_file = LaunchConfiguration('prior_pcd_file')
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    node = Node(
        package="global_relocalization_kiss_matcher",
        executable="global_kiss_matcher_relocalization_exec",
        namespace="",
        output="screen",
        emulate_tty=True,
        remappings=remappings,
        parameters=[
            {
                "num_threads": 4,
                "num_neighbors": 10,
                "global_leaf_size": 0.25,
                "registered_leaf_size": 0.25,
                "max_dist_sq": 1.0,
                "voxel_resolution": 0.25,
                "use_global_initialization": True,
                "use_kiss_recovery": True,
                "gicp_max_consecutive_failures": 2,
                # 0 = 只按 converged 判定；>0 时“收敛但 error 超阈值”也算失败，
                # 连续失败后触发 KISS 全局恢复。建议先看实际 error 再启用。
                "gicp_error_max": 0.0,
                "recovery_min_points": 1000,
                "recovery_cooldown_sec": 2.0,
                "verify_kiss_with_gicp": True,
                "loop.num_inliers_threshold": 3,
                "loop.overlap_threshold": 80.0,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "lidar_frame": "livox_frame",
                "robot_base_frame": "base_footprint",
                "prior_pcd_file": prior_pcd_file,
                "input_cloud_topic": "/registered_scan",
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('prior_pcd_file', default_value=default_pcd_file),
        node,
    ])
