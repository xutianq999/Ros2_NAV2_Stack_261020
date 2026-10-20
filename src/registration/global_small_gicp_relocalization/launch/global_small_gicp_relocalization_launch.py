# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    node = Node(
        package="small_gicp_relocalization",
        executable="small_gicp_relocalization_node",
        namespace="",
        output="screen",
        emulate_tty=True,  # 开启提示颜色
        remappings=remappings,
        parameters=[
            {
                "num_threads": 4,
                "num_neighbors": 10,
                "global_leaf_size": 0.25,
                "registered_leaf_size": 0.25,
                "max_dist_sq": 1.0,
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
