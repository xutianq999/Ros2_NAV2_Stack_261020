# Lidar_nav2_ws

> **Note:** this English document describes the earlier FAST-LIO2-based stack.
> The current, maintained manual for the Super-LIO + Nav2 + Web workbench stack is
> [`README.md`](./README.md) (Chinese), with a detailed
> [Super-LIO vs FAST-LIO2 comparison](./docs/FASTLIO2_COMPARISON.md).

A ROS 2-based 3D LiDAR autonomous navigation system

[![ROS2](https://img.shields.io/badge/ROS2-Humble-22313F?logo=ros)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu)](https://releases.ubuntu.com/22.04/)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](./LICENSE)

<p>
  <img src="docs/KISS%20show.gif" alt="KISS demo 1" width="48%" style="margin-right: 25px;">
  <img src="docs/KISS%20show_2.gif" alt="KISS demo 2" width="48%">
</p>

**Nav2_3D** is a ROS 2 Humble navigation workspace for four-wheel skid-steering robots, targeting real-robot deployment. The system uses a Livox MID-360 3D LiDAR and IMU as its core sensors, integrating LiDAR-Inertial Odometry (LIO), 3D point-cloud relocalization, BUNKER MINI 2.0 base control, and the Nav2 navigation framework, with the whole path from CAN bring-up to goal dispatch scripted.

Key features:

- **3D relocalization** - Global relocalization based on small_gicp and KISS-Matcher
- **Dual LIO backends** - Flexible switching between FAST-LIO2 and Point-LIO
- **One-command bring-up** - `./nav2_real.sh` sequences CAN, the base driver, LIO, relocalization, and Nav2
- **Complete toolchain** - Scripted workflow for build, mapping, map saving, and navigation

Data pipeline:

> LiDAR/IMU &rarr; LIO (FAST-LIO or Point-LIO) &rarr; TF bridge (`lio_interface`) &rarr; odom TF and `/registered_scan` (`sensor_scan_generation`) &rarr; 3D-to-2D slicing (`pointcloud_to_laserscan`) &rarr; relocalization (`small_gicp_relocalization` or `global_relocalization_kiss_matcher`) &rarr; Nav2 (MPPI + Smac)

TF tree: **`map` &rarr; `odom` &rarr; `base_footprint` &rarr; `chassis` &rarr; `livox_frame`**

---

## 1. Requirements

- **Operating system**: Ubuntu 22.04
- **ROS 2**: Humble Hawksbill
- **Livox-SDK2**: Prebuilt under `src/livox_ros_driver2/3rdparty/`, with amd64/arm64 support
- **Base dependencies** (real-robot mode): `libasio-dev`, `can-utils` - to build the BUNKER MINI 2.0 driver and check the CAN bus
- **Web console dependency** (optional): `ros-humble-rosbridge-suite` - the WebSocket bridge between the browser and ROS 2

```bash
sudo apt install libasio-dev can-utils ros-humble-rosbridge-suite
```

## 2. Build

```bash
source /opt/ros/humble/setup.bash
cd scripts
./build.sh
```

The build script is equivalent to:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Rebuild after every source-code change. Before launching any node, make sure `source install/setup.bash` has been executed.

## 3. Quick Start

### 3.1 Real-Robot Mapping

```bash
source install/setup.bash
cd scripts
./mapping_real.sh
```

This starts FAST-LIO, `lio_interface`, `sensor_scan_generation`, the 3D-to-2D slice, and SLAM Toolbox using the Livox MID-360 hardware driver (`livox_ros_driver2`) and the real-robot URDF (`gld_robot_description`).

Drive the robot through the environment with the remote controller. Mapping does not start the base driver, since the LIO odometry does not need wheel odometry; if you also want `/cmd_vel` and `/wheel_odom` on the ROS side, for example to drive with `gui_teleop`, start `./chassis_start.sh` separately first.

After enough area has been covered, save the maps:

```bash
./save_map.sh       # Save the 2D occupancy grid map to src/me_nav2_bringup/map/
./save_pcd.sh       # Save the 3D point cloud, then move it manually to src/me_nav2_bringup/pcd/
```

### 3.2 Real-Robot Navigation

Map paths are configured in one place, `src/me_nav2_bringup/config/map_config.yaml`, and switching maps needs no rebuild:

- `map_yaml_file` - the 2D occupancy grid loaded by Nav2, whose `image` field points at the `.pgm`
- `prior_pcd_file` - the prior 3D point cloud used for registration

Relative paths resolve against the `me_nav2_bringup` package directory; absolute paths are used as-is. `my_nav2_launch.py` and all three relocalization launch files read this file (the copy under `src/` takes priority, so edits apply immediately).

Navigation requires the BUNKER MINI 2.0 base: the base is what actually executes `/cmd_vel` and it publishes wheel odometry on `/wheel_odom` over CAN. `nav2_real.sh` already sequences the base bring-up, so no extra step is needed:

1. Configure `can0` in the foreground, so the `sudo` prompt is answered in that terminal; abort everything if it fails;
2. Open a second terminal for the base driver and wait 5 seconds for the CAN link and `/wheel_odom` to come up;
3. Bring up the robot description, FAST-LIO, `lio_interface`, `sensor_scan_generation`, the 3D-to-2D slice, `small_gicp` relocalization, and Nav2.

```bash
cd scripts
./nav2_real.sh
```

This includes `small_gicp_relocalization`, performs relocalization against a prior PCD map, and lets you send goals with **"Nav2 Goal"** in RViz or from the web console (see 3.4).

Before starting, confirm the CAN-to-USB adapter is connected, the vehicle is powered on, the **emergency stop is released**, and the remote controller is at hand. To run the base alone, for example on a bench, use `./chassis_start.sh`; only one base driver instance may run at a time - see [`src/me_nav2_bringup/README_BUNKER_MINI2.md`](./src/me_nav2_bringup/README_BUNKER_MINI2.md).

> Do not start the base driver by hand before `nav2_real.sh`: its entry cleanup also kills the base driver and restarts it, so you would only wait an extra round.

### 3.3 Global Relocalization: Three Options

The system provides three 3D relocalization options based on a prior PCD map:

- **KISS-Matcher + small_gicp**: Suitable when the robot's initial pose is unknown, **"2D Pose Estimate"** is inaccurate, or pure small_gicp cannot converge because the initial guess is too far from the true pose. `global_relocalization_kiss_matcher` first accumulates `/registered_scan` for global coarse registration. After successful initialization, it switches to small_gicp continuous tracking and continuously publishes `map` &rarr; `odom`.
- **Pure small_gicp**: Suitable when the robot's approximate initial pose is known. By default, it starts converging near (0,0,0). You can customize the startup pose in the `small_gicp_relocalization` launch file or provide an initial pose in RViz with **"2D Pose Estimate"**. `small_gicp_relocalization` then registers against the prior map, converges faster, keeps the workflow simpler, and continuously publishes `map` &rarr; `odom`.
- **ICP registration**: Similar use case to pure small_gicp, but relocalization is performed only once at startup. It does not continue maintaining `map` &rarr; `odom`.

Before use, confirm that the prior point-cloud path is correct:

```bash
vim src/registration/small_gicp_relocalization/launch/small_gicp_relocalization_launch.py
vim src/registration/global_relocalization_kiss_matcher/launch/global_kiss_matcher_relocalization_launch.py
```

Check the following carefully:

- `prior_pcd_file`: Prior PCD map path, for example `src/me_nav2_bringup/pcd/*.pcd`
- `input_cloud_topic`: Default is `/registered_scan`
- `map_frame` / `odom_frame`: Default is `map` / `odom`
- `base_frame` / `robot_base_frame` / `lidar_frame`: Default is `base_footprint` / `base_footprint` / `livox_frame`

Choose only one relocalization node at launch time. Only one node may publish `map` &rarr; `odom` at the same time.

The pure small_gicp option is integrated into the navigation scripts:

```bash
source install/setup.bash
cd scripts
./nav2_real.sh
```

To use KISS-Matcher + small_gicp, first make sure `scripts/nav2_real.sh` does not also start `small_gicp_relocalization`, then start the global relocalization node separately:

```bash
source install/setup.bash
cd scripts

# 1. Start the main navigation workflow
./nav2_real.sh

# 2. Start the KISS-Matcher global relocalization node
ros2 launch global_relocalization_kiss_matcher global_kiss_matcher_relocalization_launch.py
```

Check whether it succeeded:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 topic hz /registered_scan
```

When `KISSMatcher initialization succeeded` appears in the log, global initialization has succeeded and the node will enter the small_gicp continuous-tracking stage. If `KISSMatcher initialization failed` keeps appearing, common causes include insufficient overlap between the accumulated local cloud and the prior map, sparse point clouds, or mismatched `prior_pcd_file` / frame settings.

### 3.4 Web Console (optional)

Besides RViz, the workspace ships a rosbridge-based web console that can be operated from a phone or tablet on the same LAN. It starts no navigation node of its own - it only bridges the browser and ROS 2 - so start it after the navigation stack is up:

```bash
# 1. Start the navigation stack first
cd scripts && ./nav2_real.sh

# 2. In another terminal, start the web services (rosbridge 9090 + static files 5500)
./web_start.sh
```

Open the address the script prints:

- Navigation page `http://<robot-ip>:5500/navigation.html`: 2D map and costmaps, single-goal navigation, multi-waypoint sequential navigation (with looping), setting the initial pose, and cancelling goals. The interfaces match RViz (`/goal_pose`, `/initialpose`) and no `tf2_web_republisher` is needed.
- Dashboard page `http://<robot-ip>:5500/index.html`: 3D point cloud (`/registered_scan`), IMU, and manual teleoperation (`/cmd_vel`, to be used only while navigation is idle).

Topic lists and troubleshooting are in [`src/web/README.md`](./src/web/README.md).

## 4. Packages

The workspace contains 30-odd ROS 2 packages under `src/` (including third-party dependencies such as `rtabmap_ros` and `3d_bbs`), listed below by purpose:

**Odometry and Localization** (`src/localization/`)

- `FAST_LIO` - FAST-LIO2: Tightly coupled LiDAR-IMU odometry based on ikd-Tree, 100 Hz+
- `point_lio` - Point-LIO: High-bandwidth odometry (4-8 kHz), robust to IMU saturation
- `Sophus` - Lie group library (SO(3)/SE(3)), math dependency for LIO

**Registration and Relocalization** (`src/registration/`)

- `small_gicp_relocalization` - Relocalization when the approximate startup pose is known: 3D point-cloud registration based on small_gicp
- `global_relocalization_kiss_matcher` - KISS-Matcher + small_gicp global relocalization: coarse registration without an initial guess, followed by GICP continuous tracking of `map` &rarr; `odom`
- `KISS-Matcher` - ICRA 2025: Fast global point-cloud registration (FPFH + TEASER++ + small_gicp)

**Sensors and Bridging**

- `livox_ros_driver2` - Livox MID-360 driver, publishing both CustomMsg and PointCloud2
- `lio_interface` - TF bridge: Converts LIO internal frames to the standard `odom` frame
- `sensor_scan_generation` - Publishes the `odom` &rarr; `base_footprint` TF, `/odom`, and `/registered_scan`

**Navigation**

- `me_nav2_bringup` - Nav2 integration: launch files, parameter configuration, maps, PCD files, and RViz configuration
- `gui_teleop` - tkinter GUI teleoperation with speed adjustment and emergency stop

**Base** (real robot)

- `ugv_sdk` - AgileX robot SDK: SocketCAN protocol implementation, a dependency of the BUNKER driver
- `bunker_base` / `bunker_msgs` - BUNKER MINI 2.0 base driver: consumes `/cmd_vel`, publishes `/wheel_odom`, `/bunker_status`, and `/bunker_rc_state`

**Robot Description**

- `gld_robot_description` - Real-robot URDF, including RealSense D456/D405 and Orbbec Gemini cameras

**Tools**

- `pcd2pgm-master` - Offline tool for converting PCD point clouds to 2D occupancy grid maps

**Web Console** (not a ROS package)

- `src/web/` - Browser console: a navigation page and a 3D point-cloud dashboard, talking to ROS 2 through rosbridge

## 5. Configuration

### 5.1 Key Configuration Files

**Navigation configuration**
- `me_nav2_bringup/config/nav2_params.yaml` - Nav2 parameters
- `me_nav2_bringup/config/slam_toolbox_params.yaml` - SLAM Toolbox online-mapping parameters
- `me_nav2_bringup/config/Pointcloud2d_3d.yaml` - 3D-to-2D slicing height and angular resolution

**LIO configuration**
- `localization/FAST_LIO/config/mid360.yaml` - FAST-LIO parameters
- `localization/point_lio/config/mid360_real.yaml` - Point-LIO real-robot parameters

**Sensor configuration**
- `livox_ros_driver2/config/MID360_config.json` - LiDAR network configuration

**Registration configuration**
- `registration/icp_registration/config/icp.yaml` - ICP registration parameters
- `registration/global_relocalization_kiss_matcher/launch/global_kiss_matcher_relocalization_launch.py` - KISS-Matcher global relocalization launch parameters
- `registration/global_relocalization_kiss_matcher/config/alignment_config.yaml` - Example KISS-Matcher frame-to-frame/global registration parameters

### 5.2 Nav2 Parameter Highlights

The current configuration uses the **MPPI** local controller and the **Smac 2D** global planner (previously DWB + Navfn; the replacement was verified to load the new plugins).

> ⚠ **Editing `nav2_params.yaml` requires a rebuild to take effect**: `me_nav2_bringup/config/`
> is copied into `install/` rather than symlinked, and the launch files read the install copy.
> `map_config.yaml` is the only exception (the launch prefers the source copy for it).
> Run `cd scripts && ./build.sh` after editing.

- **Velocity limits**: Linear velocity 0.26 m/s, angular velocity 1.0 rad/s (kept identical to `velocity_smoother`'s `max_velocity`)
- **Controller**: MPPI, `motion_model: DiffDrive`, 20 Hz, horizon 56&times;0.05 s = 2.8 s, `batch_size: 1000`; `ObstaclesCritic.consider_footprint: true` (the robot is rectangular, so point-based checking would underestimate the swept corners)
- **Global planner**: Smac 2D, `tolerance: 0.5` m, `allow_unknown: true`
- **Goal tolerance**: XY 0.15 m, yaw 15&deg;
- **Local costmap**: 6&times;6 m rolling window, 0.05 m resolution
- **Global costmap**: Static layer + obstacle layer + inflation layer (0.4 m radius)
- **Robot footprint**: 0.69&times;0.57 m rectangle

### 5.3 LIO Switching

FAST-LIO is used by default. To switch to Point-LIO, comment/uncomment the corresponding lines in the launch scripts. Main differences:

- **LiDAR data format**: FAST-LIO uses `xfer_format=0` (PointCloud2); Point-LIO uses `xfer_format=1` (CustomMsg)
- **Configuration files**: `mid360.yaml` (FAST-LIO) vs. `mid360_real.yaml` (Point-LIO)
- **lio_interface**: `fastlio_lio_interface_launch.py` subscribes to `/Odometry`; `pointlio_lio_interface_launch.py` subscribes to `/aft_mapped_to_init`

> **Note**: the `lidar_type` enum values differ between FAST-LIO and Point-LIO and must not be mixed.

### 5.4 `global_relocalization_kiss_matcher` Parameters

The core logic of `global_kiss_matcher_relocalization_exec` has two stages:

1. **Global initialization**: Accumulates `/registered_scan`, downsamples it using `voxel_resolution`, then calls KISS-Matcher `coarseToFineAlignment()` without requiring a manual initial pose.
2. **Continuous tracking**: After successful initialization, uses the previous result as the initial guess for periodic small_gicp GICP registration, then updates and broadcasts `map` &rarr; `odom`.

Common parameters:

| Parameter | Default | Description |
|------|--------|------|
| `prior_pcd_file` | Prior PCD point cloud | Prior PCD map; must be set to an existing `.pcd` file |
| `input_cloud_topic` | `registered_scan` | Current local point-cloud input; this workspace usually uses `/registered_scan` |
| `use_global_initialization` | `true` | Whether to enable KISS-Matcher initialization without an initial guess |
| `voxel_resolution` | `0.25` | Downsampling voxel size for the KISS-Matcher initialization stage |
| `global_leaf_size` | `0.25` | small_gicp downsampling voxel size for the global map |
| `registered_leaf_size` | `0.25` | small_gicp downsampling voxel size for the current scan |
| `num_threads` | `4` | Number of small_gicp / local-registration threads |
| `num_neighbors` | `20` | Number of neighbors for covariance estimation |
| `max_dist_sq` | `1.0` | Maximum squared correspondence distance for GICP |
| `map_frame` | `map` | Global map frame |
| `odom_frame` | `odom` | LIO odometry frame |
| `base_frame` | base_footprint | Used to query the static `base_frame` &rarr; `lidar_frame` extrinsic when loading the map |
| `robot_base_frame` | base_footprint | Robot base frame used for RViz `/initialpose` correction |
| `lidar_frame` | livox_frame | LiDAR frame, usually `livox_frame` in this project |
| `init_pose` | `[0,0,0,0,0,0]` | Optional initial pose `[x,y,z,roll,pitch,yaw]` |

Current workspace defaults in the launch file:

```text
prior_pcd_file: /path/to/your_map.pcd
input_cloud_topic: /registered_scan
map_frame: map
odom_frame: odom
base_frame: base_footprint
robot_base_frame: base_footprint
lidar_frame: livox_frame
```

The global initialization stage requires sufficient geometric overlap. When the robot has just started, slowly rotate in place or move a short distance so `/registered_scan` can accumulate a more complete local structure. After successful initialization, the node automatically enters continuous tracking.

## 6. ROS 2 Topics

| Topic | Message Type | Publisher |
|------|----------|--------|
| `/livox/lidar` | PointCloud2 / CustomMsg | LiDAR driver |
| `/livox/imu` | sensor_msgs/Imu | LiDAR built-in IMU |
| `/cloud_registered` | PointCloud2 | FAST-LIO / Point-LIO |
| `/registered_scan` | PointCloud2 | sensor_scan_generation |
| `/odom` | Odometry | sensor_scan_generation |
| `/scan` | LaserScan | pointcloud_to_laserscan |
| `/cmd_vel` | Twist | Nav2 |
| `/wheel_odom` | Odometry | bunker_base (real-robot wheel odometry) |
| `/initialpose` | PoseWithCovarianceStamped | RViz |
| `/plan` | Path | Nav2 planner |
| `/tf` | TFMessage | LIO, sensor_scan_generation, relocalization nodes |

Additional usage by `global_relocalization_kiss_matcher`:

| Topic / TF | Direction | Description |
|-----------|------|------|
| `/registered_scan` | Subscribe | Current local point-cloud input |
| `/initialpose` | Subscribe | Optional manual pose-correction input |
| `base_footprint` &rarr; `livox_frame` | Query | Aligns the LiDAR extrinsic when loading the prior PCD |
| `map` &rarr; `odom` | Publish | Outputs the global relocalization result for Nav2 |

## 7. Troubleshooting

**LIO odometry diverges** - Check whether the IMU and LiDAR topics have data (`ros2 topic echo`) and confirm that `lidar_type` matches the sensor.

**TF disconnected / costmap is empty** - Enter `scripts/` and use `./show_tf_tree.sh` to inspect the TF tree, confirm `/scan` is being published, and check whether the target frame in `pointcloud_to_laserscan` matches the LiDAR frame.

**Relocalization fails** - Confirm that the PCD file exists and is not empty. Provide an approximate initial pose in RViz with "2D Pose Estimate", or try the global relocalization option.

**KISS-Matcher global relocalization keeps failing** - Check whether `/registered_scan` has data, confirm that `prior_pcd_file` points to the PCD of the current environment, ensure that the `base_footprint` &rarr; `livox_frame` TF can be queried, rotate the robot in place or move a short distance to increase accumulated point-cloud overlap, and consider increasing `voxel_resolution` to reduce memory pressure for large-map matching.

**TF jitter or Nav2 pose jumps** - Check whether `small_gicp_relocalization` and `global_relocalization_kiss_matcher` are running at the same time. Only one node may publish `map` &rarr; `odom` at once.

**Real base does not move / no CAN frames** - Check the adapter and power first: `/sys/class/net/can0` exists, the vehicle is powered on, and the **emergency stop is released**; then run `candump can0` to see whether frames are on the bus. The bitrate must be 500 kbit/s. `./chassis_start.sh --can-only` walks through these steps and reports what it finds. If the interface has another name, use `CAN_IFACE=can1 ./chassis_start.sh`.

**Two base drivers running** - Only one `bunker_base_node` may run at a time: two instances both consume `/cmd_vel` and both write CAN frames to the vehicle, with unpredictable results. Re-running `./nav2_real.sh` cleans up automatically; when starting manually, take care not to duplicate.

**Web console cannot connect / map never appears** - Confirm rosbridge is running (`./web_start.sh` exits with an explicit error when it is missing) and that the WebSocket address in the browser is `ws://<robot-ip>:9090`. `/map` is a transient_local latched topic; when the page sees nothing for 3 seconds it falls back to the `/map_server/load_map` service. If both fail, the navigation stack is not up. A semi-transparent robot arrow means TF has not updated for over 2 seconds, so go back to the localization/odometry chain.

**Real LiDAR has no data** - Check the Ethernet connection, confirm the IP addresses in `MID360_config.json`, and confirm that Livox-SDK2 is installed.

**Build fails** - Clean and rebuild:

```bash
rm -rf build/ install/ log/
cd scripts
./build.sh
```

## 8. Acknowledgements

This project is built on the following open-source projects:

- [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO/tree/ros2) - LiDAR-inertial odometry with OctVox compact mapping, HKNN correspondence search, and built-in relocalization (GPLv3, external workspace; see the [RA-L 2026 paper](https://arxiv.org/abs/2509.05723))
- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) - Tightly coupled LiDAR-IMU odometry
- [Point-LIO](https://github.com/hku-mars/Point-LIO) - High-bandwidth LiDAR-IMU odometry
- [Nav2](https://github.com/ros-planning/navigation2) - ROS 2 navigation framework
- [small_gicp](https://github.com/koide3/small_gicp) - Efficient parallelized GICP registration
- [KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher) - Fast global point-cloud registration (ICRA 2025)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) - 2D SLAM
- [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2) - Livox LiDAR SDK
- [Sophus](https://github.com/strasdat/Sophus) - Lie group C++ library

If you use this stack in research, please also cite the Super-LIO paper:

```bibtex
@article{wang2026superlio,
  title   = {Super-LIO: A Robust and Efficient LiDAR-Inertial Odometry System with a Compact Mapping Strategy},
  author  = {Wang, Liansheng and Zhang, Xinke and Li, Chenhui and He, Dongjiao and Pan, Yihan and Yi, Jianjun},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2026},
  volume  = {11},
  number  = {3},
  pages   = {2666--2673},
  doi     = {10.1109/LRA.2026.3653372}
}
```

## 9. License

This project is open source under the [MIT License](./LICENSE).

---
