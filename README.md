# Lidar_nav2_ws

[English Documentation](./README_EN.md)

基于 ROS 2 的 3D LiDAR 自主导航系统

[![ROS2](https://img.shields.io/badge/ROS2-Humble-22313F?logo=ros)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu)](https://releases.ubuntu.com/22.04/)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](./LICENSE)

<p>
  <img src="docs/KISS%20show.gif" alt="KISS demo 1" width="48%" style="margin-right: 25px;">
  <img src="docs/KISS%20show_2.gif" alt="KISS demo 2" width="48%">
</p>

**Nav2_3D** 是一个面向四轮滑移转向机器人的 ROS 2 Humble 导航工作空间，面向实机部署。系统以 Livox MID-360 3D LiDAR 和 IMU 为核心传感器，集成 LiDAR-Inertial Odometry (LIO) 里程计、3D 点云重定位、BUNKER MINI 2.0 底盘控制和 Nav2 导航框架，从 CAN 配置到导航目标下发全流程脚本化。

核心特性：

- **3D 重定位** — 基于 small_gicp + KISS-Matcher 的全局重定位
- **双 LIO 后端** — FAST-LIO2 与 Point-LIO 可灵活切换
- **实机一键启动** — `./nav2_real.sh` 按序拉起 CAN、底盘、LIO、重定位与 Nav2
- **完整工具链** — 构建、建图、保存地图、导航全流程脚本化

数据管线：

> LiDAR/IMU &rarr; LIO (FAST-LIO 或 Point-LIO) &rarr; TF 桥接 (`lio_interface`) &rarr; odom TF &amp; `/registered_scan` (`sensor_scan_generation`) &rarr; 3D&rarr;2D 切片 (`pointcloud_to_laserscan`) &rarr; 重定位 (`small_gicp_relocalization` 或 `global_relocalization_kiss_matcher`) &rarr; Nav2 (MPPI + Smac)

TF 坐标树：**`map` &rarr; `odom` &rarr; `base_footprint` &rarr; `chassis` &rarr; `livox_frame`**

---

## 1. 环境要求

- **操作系统**：Ubuntu 22.04
- **ROS 2**：Humble Hawksbill
- **Livox-SDK2**：已预编译于 `src/livox_ros_driver2/3rdparty/`，支持 amd64/arm64
- **底盘依赖**（实机模式）：`libasio-dev`、`can-utils` —— 编译 BUNKER MINI 2.0 驱动并做 CAN 总线自检
- **网页控制台依赖**（可选）：`ros-humble-rosbridge-suite` —— 浏览器与 ROS 2 之间的 WebSocket 桥

```bash
sudo apt install libasio-dev can-utils ros-humble-rosbridge-suite
```

## 2. 构建

```bash
source /opt/ros/humble/setup.bash
cd scripts
./build.sh
```

构建脚本等价于：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

每次修改源代码后需重新构建。启动任何节点前确保已执行 `source install/setup.bash`。

## 3. 快速开始

### 3.1 实机建图

```bash
source install/setup.bash
cd scripts
./mapping_real.sh
```

该脚本以 Livox MID-360 硬件驱动 (`livox_ros_driver2`) 和实机 URDF (`gld_robot_description`) 启动 FAST-LIO、`lio_interface`、`sensor_scan_generation`、3D→2D 切片和 SLAM Toolbox。

底盘用遥控器直接驾驶遍历环境。建图不启动底盘驱动，因为 LIO 的里程计不依赖轮式里程计；如果需要 ROS 侧同时有 `/cmd_vel` 和 `/wheel_odom`（例如用 `gui_teleop` 遥控），先单独运行 `./chassis_start.sh`。

覆盖足够面积后保存地图：

```bash
./save_map.sh       # 保存 2D 占用栅格地图至 src/me_nav2_bringup/map/
./save_pcd.sh       # 保存 3D 点云，手动移至 src/me_nav2_bringup/pcd/
```

### 3.2 实机导航

地图路径统一配置在 `src/me_nav2_bringup/config/map_config.yaml`，换地图只改这一个文件，且无需重新编译：

- `map_yaml_file` — Nav2 加载的 2D 栅格地图，其内部 `image` 字段指向 `.pgm`
- `prior_pcd_file` — 重定位配准用的先验 3D 点云

相对路径以 `me_nav2_bringup` 包目录为基准，也可以直接写绝对路径。`my_nav2_launch.py` 和三个重定位 launch 都读取该文件（优先读源码目录下的版本，改完立即生效）。

导航依赖 BUNKER MINI 2.0 底盘：底盘是 `/cmd_vel` 的实际执行者，并通过 CAN 把轮式里程计发布到 `/wheel_odom`。`nav2_real.sh` 已经把底盘启动串进流程，不需要额外操作：

1. 在前台配置 `can0`（`sudo` 密码就在当前终端输入），失败则整体中止；
2. 新开一个终端启动底盘驱动，等 5 秒让 CAN 连接和 `/wheel_odom` 就绪；
3. 依次拉起机器人描述、FAST-LIO、`lio_interface`、`sensor_scan_generation`、3D→2D 切片、`small_gicp` 重定位和 Nav2。

```bash
cd scripts
./nav2_real.sh
```

包含 `small_gicp_relocalization`，基于先验 PCD 地图进行重定位。在 RViz 中使用 **"Nav2 Goal"** 发送导航目标，或改用网页控制台（见 3.4）。

启动前请确认：CAN-USB 适配器已插好、底盘已上电、**急停已松开**、遥控器在手边。只想单独起底盘（例如上台架测试）时用 `./chassis_start.sh`；同一时间只能有一个底盘驱动实例，细节见 [`src/me_nav2_bringup/README_BUNKER_MINI2.md`](./src/me_nav2_bringup/README_BUNKER_MINI2.md)。

> 不要在 `nav2_real.sh` 之前手动启动底盘驱动：脚本入口的清理会连同它一起杀掉再重启，白白多等一轮。

### 3.3 全局重定位：三种方案

系统提供三种基于先验 PCD 地图的 3D 重定位方案：

- **KISS-Matcher + small_gicp**：适合机器人初始位姿未知、**"2D Pose Estimate"** 给不准，或纯 small_gicp 因初值偏差过大难以收敛的场景。`global_relocalization_kiss_matcher` 会先累计 `/registered_scan` 做全局粗配准，初始化成功后再切换到 small_gicp 连续跟踪，并持续发布 `map` &rarr; `odom`。
- **纯 small_gicp**：适合机器人初始位姿大致已知的场景。默认在机器在 (0,0,0) 附近开始收敛，可在 small_gicp_relocalization 的 launch 中自定义开机点位或者在 rviz 中 **"2D Pose Estimate"** 给定初始位姿，再由 `small_gicp_relocalization` 配准到先验地图，收敛更快、流程更简单，持续发布 `map` &rarr; `odom`。
- **ICP 配准**：适用场景和纯 small_gicp 类似，不过只有开机那一刻进行重定位，后续不再维护 `map` &rarr; `odom`。

使用前先确认先验点云路径正确：

```bash
vim src/registration/small_gicp_relocalization/launch/small_gicp_relocalization_launch.py
vim src/registration/global_relocalization_kiss_matcher/launch/global_kiss_matcher_relocalization_launch.py
```

重点检查：

- `prior_pcd_file`：先验 PCD 地图路径，例如 `src/me_nav2_bringup/pcd/*.pcd`
- `input_cloud_topic`：默认 `/registered_scan`
- `map_frame` / `odom_frame`：默认 `map` / `odom`
- `base_frame` / `robot_base_frame` / `lidar_frame`：默认 `base_footprint` / `base_footprint` / `livox_frame`

启动时二选一即可，同一时间只能有一个节点发布 `map` &rarr; `odom`。

纯 small_gicp 方案已集成在导航脚本中：

```bash
source install/setup.bash
cd scripts
./nav2_real.sh
```

如果要使用 KISS-Matcher + small_gicp，请先确保 `scripts/nav2_real.sh` 中没有同时启动 `small_gicp_relocalization`，然后单独启动全局重定位节点：

```bash
source install/setup.bash
cd scripts

# 1. 启动导航主流程
./nav2_real.sh

# 2. 启动 KISS-Matcher 全局重定位节点
ros2 launch global_relocalization_kiss_matcher global_kiss_matcher_relocalization_launch.py
```

判断是否成功：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 topic hz /registered_scan
```

日志中出现 `KISSMatcher initialization succeeded` 表示全局初始化成功，随后会进入 small_gicp 连续跟踪阶段。若持续出现 `KISSMatcher initialization failed`，通常是当前累计点云与先验地图重叠不足、点云太稀疏，或 `prior_pcd_file` / 坐标系设置不匹配。

### 3.4 网页控制台（可选）

除 RViz 外，工作空间还带一个基于 rosbridge 的网页控制台，可在同一局域网的手机或平板上操作。它不启动任何导航节点，只做浏览器与 ROS 2 之间的桥接，所以要在导航栈起来之后再启动：

```bash
# 1. 先起导航栈
cd scripts && ./nav2_real.sh

# 2. 另开终端起网页服务（rosbridge 9090 + 静态文件 5500）
./web_start.sh
```

浏览器打开脚本打印的地址：

- 导航页 `http://<机器人IP>:5500/navigation.html`：2D 地图与代价地图、单点目标、多点航点顺序导航（可循环）、设置初始位姿、取消目标。接口与 RViz 一致（`/goal_pose`、`/initialpose`），无需 `tf2_web_republisher`。
- 仪表盘页 `http://<机器人IP>:5500/index.html`：3D 点云（`/registered_scan`）、IMU、手动遥控（`/cmd_vel`，请在导航空闲时使用）。

页面话题清单与排障详见 [`src/web/README.md`](./src/web/README.md)。

## 4. 功能包

工作空间在 `src/` 下包含 30 余个 ROS 2 功能包（含 `rtabmap_ros`、`3d_bbs` 等第三方依赖），下面按用途列出主要的那些：

**里程计与定位** (`src/localization/`)

- `FAST_LIO` — FAST-LIO2：基于 ikd-Tree 的紧耦合 LiDAR-IMU 里程计，100 Hz+
- `point_lio` — Point-LIO：高带宽里程计 (4&ndash;8 kHz)，抗 IMU 饱和
- `Sophus` — 李群库 (SO(3)/SE(3))，LIO 数学依赖

**配准与重定位** (`src/registration/`)

- `small_gicp_relocalization` — 已知大概开机位姿重定位方案：基于 small_gicp 的 3D 点云配准
- `global_relocalization_kiss_matcher` — KISS-Matcher + small_gicp 全局重定位：无初值粗配准初始化，随后用 GICP 持续跟踪 `map` &rarr; `odom`
- `KISS-Matcher` — ICRA 2025：快速全局点云配准 (FPFH + TEASER++ + small_gicp)

**传感器与桥接**

- `livox_ros_driver2` — Livox MID-360 驱动，同时发布 CustomMsg 和 PointCloud2
- `lio_interface` — TF 桥接：LIO 内部坐标系转标准 `odom` 坐标系
- `sensor_scan_generation` — 发布 `odom` &rarr; `base_footprint` TF、`/odom` 和 `/registered_scan`

**导航**

- `me_nav2_bringup` — Nav2 集成：启动文件、参数配置、地图、PCD、RViz 配置
- `gui_teleop` — tkinter GUI 遥控，持速度调节和紧急停止

**底盘**（实机）

- `ugv_sdk` — AgileX 机器人 SDK：SocketCAN 协议实现，BUNKER 驱动依赖
- `bunker_base` / `bunker_msgs` — BUNKER MINI 2.0 底盘驱动：消费 `/cmd_vel`，发布 `/wheel_odom`、`/bunker_status`、`/bunker_rc_state`

**机器人描述**

- `gld_robot_description` — 实机 URDF（含 RealSense D456/D405、Orbbec Gemini 相机）

**工具**

- `pcd2pgm-master` — PCD 点云转 2D 占用栅格地图离线工具

**网页控制台**（非 ROS 功能包）

- `src/web/` — 浏览器控制台：导航页与 3D 点云仪表盘，经 rosbridge 与 ROS 2 通信

## 5. 配置说明

### 5.1 关键配置文件

**导航配置**
- `me_nav2_bringup/config/nav2_params.yaml` — Nav2 参数
- `me_nav2_bringup/config/slam_toolbox_params.yaml` — SLAM Toolbox 在线建图参数
- `me_nav2_bringup/config/Pointcloud2d_3d.yaml` — 3D→2D 切片高度和角分辨率

**LIO 配置**
- `localization/FAST_LIO/config/mid360.yaml` — FAST-LIO 参数
- `localization/point_lio/config/mid360_real.yaml` — Point-LIO 实机参数

**传感器配置**
- `livox_ros_driver2/config/MID360_config.json` — LiDAR 网络配置

**配准配置**
- `registration/icp_registration/config/icp.yaml` — ICP 配准参数
- `registration/global_relocalization_kiss_matcher/launch/global_kiss_matcher_relocalization_launch.py` — KISS-Matcher 全局重定位启动参数
- `registration/global_relocalization_kiss_matcher/config/alignment_config.yaml` — KISS-Matcher 帧间/全局配准示例参数

### 5.2 Nav2 参数要点

当前配置使用 **MPPI** 局部控制器和 **Smac 2D** 全局规划器（原为 DWB + Navfn，
已替换并实测插件正确加载）。

> ⚠ **改 `nav2_params.yaml` 后必须重新编译才生效**：`me_nav2_bringup/config/` 在
> `install/` 里是拷贝而不是符号链接，launch 读的是 install 里的那份。
> 只有 `map_config.yaml` 是例外（launch 专门写了优先读源码目录的逻辑）。
> 改完执行 `cd scripts && ./build.sh`。

- **速度限制**：线速度 0.26 m/s，角速度 1.0 rad/s（与 `velocity_smoother` 的
  `max_velocity` 保持一致，两层限制不一致会反复裁剪）
- **控制器**：MPPI，`motion_model: DiffDrive`，20 Hz，预测时域 56×0.05 s = 2.8 s，
  `batch_size: 1000`；`ObstaclesCritic.consider_footprint: true`（本车是矩形轮廓，
  关掉会低估车角扫过的区域）
- **全局规划器**：Smac 2D，`tolerance: 0.5` m，`allow_unknown: true`
- **目标容差**：XY 0.15 m，偏航角 15&deg;
- **局部代价地图**：6&times;6 m 滚动窗口，0.05 m 分辨率
- **全局代价地图**：静态层 + 障碍物层 + 膨胀层（0.4 m 半径）
- **机器人外轮廓**：0.69&times;0.57 m 矩形

### 5.3 LIO 切换

默认使用 FAST-LIO。切换至 Point-LIO 需在启动脚本中注释/取消注释对应行。主要差异：

- **LiDAR 数据格式**：FAST-LIO 使用 `xfer_format=0`（PointCloud2）；Point-LIO 使用 `xfer_format=1`（CustomMsg）
- **配置文件**：`mid360.yaml`（FAST-LIO）vs. `mid360_real.yaml`（Point-LIO）
- **lio_interface**：`fastlio_lio_interface_launch.py` 订阅 `/Odometry`；`pointlio_lio_interface_launch.py` 订阅 `/aft_mapped_to_init`

> **注意**：`lidar_type` 枚举值在 FAST-LIO 和 Point-LIO 中定义不同，不可混用。

### 5.4 `global_relocalization_kiss_matcher` 参数

`global_kiss_matcher_relocalization_exec` 的核心逻辑分两阶段：

1. **全局初始化**：累计 `/registered_scan`，按 `voxel_resolution` 降采样后调用 KISS-Matcher 的 `coarseToFineAlignment()`，不依赖人工初始位姿。
2. **连续跟踪**：初始化成功后，使用 small_gicp 以上一帧结果为初值进行周期性 GICP 配准，更新并广播 `map` &rarr; `odom`。

常用参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `prior_pcd_file` | 先验 PCD 点云 | 先验 PCD 地图，必须设置为实际存在的 `.pcd` 文件 |
| `input_cloud_topic` | `registered_scan` | 当前局部点云输入；本工作空间通常使用 `/registered_scan` |
| `use_global_initialization` | `true` | 是否启用 KISS-Matcher 无初值初始化 |
| `voxel_resolution` | `0.25` | KISS-Matcher 初始化阶段降采样体素大小 |
| `global_leaf_size` | `0.25` | 全局地图 small_gicp 降采样体素大小 |
| `registered_leaf_size` | `0.25` | 当前扫描 small_gicp 降采样体素大小 |
| `num_threads` | `4` | small_gicp / 局部配准线程数 |
| `num_neighbors` | `20` | 协方差估计邻居数量 |
| `max_dist_sq` | `1.0` | GICP 对应点最大距离平方 |
| `map_frame` | `map` | 全局地图坐标系 |
| `odom_frame` | `odom` | LIO 里程计坐标系 |
| `base_frame` | base_footprint | 加载地图时用于查询 `base_frame` &rarr; `lidar_frame` 静态外参 |
| `robot_base_frame` | base_footprint | RViz `/initialpose` 修正时使用的机器人基座坐标系 |
| `lidar_frame` | livox_frame | LiDAR 坐标系，本项目通常为 `livox_frame` |
| `init_pose` | `[0,0,0,0,0,0]` | 可选初始位姿 `[x,y,z,roll,pitch,yaw]` |

当前 launch 文件给出的工作空间默认值为：

```text
prior_pcd_file: /path/to/your_map.pcd
input_cloud_topic: /registered_scan
map_frame: map
odom_frame: odom
base_frame: base_footprint
robot_base_frame: base_footprint
lidar_frame: livox_frame
```

全局初始化阶段需要足够的几何重叠。机器人刚启动时可原地缓慢旋转或短距离移动，让 `/registered_scan` 累计到更完整的局部结构；初始化成功后节点会自动进入连续跟踪。

## 6. ROS 2 话题

| 话题 | 消息类型 | 发布者 |
|------|----------|--------|
| `/livox/lidar` | PointCloud2 / CustomMsg | LiDAR 驱动 |
| `/livox/imu` | sensor_msgs/Imu | LiDAR 内置 IMU |
| `/cloud_registered` | PointCloud2 | FAST-LIO / Point-LIO |
| `/registered_scan` | PointCloud2 | sensor_scan_generation |
| `/odom` | Odometry | sensor_scan_generation |
| `/scan` | LaserScan | pointcloud_to_laserscan |
| `/cmd_vel` | Twist | Nav2 |
| `/wheel_odom` | Odometry | bunker_base（实机底盘轮式里程计） |
| `/initialpose` | PoseWithCovarianceStamped | RViz |
| `/plan` | Path | Nav2 规划器 |
| `/tf` | TFMessage | LIO、sensor_scan_generation、重定位节点 |

`global_relocalization_kiss_matcher` 额外使用：

| 话题 / TF | 方向 | 说明 |
|-----------|------|------|
| `/registered_scan` | 订阅 | 当前局部点云输入 |
| `/initialpose` | 订阅 | 可选的人工位姿修正输入 |
| `base_footprint` &rarr; `livox_frame` | 查询 | 加载先验 PCD 时对齐 LiDAR 外参 |
| `map` &rarr; `odom` | 发布 | 输出全局重定位结果，供 Nav2 使用 |

## 7. 常见问题

**LIO 里程计发散** — 检查 IMU 和 LiDAR 话题是否有数据（`ros2 topic echo`），确认 `lidar_type` 与传感器匹配。

**TF 断开 / 代价地图空白** — 进入 `scripts/` 后使用 `./show_tf_tree.sh` 检查 TF 树，确认 `/scan` 正在发布，检查 `pointcloud_to_laserscan` 的目标坐标系是否与 LiDAR 坐标系一致。

**重定位失败** — 确认 PCD 文件存在且非空，在 RViz 中使用 "2D Pose Estimate" 给出大致初始位姿，或尝试全局重定位方案。

**KISS-Matcher 全局重定位一直失败** — 检查 `/registered_scan` 是否有数据，确认 `prior_pcd_file` 指向当前环境的 PCD；确保 `base_footprint` &rarr; `livox_frame` TF 可查询；让机器人原地旋转或移动一小段距离以增加累计点云重叠；适当增大 `voxel_resolution` 可降低大地图匹配的内存压力。

**TF 抖动或 Nav2 位姿跳变** — 检查是否同时运行了 `small_gicp_relocalization` 和 `global_relocalization_kiss_matcher`。同一时间只能有一个节点发布 `map` &rarr; `odom`。

**实机底盘不动 / 收不到 CAN 报文** — 先确认适配器和供电：`ls /sys/class/net/can0` 存在、底盘已上电、**急停已松开**，再用 `candump can0` 看总线上有没有帧。波特率必须是 500 kbit/s。`./chassis_start.sh --can-only` 会把这几步走一遍并给出提示。接口名不是 `can0` 时用 `CAN_IFACE=can1 ./chassis_start.sh`。

**出现两个底盘驱动** — 同一时间只能有一个 `bunker_base_node`：两个实例会同时消费 `/cmd_vel` 并向底盘写 CAN 帧，行为不可预期。重跑 `./nav2_real.sh` 会自动清理，手动启动时注意不要重复。

**网页连不上 / 地图不显示** — 确认 rosbridge 在运行（`./web_start.sh` 会在缺失时直接报错退出），浏览器里 WebSocket 地址应为 `ws://<机器人IP>:9090`。`/map` 是 transient_local 锁存话题，页面 3 秒收不到会自动改用 `/map_server/load_map` 服务兜底；两者都失败说明导航栈没起来。机器人箭头变半透明表示 TF 超过 2 秒没更新，回到定位/里程计排查。

**实机 LiDAR 无数据** — 检查网线连接，确认 `MID360_config.json` 中的 IP 地址，确认 Livox-SDK2 已安装。

**构建失败** — 清理后重新构建：

```bash
rm -rf build/ install/ log/
cd scripts
./build.sh
```

## 8. 致谢

本项目基于以下开源项目构建：

- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) — 紧耦合 LiDAR-IMU 里程计
- [Point-LIO](https://github.com/hku-mars/Point-LIO) — 高带宽 LiDAR-IMU 里程计
- [Nav2](https://github.com/ros-planning/navigation2) — ROS 2 导航框架
- [small_gicp](https://github.com/koide3/small_gicp) — 高效并行化 GICP 配准
- [KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher) — 快速全局点云配准 (ICRA 2025)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) — 2D SLAM
- [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2) — Livox LiDAR SDK
- [Sophus](https://github.com/strasdat/Sophus) — 李群 C++ 库

## 9. 许可证

本项目依据 [MIT License](./LICENSE) 开源。

---
