# BUNKER MINI 2.0 · Super-LIO + Nav2 自主导航栈

[English README](./README_EN.md) · [Web 控制台文档](./src/web/README.md) · [安全层文档](./src/nav2_safety/README.md) · [Super-LIO 接入说明](./src/super_lio/README.md) · [与 FAST-LIO2 的对比](./docs/FASTLIO2_COMPARISON.md)

面向 **BUNKER MINI 2.0 四轮滑移转向底盘 + Livox MID-360** 的 ROS 2 Humble 实机导航栈。
当前主用定位方案是 **Super-LIO**（RA-L 2026），并提供 Super-LIO 自带的 One-shot 重定位、
small_gicp 持续重定位、KISS-Matcher 全局初始化 + GICP 跟踪三种使用方式；同时保留
FAST-LIO2 / Point-LIO 旧链路作为回退方案。

系统不是“只跑一个 LIO 节点”的 demo，而是从 CAN 底盘、雷达驱动、LIO、TF 桥接、
3D 点云转 2D 激光、Nav2 规划控制、碰撞监控/优先级仲裁/急停，到浏览器操作台和任务执行器的
完整实机链路。

<p>
  <img src="docs/KISS%20show.gif" alt="KISS 全局重定位演示" width="48%">
  <img src="docs/KISS%20show_2.gif" alt="KISS 全局重定位演示 2" width="48%">
</p>

---

## 目录

1. [核心特性](#1-核心特性)
2. [系统架构](#2-系统架构)
3. [仓库结构](#3-仓库结构)
4. [环境要求](#4-环境要求)
5. [安装与编译](#5-安装与编译)
6. [坐标系与数据流](#6-坐标系与数据流)
7. [运行模式](#7-运行模式)
8. [建图、重定位与地图管理](#8-建图重定位与地图管理)
9. [Web 操作台](#9-web-操作台)
10. [安全层](#10-安全层)
11. [自主探索](#11-自主探索)
12. [配置参考](#12-配置参考)
13. [话题、TF 与 Action 清单](#13-话题tf-与-action-清单)
14. [标准操作流程](#14-标准操作流程)
15. [测试](#15-测试)
16. [故障排查](#16-故障排查)
17. [已知限制与安全边界](#17-已知限制与安全边界)
18. [与 FAST-LIO2 的对比](#18-与-fast-lio2-的对比)
19. [许可证与致谢](#19-许可证与致谢)
20. [常用命令速查](#20-常用命令速查)

---

## 1. 核心特性

- **Super-LIO 单坐标系 LIO + 桥接层**：Super-LIO 原生只发布 `world -> imu` 一个 TF；
  `super_lio_bridge` 把它拆成 Nav2 需要的 `map -> odom` 与 `odom -> base_footprint`，
  并把点云转发为 `/registered_scan`、输出 `/odom` 与定位健康度。
- **四种定位/建图模式**：
  - `mapping`：纯建图，退出时保存 3D 点云；
  - `relocation`：使用 Super-LIO 自带重定位，加载先验地图，只初始化一次；
  - `tracking`：Super-LIO 只当里程计，`map -> odom` 由 small_gicp 以约 2 Hz 持续修正；
  - `tracking_kiss`：KISS-Matcher 无初值全局初始化 + GICP 持续跟踪 + 丢失后自动 KISS 恢复。
- **FAST-LIO2 / Point-LIO 回退链路**：旧的 `FAST-LIO + lio_interface +
  sensor_scan_generation + small_gicp + Nav2` 链路仍然保留，可用 `scripts/nav2_real.sh`
  启动，与 Super-LIO 链路二选一。
- **Nav2 实机调参**：SmacPlanner2D 全局规划 + MPPI 局部控制，矩形足迹碰撞检查，
  目标容差、速度限制、局部/全局代价地图均按 BUNKER MINI 2.0 调整。
- **独立安全层 `nav2_safety`**：速度平滑后接碰撞监控、`twist_mux` 优先级仲裁、
  兜底零速、定位看门狗、锁存式软件急停、恢复行为方向性守卫。
- **自主探索链路**：`grid_mapper` 从 Super-LIO 点云实时生长 `/map`，
  `explore_lite` 做前沿探索，支持方向引导、边界限制、目标承诺和失败黑名单。
- **Web 操作台**：工作区/巡检方案、地图上传与选点、点位朝向与停留时间、
  文本标注、目标确认后下发、航点编辑、任务队列、任务历史、机器人/电池/IMU 状态、
  3D 点云、软件急停。后端断浏览器也不中断任务。
- **实机工程化**：启动前清理残留节点、启动后自检、QoS 中继、低内存顺序编译、
  CAN 总线自检、按工作区隔离地图/点位/任务。

> 本仓库是 **2026-10-20 的干净快照**。Super-LIO 上游核心代码不在仓库内（GPLv3，
> 独立工作空间编译）；本仓库包含 MIT 的桥接层、Nav2/安全层/Web/探索集成，
> 以及 FAST-LIO2/Point-LIO 回退链路的源码。

---

## 2. 系统架构

### 2.1 数据流

```
                    Livox MID-360
             /livox/lidar (CustomMsg)     /livox/imu
                        │                     │
                        └──────────┬──────────┘
                                   ▼
                    Super-LIO（独立工作空间，GPLv3）
                    /lio/odom (frame=world)
                    /lio/cloud_world (frame=world)
                                   │
                         super_lio_bridge（本仓库，MIT）
          ┌────────────────────────┼────────────────────────┐
          ▼                        ▼                        ▼
   map -> odom              odom -> base_footprint      /registered_scan
   （internal 模式）          （连续里程计）               /localization/tracking_ok
          │                        │                        │
          └────────────┬───────────┘                        │
                       ▼                                    ▼
              pointcloud_to_laserscan  ──> /scan ──> scan_qos_relay ──> /scan_reliable
                       │
                       ▼
        Nav2：SmacPlanner2D + MPPI + 代价地图 + 行为树
                       │
             /cmd_vel_nav_smoothed
                       ▼
        collision_monitor（独立避障）
                       ▼
             /cmd_vel_nav_checked
                       ▼
        twist_mux（急停 255 > 遥控 100 > 看门狗 50 > 自动 10）
                       ▼
               /cmd_vel_muxed
                       ▼
        cmd_vel_deadman（静默兜底零速）
                       ▼
                   /cmd_vel
                       ▼
        bunker_base / CAN 500k ──> BUNKER MINI 2.0
```

恢复行为走独立通道，避免绕过安全链：

```
behavior_server (spin/backup/smart_recovery)
        └─> /cmd_vel_recovery
              └─> recovery_guard（方向性放行）
                    └─> /cmd_vel_recovery_checked
                          └─> twist_mux(priority=20) ──> ... ──> /cmd_vel
```

### 2.2 TF 树

```
map ──(Super-LIO / 重定位节点)──> odom ──(super_lio_bridge)──> base_footprint
                                                              ├── chassis
                                                              │     └── livox_frame
                                                              ├── D456_1 / D456_2
                                                              ├── D405_1 / D405_2
                                                              └── orbbec_gemini_1..3
```

- `base_footprint -> chassis`：`z +0.06 m`
- `chassis -> livox_frame`：`z +0.26 m`
- 因此 `base_footprint -> livox_frame` 约 `z +0.32 m`，桥接层的 `imu_to_base` 默认取
  `[0, 0, -0.32, 0, 0, 0]`。**这是当前精度最弱的一环**，建议实机标定后修改
  `src/super_lio/super_lio_bridge/config/bridge.yaml`。

### 2.3 两套 LIO 链路的关系

| | Super-LIO 链路（当前主用） | FAST-LIO2 回退链路 |
|---|---|---|
| 启动脚本 | `scripts/super_lio_nav.sh` | `scripts/nav2_real.sh` |
| LIO 输出 | `/lio/odom`（`world`） | `/Odometry`（`camera_init`） |
| 中间层 | `super_lio_bridge` | `lio_interface` + `sensor_scan_generation` |
| 重定位 | Super-LIO relocation / small_gicp / KISS | small_gicp 或 KISS |
| 能否同时运行 | **不能** | **不能** |

两者都会发布 `map -> odom` 和消耗 `/cmd_vel`，同时运行会抢 TF 和控制权。

---

## 3. 仓库结构

```
Lidar_nav2_ws/
├── README.md                        # 本文件
├── README_EN.md                     # 英文说明（旧版，当前以中文手册为准）
├── LICENSE                          # MIT（本仓库自有代码）
├── docs/
│   ├── FASTLIO2_COMPARISON.md       # Super-LIO 链路 vs FAST-LIO2 链路
│   ├── KISS show.gif                # KISS 重定位演示
│   └── ndt_demo.html
├── scripts/
│   ├── build.sh                     # 低内存顺序编译
│   ├── chassis_start.sh             # CAN + BUNKER 底盘驱动
│   ├── super_lio_nav.sh             # Super-LIO 主启动脚本
│   ├── super_lio_explore.sh         # 探索启动 + 启动前后自检
│   ├── explore_record.sh            # 探索 + rosbag + 配置快照
│   ├── cleanup_ros_stack.sh         # 清理残留 ROS 进程
│   ├── web_start.sh                 # rosbridge + Web 后端
│   ├── web_backend.sh               # 只启动 Web 后端
│   ├── save_map.sh                  # 保存 2D 栅格图
│   ├── save_pcd.sh                  # 旧 FAST-LIO 链路的 PCD 保存服务
│   ├── pcd_to_occupancy_map.py      # PCD -> PGM/YAML
│   ├── test_mission_executor.py     # 任务执行器离线测试
│   └── voice_agent_start.sh         # 语音 Agent
├── src/
│   ├── super_lio/super_lio_bridge/  # Super-LIO -> Nav2 桥接（MIT）
│   ├── me_nav2_bringup/             # Nav2 启动、参数、地图、行为树、恢复插件
│   ├── nav2_safety/                 # 安全层与状态聚合
│   ├── grid_mapper/                 # 实时 2D 占据栅格
│   ├── web/                         # Web 工作台 + 旧导航页 + 后端静态资源
│   ├── registration/                # small_gicp / KISS 重定位 + KISS-Matcher
│   ├── localization/                # FAST-LIO2 / Point-LIO / Sophus（回退链路）
│   ├── livox_ros_driver2/           # Livox MID-360 驱动 + Livox-SDK2
│   ├── bunker_ros2-humble/          # BUNKER 底盘驱动
│   ├── ugv_sdk/                     # AgileX SocketCAN SDK
│   ├── m-explore-ros2/              # 前沿探索（explore_lite）
│   ├── pcd2pgm-master/              # PCD 转栅格工具
│   ├── gld_robot_description/       # 实机 URDF / 静态 TF
│   ├── lio_interface/               # 旧链路的 LIO -> odom TF 桥
│   ├── sensor_scan_generation/      # 旧链路的 odom -> base TF 与 /registered_scan
│   ├── gui_teleop/                  # tkinter 遥控
│   └── voice_agent/                 # 语音交互（可选）
└── web_backend_data/                # 运行时生成，已被 .gitignore 排除
```

> `web_backend_data/`、`runs/`、`build/`、`install/`、`log/`、PCD/rosbag 都不进仓库。
> 首次运行 Web 工作台时 `web_backend_data/` 会自动创建。

---

## 4. 环境要求

### 4.1 硬件

| 部件 | 说明 |
|---|---|
| 底盘 | BUNKER MINI 2.0（AgileX），CAN 500 kbit/s |
| 雷达 | Livox MID-360（内置 IMU），网口连接 |
| CAN 适配器 | gs_usb 兼容，通常识别为 `can0` |
| 遥控器 | 建图/手动兜底；SWB 拨到“遥控模式”时由遥控器控制 |
| 急停 | 底盘硬件急停必须保留；软件急停只是补充 |

### 4.2 软件

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- C++17 / C++20（Super-LIO 上游要求 C++20）
- 推荐内存 ≥ 8 GB；编译时默认单线程，避免小内存机器卡死

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  libasio-dev can-utils \
  libgoogle-glog-dev libtbb-dev \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-rosbridge-suite ros-humble-twist-mux \
  ros-humble-slam-toolbox \
  python3-pip

python3 -m pip install fastapi uvicorn python-multipart pyyaml pillow
```

> `KISS-Matcher`、`livox_ros_driver2`、`m-explore-ros2`、`bunker_ros2-humble` 等
> 已随仓库提供，不需要额外下载。

### 4.3 网络

Livox MID-360 的默认网络配置见
`src/livox_ros_driver2/config/MID360_config.json`：

- 雷达 IP：`192.168.1.170`
- 主机 IP：`192.168.1.50`
- 数据端口：`56100/56200/56300/56400/56500` 系列

把主机对应网口配置到 `192.168.1.50/24`，确认能 `ping 192.168.1.170`。
如果现场网段不同，修改该 JSON 后重新编译/启动驱动。

CAN 适配器：

```bash
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
candump can0          # 应能看到底盘报文
```

接口不是 `can0` 时：

```bash
CAN_IFACE=can1 ./scripts/chassis_start.sh
```

---

## 5. 安装与编译

### 5.1 编译本工作空间

```bash
cd /path/to/Lidar_nav2_ws
source /opt/ros/humble/setup.bash
./scripts/build.sh
```

`build.sh` 默认 **`--executor sequential --parallel-workers 1` + `MAKEFLAGS=-j1`**，
这是为了避免 rtabmap 等大翻译单元在小内存机器上把系统压到卡死。内存充足时可以：

```bash
BUILD_JOBS=4 ./scripts/build.sh
```

编译完成后：

```bash
source install/setup.bash
```

### 5.2 编译 Super-LIO 上游（必须）

Super-LIO 核心是 **独立工作空间**，且为 GPLv3；本仓库不包含它的代码。
默认路径：

```bash
export SUPERLIO_WS=/path/to/Super-LIO-ros2
```

首次编译时，必须先 source 本工作空间，让 Super-LIO 找到 `livox_ros_driver2`：

```bash
source /opt/ros/humble/setup.bash
source /path/to/Lidar_nav2_ws/install/setup.bash
cd "$SUPERLIO_WS"
colcon build --symlink-install
```

如果路径不是默认值，运行脚本时传入：

```bash
SUPERLIO_WS=/path/to/Super-LIO-ros2 ./scripts/super_lio_nav.sh relocation false
```

### 5.3 可选：启用 FAST-LIO2 回退链路

`src/localization/FAST_LIO`、`point_lio`、`Sophus` 已包含在仓库中，`build.sh` 会一起编译。
旧链路启动：

```bash
./scripts/nav2_real.sh
```

它与 Super-LIO 链路**二选一**，不要同时启动。

### 5.4 编译常见问题

| 现象 | 处理 |
|---|---|
| 编译时整机卡死/失去响应 | 保持默认 `BUILD_JOBS=1`，关闭浏览器/大内存程序；不要用 `-j16` |
| 找不到 `super_lio` 包 | 没有 source Super-LIO 工作空间，或它没编译成功 |
| 找不到 `livox_ros_driver2`（编译 Super-LIO 时） | 先 source 本工作空间，再编译 Super-LIO |
| 修改 `nav2_params.yaml` 不生效 | 它被安装到 `install/` 的拷贝里；重新执行 `./scripts/build.sh` |
| 修改 `map_config.yaml` 不想重编译 | 该文件有源码目录优先读取逻辑，改完可直接重启导航 |

---

## 6. 坐标系与数据流

### 6.1 Super-LIO 的单坐标系输出

Super-LIO 原生输出：

| 话题 | 类型 | frame |
|---|---|---|
| `/lio/odom` | `nav_msgs/Odometry` | `world` |
| `/lio/cloud_world` | `sensor_msgs/PointCloud2` | `world` |
| TF | `world -> imu` | — |

`world` 在 relocation 模式下就是先验地图坐标系；在 mapping/tracking 模式下是启动时
IMU 所在的局部坐标系。

### 6.2 桥接层做了什么

`super_lio_bridge/tf_bridge.py` 只通过 ROS 话题/TF 与 Super-LIO 交互，不链接它的库：

```
odom -> base_footprint = 相对启动位姿的运动（连续、无跳变）
map  -> odom           = internal 模式下启动瞬间的绝对位姿（常量）
                          external 模式下由 small_gicp/KISS 节点发布
```

- `localization_mode=internal`：Super-LIO relocation 模式，桥接层用第一帧锚定
  `map -> odom`，之后保持不变；`/localization/tracking_ok` 由桥接层按位姿新鲜度发布。
- `localization_mode=external`：Super-LIO 只当里程计；桥接层不发布 `map -> odom`，
  由 `small_gicp_relocalization` 或 `global_relocalization_kiss_matcher` 发布。

> 单坐标系 LIO 的漂移会直接体现在 `map` 系位姿上，没有独立修正机制。
> 这是 Super-LIO relocation 模式的固有特性；需要持续修正时用 `tracking` 或
> `tracking_kiss`。

### 6.3 3D 点云 -> 2D 激光

`pointcloud_to_laserscan_launch.py` 使用 `Pointcloud2d_3d.yaml`：

- `target_frame: livox_frame`
- `min_height: -0.20`（约离地 0.12 m）
- `max_height: 1.20`（约离地 1.52 m）
- `angle_increment: 0.0087`（约 0.5°）
- `range_max: 70.0`

输出 `/scan`，再由 `scan_qos_relay` 转成 Reliable 的 `/scan_reliable` 给 Nav2
代价地图使用。

---

## 7. 运行模式

`scripts/super_lio_nav.sh` 支持五种模式：

```bash
./scripts/super_lio_nav.sh <mode> [rviz]
```

| 模式 | 命令 | LIO 作用 | `map -> odom` 来源 | 适用场景 |
|---|---|---|---|---|
| `mapping` | `./scripts/super_lio_nav.sh mapping` | 建图，退出时保存 `map_explore.pcd` | 桥接层按启动位姿锚定 | 首次建图、探索建图 |
| `relocation` | `./scripts/super_lio_nav.sh relocation` | Super-LIO 自带重定位（一次性初始化） | Super-LIO 通过桥接层 | 已知地图、起点大致明确、不需要持续修正 |
| `tracking` | `./scripts/super_lio_nav.sh tracking` | 只做里程计 | `small_gicp_relocalization` 约 2 Hz 持续修正 | 已知地图、希望持续纠偏 |
| `tracking_kiss` | `./scripts/super_lio_nav.sh tracking_kiss` | 只做里程计 | KISS 全局初始化 + GICP 跟踪 + 失败自动恢复 | 起点未知、需要自动找回 |
| `exploration` | `./scripts/super_lio_nav.sh exploration` | 建图模式 | 桥接层 | 未知环境自主探索 |

默认 `mode=tracking`，默认打开 RViz。关闭 RViz：

```bash
./scripts/super_lio_nav.sh tracking false
```

### 7.1 底盘

Super-LIO 链路只负责定位/导航，底盘单独启动：

```bash
./scripts/chassis_start.sh
```

它会：

1. 加载 `gs_usb`；
2. 拉起 `can0` 并设置 500 kbit/s；
3. 用 `candump` 采样确认总线有报文；
4. 启动 `bunker_mini2_base.launch.py`。

只配置 CAN、不启动驱动：

```bash
./scripts/chassis_start.sh --can-only
```

> 同一时间只能有一个底盘驱动实例。两个实例会同时消费 `/cmd_vel` 并写 CAN，
> 行为不可预期。

### 7.2 启动顺序建议

**建图：**

```bash
# 终端 1
./scripts/chassis_start.sh

# 终端 2
./scripts/super_lio_nav.sh mapping
```

用遥控器把车开一圈（SWB 拨到遥控模式），然后 Ctrl-C 退出。
Super-LIO 会把分片点云合并保存到：

```
$SUPERLIO_WS/src/super_lio/map/map_explore.pcd
```

**重定位 + 导航：**

```bash
# 终端 1
./scripts/chassis_start.sh

# 终端 2
./scripts/super_lio_nav.sh relocation
# 或
./scripts/super_lio_nav.sh tracking
# 或
./scripts/super_lio_nav.sh tracking_kiss
```

如果车不在先验地图原点附近，用 RViz 或网页的“初始位姿”发 `/initialpose`。

**Web 操作台：**

```bash
# 导航栈起来后
./scripts/web_start.sh
```

浏览器打开脚本打印的地址。

### 7.3 模式选择建议

- **第一次跑、没有地图**：`mapping`。
- **有地图、起点大致在原点**：`relocation`，最简单。
- **有地图、起点不确定，但希望运行中持续纠偏**：`tracking`。
- **有地图、起点完全不确定，且允许 KISS 初始化耗时**：`tracking_kiss`。
- **没有地图、要自主探索**：`exploration`。

### 7.4 自动清理与回放

启动脚本默认会调用 `cleanup_ros_stack.sh` 清理上次残留的同栈进程；
不想清理时：

```bash
SLIO_NO_CLEAN=1 ./scripts/super_lio_nav.sh tracking false
```

无硬件回放数据包：

```bash
SLIO_NO_DRIVER=1 SLIO_USE_SIM_TIME=1 ./scripts/super_lio_nav.sh mapping false
ros2 bag play /path/to/bag --clock
```

> `SLIO_NO_DRIVER=1` 和 `SLIO_USE_SIM_TIME=1` 必须成对使用，否则 TF 时间戳会错位。

---

## 8. 建图、重定位与地图管理

### 8.1 Super-LIO 建图

```bash
./scripts/super_lio_nav.sh mapping
```

关键配置：`src/super_lio/super_lio_bridge/config/super_lio_mid360_mapping.yaml`

- `lio.map.save_map: true` —— 上游默认是 `false`，不改就永远没有地图；
- `lio.map.save_map_dir: "map"` —— **这是后缀，不是绝对路径**，实际落盘在
  `$SUPERLIO_WS/src/super_lio/map/`；
- `lio.map.map_name: "map_explore.pcd"` —— 建图/探索产出，避免覆盖重定位用的 `map.pcd`；
- `lio.map.save_interval: 500` —— 每 500 帧落一个分片，退出时合并；
- `lio.sensor.blind: 0.4` —— 避免车贴近墙时墙从点云中消失；
- `lio.sensor.filter_rate: 3` —— **必须是整数**，写成 `3.0` 会直接 SIGABRT。

### 8.2 Super-LIO 自带重定位

把建图产出的 PCD 命名为 `map.pcd` 放到：

```
$SUPERLIO_WS/src/super_lio/map/map.pcd
```

然后：

```bash
./scripts/super_lio_nav.sh relocation
```

配置：`super_lio_mid360_relocation.yaml`

- `lio.map.save_map: false`
- `lio.map.map_name: "map.pcd"`
- `lio.relocation.init_pose: [x, y, z, roll, pitch, yaw]`
- 运行时可用 `/initialpose` 覆盖初始猜测。

> 重定位模式只初始化一次；如果中途跟丢，需要重新启动或发 `/initialpose`。

### 8.3 tracking / tracking_kiss 的先验点云

这两种模式使用 **外部重定位节点**，读的是：

```yaml
# src/me_nav2_bringup/config/map_config.yaml
prior_pcd_file: "pcd/map.pcd"
map_yaml_file: "map/nav_test_4_27_from_pcd.yaml"
```

- 相对路径以 `src/me_nav2_bringup/` 为基准；
- 也可以写绝对路径；
- `tracking` 启动 `small_gicp_relocalization`；
- `tracking_kiss` 启动 `global_relocalization_kiss_matcher`
  （KISS 全局初始化 + GICP 跟踪 + 自动恢复）。

`tracking_kiss` 的关键参数在
`src/registration/global_relocalization_kiss_matcher/launch/global_kiss_matcher_relocalization_launch.py`：

| 参数 | 默认 | 说明 |
|---|---|---|
| `prior_pcd_file` | `map_config.yaml` 的 `prior_pcd_file` | 先验 PCD |
| `input_cloud_topic` | `/registered_scan` | 当前扫描 |
| `voxel_resolution` | 0.25 | KISS 初始化降采样 |
| `global_leaf_size` | 0.25 | 全局图降采样 |
| `registered_leaf_size` | 0.25 | 当前扫描降采样 |
| `num_threads` | 4 | 配准线程数 |
| `loop.num_inliers_threshold` | 3 | 全局匹配内点阈值 |
| `loop.overlap_threshold` | 80.0 | 重叠率阈值 |

### 8.4 保存 2D 栅格地图

旧链路（FAST-LIO + slam_toolbox 或 map_server）可直接保存：

```bash
./scripts/save_map.sh
# 默认输出 src/me_nav2_bringup/map/test_map_2.pgm/.yaml
```

从 3D PCD 离线投影：

```bash
python3 scripts/pcd_to_occupancy_map.py \
  /path/to/map.pcd \
  /path/to/output/map \
  --resolution 0.05
```

常用参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `--resolution` | 0.05 | 栅格分辨率 |
| `--ground-min/max` | -0.35 / 0.15 | 地面高度带 |
| `--obstacle-min/max` | 0.20 / 1.50 | 障碍高度带 |
| `--padding` | 0.50 | 地图边界留白 |
| `--min-ground-hits` | 3 | 地面确认阈值 |
| `--min-obstacle-hits` | 30 | 障碍确认阈值 |

### 8.5 在 Web 工作台里管理地图

工作区页支持：

1. 选择 PGM + 配套 YAML（可分别选、可多选、可选整个文件夹）；
2. 上传后自动加载并作为当前工作区地图；
3. 扫描已有地图并绑定；
4. 在离线地图上选点、设置停留时间和朝向、写文本标注；
5. 上传先验 PCD 给 tracking / tracking_kiss 使用。

切换工作区会把该工作区的二维地图和先验 PCD 写入
`src/me_nav2_bringup/config/map_config.yaml`，**需要重启导航栈才会真正加载新地图**。

---

## 9. Web 操作台

完整接口和页面说明见 [src/web/README.md](./src/web/README.md)。这里给出操作视角的说明。

### 9.1 启动

```bash
# 导航栈已启动后
./scripts/web_start.sh
```

脚本会：

- 检查 `rosbridge_server` 是否安装；
- 未运行时自动开终端启动 rosbridge（9090）；
- 启动 `scripts/web_backend.py`（5500，静态页面 + API + 任务执行器）。

访问：

```
http://<机器人IP>:5500/workbench.html     # 新版工作台
http://<机器人IP>:5500/navigation.html    # 旧版导航页
http://<机器人IP>:5500/cockpit.html       # 座舱页
http://<机器人IP>:5500/index.html         # 3D 点云仪表盘
```

### 9.2 页面与功能

| 页面 | 功能 |
|---|---|
| 工作区 | 工作区/巡检方案管理、地图上传与绑定、选点、停留时间、朝向、文本标注、先验 PCD |
| 导航 | 2D 地图、代价地图、`/plan` 规划路径、机器人位姿、目标/航点/初始位姿 |
| 建图 | 建图状态与地图预览 |
| 探索 | 自动/手动探索、暂停/继续、边界与方向引导 |
| 任务 | 任务队列、巡检方案、服务端执行、暂停/继续/取消、历史记录 |
| 机器人 | 电池电压/SOC/电流/温度/SOH、IMU、手动控制 |
| 工具 | 保存地图、PCD 转栅格、TF/诊断命令提示 |
| 系统 | 运行链、健康检查、安全状态、恢复行为 |
| 设置 | rosbridge 地址、话题、图层、手动速度上限 |
| 三维点云 | 内嵌 ros3d，2 Hz / 48000 点上限 |

### 9.3 目标下发（重点）

「目标」模式下点击地图 **不会立即让机器人移动**：

1. 点击地图 → 出现橙色虚线箭头和“待下发”；
2. 拖动箭头，或在“朝向”输入框填写角度（度，0=东，90=北）；
3. 点“下发”或按回车 → 目标才发给 Nav2；
4. 点“取消” → 放弃本次选点。

目标执行中不能直接改朝向，需要先“取消任务”。

### 9.4 航点

- 「航点」模式点击地图添加航点；
- 每个航点可设置到达后的朝向：地图上按住航点拖动，或在列表里输入角度；
- 「开始导航」按顺序执行，到达后自动发下一个；
- 支持循环、撤销、清空、单点 120 秒超时；
- 航点到达判定基于 action 状态话题，做了重复状态去重。

### 9.5 工作区与巡检方案

- 每个工作区有独立的地图、先验 PCD、点位、标注和任务历史；
- 同一工作区可以有多个巡检方案，共用地图和标注，点位表独立；
- 数据目录：`web_backend_data/workspaces/ws_<id>_<slug>/`
- 默认工作区会从旧的 `config/waypoints.yaml`、`map_config.yaml`、`map_annotations.yaml`
  迁移数据；
- 默认方案会同步一份镜像到 `src/me_nav2_bringup/config/waypoints.yaml`，
  供语音 Agent 等旧节点读取；
- 删除工作区会级联删除其任务、历史、方案和目录。

### 9.6 服务端任务执行

- 任务由后端通过 `NavigateToPose` action 执行；
- 关闭浏览器不会中断任务；
- 支持暂停/继续/取消/重置；
- 后端重启后，未完成任务标记为 `interrupted`，可继续；
- 每个点位可带 `dwell`（到达后停留秒数）和 `yaw`（目标朝向）；
- 任务历史写入 `web_backend_data/missions.db` 的 `mission_runs` 表。

### 9.7 鉴权

默认 **无鉴权**，只适合可信局域网。启用 Bearer Token：

```bash
BUNKER_WEB_TOKEN='足够长的随机串' ./scripts/web_start.sh
# 或
./scripts/web_backend.sh --port 5500 --token '足够长的随机串'
```

启用后 `/api/*` 需要：

```
Authorization: Bearer <token>
```

静态页面仍可打开，页面会弹出登录框。**这不是 TLS**；不可信网络请加反向代理和 HTTPS。

### 9.8 数据与持久化

```
web_backend_data/
├── missions.db                 # 任务模板、运行历史
├── runtime_mission.json        # 当前任务状态（后端重启恢复）
├── active_workspace.json       # 当前工作区
├── workspaces/
│   └── ws_<id>_<slug>/
│       ├── points.yaml
│       ├── annotations.yaml
│       ├── maps/
│       ├── plans/plan_<id>_<slug>/points.yaml
│       └── prior_pcd/
└── map_uploads/                # 浏览器上传的地图
```

> 这些文件都是运行时数据，已在 `.gitignore` 中排除。

---

## 10. 安全层

安全层由 `src/nav2_safety` 提供，启动时与 Nav2 一起拉起。详细设计与实测记录见
[src/nav2_safety/README.md](./src/nav2_safety/README.md)。

### 10.1 速度链

```
controller_server  -> cmd_vel_nav
velocity_smoother  -> cmd_vel_nav_smoothed
collision_monitor  -> cmd_vel_nav_checked
twist_mux          -> cmd_vel_muxed
deadman            -> /cmd_vel
```

### 10.2 优先级

| 优先级 | 来源 | 说明 |
|---|---|---|
| 255 | 急停锁 `/safety/estop_active` | 压制一切，包括遥控 |
| 100 | 遥控 `/cmd_vel_teleop` | 人工接管 |
| 50 | 看门狗 `/safety/autonomy_blocked` | 只锁自动驾驶，遥控保留 |
| 20 | 恢复行为 `/cmd_vel_recovery_checked` | 脱困专用 |
| 10 | 自动驾驶 `/cmd_vel_nav_checked` | Nav2 正常输出 |

### 10.3 组件

| 组件 | 职责 |
|---|---|
| `collision_monitor` | 按真实足迹预测 1 s 扫掠碰撞，独立于规划器 |
| `twist_mux` | 多发布者优先级仲裁 + 锁 |
| `cmd_vel_deadman` | 上游静默时持续发零速（0.3 s） |
| `localization_watchdog` | TF/tracking_ok 不健康时锁自动驾驶 |
| `estop_latch` | 把一次性急停指令锁存成持续状态 |
| `recovery_guard` | 恢复行为的方向性碰撞守卫 |
| `robot_status_monitor` | 聚合 `/robot/status` JSON |
| `scan_qos_relay` | `/scan` Best Effort -> `/scan_reliable` Reliable |

### 10.4 软件急停

- 触发：网页长按急停按钮（1.2 s），或向 `/safety/estop` 发 `std_msgs/Bool true`；
- 状态：`/safety/estop_active`（transient_local，持续发布）；
- 解除：网页长按解除按钮，或发 `false`；
- 锁存状态落盘到 `/tmp/nav2_safety_estop.state`，节点重启后恢复；
- **软件急停不能替代底盘硬件急停**。

### 10.5 已知安全边界

- `localization_watchdog` 只看话题/TF 是否新鲜，不做位姿跳变或配准残差判断；
- Super-LIO 的 `tracking_ok` 是**弱信号**：只能发现“位姿不再更新”，发现不了
  “重定位收敛到错误位姿”；
- `collision_monitor` 的预测窗口按当前 0.4 m/s 调；放大 `max_vel_x` 时必须同步加大
  `time_before_collision`；
- 这是软件层安全，不是功能安全认证（ISO 3691-4 需要安全 PLC、安全激光雷达、
  双回路急停等硬件冗余）。

---

## 11. 自主探索

### 11.1 组成

```
Super-LIO（mapping 模式，不加载先验图）
      │ /lio/odom + /lio/cloud_world
      ▼
grid_mapper（实时生长 /map）
      │
      ▼
Nav2 全局代价地图（track_unknown_space: true）
      │
      ▼
explore_lite（前沿探索）
      │ NavigateToPose
      ▼
Nav2 + 安全层 -> /cmd_vel
```

关键点：

- 探索时必须**关闭 map_server**，否则它和 `grid_mapper` 同时发布 `/map`，
  全局代价地图会在静态先验图和实时图之间跳；
- `grid_mapper` 用 log-odds 累积、2 秒稳定确认、射线清理，过滤行人短时足迹；
- `explore_lite` 用 `/global_costmap/costmap` 而不是静态 `/map`，让前沿和规划器看到同一张图。

### 11.2 启动

```bash
./scripts/chassis_start.sh          # 终端 1
./scripts/super_lio_explore.sh      # 终端 2（默认 auto）
# 或带 RViz
./scripts/super_lio_explore.sh rviz
```

手动模式（RViz 2D Goal Pose 直接交给 Nav2）：

```bash
./scripts/super_lio_explore.sh rviz manual
```

纯启动、跳过启动前后自检：

```bash
./scripts/super_lio_nav.sh exploration false
```

### 11.3 探索参数

`src/me_nav2_bringup/config/explore_params.yaml` 的关键参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `planner_frequency` | 0.33 | 前沿重算频率约 3 s |
| `progress_timeout` | 30.0 | 无进展判定 |
| `no_valid_frontier_grace` | 60.0 | 无前沿等待时间 |
| `potential_scale` | 5.0 | 距离惩罚 |
| `gain_scale` | 1.0 | 信息增益权重 |
| `min_frontier_size` | 0.75 | 最小前沿宽度（车宽 0.69 m） |
| `goal_commit` | true | 目标承诺，避免来回跳 |
| `switch_improvement` | 0.3 | 新目标需优 30% 才切换 |
| `blacklist_timeout` | 120.0 | 失败前沿拉黑时间 |
| `guide_mode` | filter | 方向引导只保留锥内前沿 |
| `guide_navigation` | true | 引导点可直接作为导航目标 |

### 11.4 探索的常见失败

| 现象 | 原因 |
|---|---|
| 车不动，但 explore 有目标 | 安全看门狗锁住自动驾驶；检查 `/localization/tracking_ok`、`/safety/autonomy_blocked` |
| 地图来回跳 | 残留 `map_server` 或重复 `grid_mapper`；`/map` 只能有一个发布者 |
| 前沿一直不变 | 没有未知区域，或 `/map` 不是增长图 |
| 车在门口来回 | 前沿宽度/目标承诺参数；调 `min_frontier_size`、`switch_improvement` |
| 探索过早结束 | `no_valid_frontier_grace` 太短，或所有前沿被拉黑 |

---

## 12. 配置参考

### 12.1 关键配置文件

| 文件 | 作用 |
|---|---|
| `src/me_nav2_bringup/config/map_config.yaml` | 2D 地图 YAML、先验 PCD 路径 |
| `src/me_nav2_bringup/config/nav2_params.yaml` | Nav2 全部参数 |
| `src/me_nav2_bringup/config/Pointcloud2d_3d.yaml` | 3D 点云转 2D 激光高度/角度 |
| `src/me_nav2_bringup/config/explore_params.yaml` | 前沿探索参数 |
| `src/me_nav2_bringup/config/slam_toolbox_params.yaml` | 旧链路 2D SLAM |
| `src/super_lio/super_lio_bridge/config/bridge.yaml` | Super-LIO 桥接、外参、超时 |
| `src/super_lio/super_lio_bridge/config/super_lio_mid360_mapping.yaml` | 建图/里程计 |
| `src/super_lio/super_lio_bridge/config/super_lio_mid360_relocation.yaml` | Super-LIO 重定位 |
| `src/nav2_safety/config/*.yaml` | 安全层各节点 |
| `src/grid_mapper/config/grid_mapper.yaml` | 实时栅格 |
| `src/livox_ros_driver2/config/MID360_config.json` | 雷达网络 |
| `src/gld_robot_description/model/gld_robot_description.urdf` | 机器人 URDF / 静态 TF |

### 12.2 Nav2 关键参数（当前值）

| 项 | 值 |
|---|---|
| 局部控制器 | `nav2_mppi_controller::MPPIController` |
| 全局规划器 | `nav2_smac_planner/SmacPlanner2D` |
| 控制频率 | 20 Hz |
| MPPI 预测时域 | 56 × 0.05 s = 2.8 s |
| MPPI batch | 1000 |
| 线速度 | ±0.4 m/s |
| 角速度 | ±1.0 rad/s |
| 加速度 | 2.5 m/s² / 3.2 rad/s² |
| 足迹 | 0.69 × 0.57 m 矩形（±0.345 × ±0.285） |
| 目标容差 | XY 0.15 m，yaw 15° |
| 局部代价地图 | 6 × 6 m 滚动窗口，0.05 m |
| 全局代价地图 | 0.05 m，`track_unknown_space: true` |
| 膨胀半径 | 局部 0.4 m，全局 0.8 m |
| 进度检查 | 0.25 m / 15 s |
| 行为树 | `navigate_to_pose_w_replanning_and_recovery.xml`（默认） |
| 探索行为树 | `navigate_w_replanning_and_exploration_recovery.xml` |

> `nav2_params.yaml` 修改后需要重新 `./scripts/build.sh`，因为安装目录里是拷贝。
> `map_config.yaml` 是例外，launch 会优先读源码目录。

### 12.3 Super-LIO 桥接参数

`src/super_lio/super_lio_bridge/config/bridge.yaml`：

| 参数 | 默认 | 说明 |
|---|---|---|
| `lio_odom_topic` | `/lio/odom` | Super-LIO 输出 |
| `lio_cloud_topic` | `/lio/cloud_world` | Super-LIO 点云 |
| `imu_to_base` | `[0,0,-0.32,0,0,0]` | IMU 到 base 外参，**建议标定** |
| `odom_timeout` | 1.0 | 位姿超时判失效 |
| `publish_rate` | 50.0 | 桥接发布频率 |
| `stamp_offset` | 0.1 | TF 时间戳补偿 |
| `localization_mode` | `internal` | `internal` / `external` |

### 12.4 Super-LIO 上游参数中的坑

| 参数 | 必须注意 |
|---|---|
| `lio.sensor.filter_rate` | **整数**，写 `3.0` 会 SIGABRT |
| `lio.map.save_map_dir` | **后缀**，不是绝对路径 |
| `lio.map.save_map` | 建图必须 `true`，上游默认 `false` |
| `lio.map.map_name` | 建图/探索用 `map_explore.pcd`，避免覆盖 `map.pcd` |
| `lio.sensor.blind` | 0.4 m，避免贴墙时墙消失 |
| `lidar_type` | `1` = Livox CustomMsg；驱动用 `xfer_format=1` |

### 12.5 安全层参数

| 文件 | 关键值 |
|---|---|
| `collision_monitor.yaml` | `time_before_collision: 1.0`，`max_points: 5`，`/scan` |
| `twist_mux.yaml` | 255 急停 / 100 遥控 / 50 看门狗 / 20 恢复 / 10 自动 |
| `deadman.yaml` | 0.3 s 静默归零 |
| `recovery_guard.yaml` | 0.5 s 扫掠，近障等比例降速 |
| `estop_latch.yaml` | 锁存状态落盘 `/tmp/nav2_safety_estop.state` |
| `robot_status_monitor.yaml` | 卡住判定 12 s，电池阈值 20%/10% |

---

## 13. 话题、TF 与 Action 清单

### 13.1 传感器与 LIO

| 话题 | 类型 | 说明 |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | Super-LIO 使用 |
| `/livox/imu` | `sensor_msgs/Imu` | MID-360 内置 IMU |
| `/lio/odom` | `nav_msgs/Odometry` | Super-LIO 输出，frame=world |
| `/lio/cloud_world` | `sensor_msgs/PointCloud2` | Super-LIO 世界系点云 |
| `/odom` | `nav_msgs/Odometry` | 桥接层输出 |
| `/registered_scan` | `sensor_msgs/PointCloud2` | 桥接层转发 |
| `/scan` | `sensor_msgs/LaserScan` | 3D→2D 切片 |
| `/scan_reliable` | `sensor_msgs/LaserScan` | QoS 中继，Nav2 使用 |

### 13.2 导航与底盘

| 话题 / Action | 类型 | 说明 |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | map_server 或 grid_mapper |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | 全局代价地图 |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | 局部代价地图 |
| `/plan` | `nav_msgs/Path` | planner_server 实际发布 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 单目标 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 初始位姿 |
| `/navigate_to_pose` | action | Nav2 导航 |
| `/navigate_to_pose/_action/status` | `action_msgs/GoalStatusArray` | 航点状态 |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | 控制器输出 |
| `/cmd_vel_nav_smoothed` | `geometry_msgs/Twist` | 平滑后 |
| `/cmd_vel_nav_checked` | `geometry_msgs/Twist` | 碰撞监控后 |
| `/cmd_vel_muxed` | `geometry_msgs/Twist` | 仲裁后 |
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘最终输入 |
| `/cmd_vel_teleop` | `geometry_msgs/Twist` | 网页/GUI 遥控 |
| `/cmd_vel_recovery` | `geometry_msgs/Twist` | 恢复行为原始输出 |
| `/cmd_vel_recovery_checked` | `geometry_msgs/Twist` | 守卫后 |
| `/wheel_odom` | `nav_msgs/Odometry` | 轮式里程计 |
| `/bunker_status` | `bunker_msgs/BunkerStatus` | 底盘/BMS 状态 |

### 13.3 安全与状态

| 话题 | 类型 | 说明 |
|---|---|---|
| `/localization/tracking_ok` | `std_msgs/Bool` | 定位健康（含义随模式不同） |
| `/localization/tracking_status` | `std_msgs/String` | 桥接层状态 |
| `/safety/autonomy_blocked` | `std_msgs/Bool` | 看门狗锁 |
| `/safety/deadman_stopping` | `std_msgs/Bool` | 兜底零速 |
| `/safety/estop` | `std_msgs/Bool` | 急停写入 |
| `/safety/estop_active` | `std_msgs/Bool` | 急停锁存状态 |
| `/safety/estop_status` | `std_msgs/String` | 急停详情 |
| `/safety/watchdog_status` | `std_msgs/String` | 看门狗明细 |
| `/safety/recovery_blocked` | `std_msgs/Bool` | 恢复通道被拦 |
| `/robot/status` | `std_msgs/String` (JSON) | 聚合状态，2 Hz |
| `/robot/status_markers` | `visualization_msgs/MarkerArray` | RViz 文字 |

### 13.4 TF

| 变换 | 发布者 |
|---|---|
| `map -> odom` | Super-LIO 桥接（internal）或 small_gicp/KISS（external） |
| `odom -> base_footprint` | `super_lio_bridge`（Super-LIO 链路）或 `sensor_scan_generation`（FAST-LIO 链路） |
| `base_footprint -> chassis` | `robot_state_publisher` / URDF |
| `chassis -> livox_frame` | `robot_state_publisher` / URDF |
| `chassis -> cameras` | `robot_state_publisher` / URDF |

> 同一时间只能有一个节点发布 `map -> odom`，一个节点发布 `odom -> base_footprint`。

---

## 14. 标准操作流程

### 14.1 出车前检查

- [ ] 底盘已上电，硬件急停已松开；
- [ ] CAN 适配器插好，`candump can0` 有报文；
- [ ] MID-360 网线连接，`ping 192.168.1.170` 通；
- [ ] 遥控器在手边，SWB 模式正确；
- [ ] 导航栈/安全层已启动，`/cmd_vel` 只有一个发布者（deadman）；
- [ ] `/localization/tracking_ok` 为 true；
- [ ] `/map`、`/scan`、TF 正常；
- [ ] 网页安全面板显示“未锁 / 未触发”。

### 14.2 建图流程

1. `./scripts/chassis_start.sh`
2. `./scripts/super_lio_nav.sh mapping`
3. 遥控器驾驶覆盖环境；
4. Ctrl-C 退出，确认 `map_explore.pcd` 已生成；
5. 需要 2D 图时：`pcd_to_occupancy_map.py` 或 `save_map.sh`；
6. 在 Web 工作台上传 PGM/YAML 和 PCD，绑定到工作区；
7. 修改 `map_config.yaml` 或让 Web 后端写入；
8. 重启导航栈。

### 14.3 导航流程

1. `./scripts/chassis_start.sh`
2. `./scripts/super_lio_nav.sh relocation`（或 `tracking` / `tracking_kiss`）
3. 如需要，发 `/initialpose`；
4. `./scripts/web_start.sh`
5. 浏览器打开发送目标（先选点、调朝向、再下发）；
6. 观察 `/plan` 路径、代价地图、安全面板；
7. 结束时先取消任务，再停导航栈。

### 14.4 急停/恢复

- 软件急停：网页长按 1.2 s；解除同样长按；
- 定位跟丢：看 `/localization/tracking_ok` 和 `/safety/autonomy_blocked`；
  先取消任务，发 `/initialpose` 或重启重定位；
- 卡住：查看 `/robot/status` 的 `stuck`、`recovery_active`、
  `/safety/watchdog_status`；
- 脱困：`SmartRecovery` 会优先短退/缓弯，自由空间脱困会搜索最近可通行格；
- 任何时候硬件急停优先。

### 14.5 关机

1. 取消任务、停止探索；
2. Ctrl-C 停导航栈；
3. 停底盘驱动；
4. 关闭 rosbridge / Web 后端；
5. 断底盘电源。

---

## 15. 测试

### 15.1 Super-LIO 桥接测试

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest src/super_lio/super_lio_bridge/tests -q
```

覆盖：

- 位姿/外参组合不变式：`map->base` = Super-LIO 位姿 ⊗ `imu->base`；
- 超时后 `tracking_ok=false`；
- 欧拉角约定与 ROS/URDF 一致。

### 15.2 任务执行器测试

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/test_mission_executor.py
```

使用假 `navigate_to_pose` action server，覆盖成功、停留、暂停/继续、取消、失败、
重置、历史记录。脚本使用 `ROS_DOMAIN_ID=97`。

### 15.3 安全层测试

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest src/nav2_safety/test -q
```

### 15.4 grid_mapper 测试

```bash
python3 -m pytest src/grid_mapper/test -q
```

### 15.5 端到端检查命令

```bash
ros2 topic hz /lio/odom
ros2 topic hz /registered_scan
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic info /cmd_vel -v          # 应只有 cmd_vel_deadman 一个发布者
ros2 topic echo /robot/status --once
ros2 topic echo /safety/estop_active --once
```

---

## 16. 故障排查

### 16.1 定位 / LIO

| 现象 | 可能原因 | 处理 |
|---|---|---|
| `/lio/odom` 没有数据 | 雷达驱动未起、CustomMsg 格式不对、Super-LIO 未 source | 检查 `/livox/lidar`；确认 `xfer_format=1`；source 两个工作空间 |
| Super-LIO 启动即 abort | `lio.sensor.filter_rate` 写成 `3.0` | 改成整数 `3` |
| 重定位一直 `Load map failed` | `save_map_dir` 写成绝对路径，或 `map.pcd` 不存在 | 用后缀 `"map"`；确认 `$SUPERLIO_WS/src/super_lio/map/map.pcd` |
| 地图越跑越歪 | relocation 模式只初始化一次，没有持续修正 | 改用 `tracking` 或 `tracking_kiss` |
| 贴墙时墙消失 | `blind` 太大 | 用 0.4 m；检查 `Pointcloud2d_3d.yaml` 高度带 |
| TF 抖动/位姿跳变 | 同时运行两套 LIO/重定位 | 只保留一个 `map->odom` 发布者 |
| 找不到 `super_lio` 包 | 没 source Super-LIO 工作空间 | `source $SUPERLIO_WS/install/setup.bash` |

### 16.2 导航

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 车不动，Nav2 正常 | 安全看门狗锁定 | 看 `/safety/autonomy_blocked`、`/localization/tracking_ok` |
| 车不动，`/cmd_vel` 为 0 | 急停锁、碰撞监控、死区 | 看 `/safety/estop_active`、`/safety/watchdog_status` |
| 规划失败 | 目标在障碍内、未知区域 | 看 `/plan`、全局代价地图；Smac `allow_unknown: true` |
| 局部规划抖动 | 窄门/膨胀半径 | 调 `inflation_radius`、MPPI 参数 |
| 代价地图空白 | `/scan` QoS 不兼容、TF 断 | 检查 `/scan_reliable`、TF |
| 恢复行为绕过安全链 | `behavior_server` 直发 `/cmd_vel` | 确认使用 `navigation_safe_launch.py` 的重映射 |
| 底盘不动 | CAN、供电、急停、模式开关 | `candump can0`、`chassis_start.sh` |

### 16.3 Web / rosbridge

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 网页连不上 | rosbridge 未起、代理配置错误 | `pgrep -f rosbridge`；用 `127.0.0.1` 或正确 IP |
| 地图不显示 | `/map` 无数据、QoS 锁存 | `ros2 topic echo /map --once`；页面 3 秒后走 `load_map` 服务兜底 |
| 页面卡顿/rosbridge 内存暴涨 | 全量 TF/代价地图订阅 | 保持页面默认 `throttle_rate`；不要同时开多个页面 |
| 急停显示未触发但车不动 | 读错话题 | 页面应读 `/safety/estop_active`，不是 `/safety/estop` |
| 上传地图失败 | 缺少 `python-multipart` | `python3 -m pip install python-multipart` |
| 地图渲染空白 | 缺少 Pillow | `python3 -m pip install pillow` |
| 修改页面不生效 | 浏览器缓存 | Ctrl+Shift+R |

### 16.4 CAN / 底盘

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 找不到 `can0` | 适配器未插、`gs_usb` 未加载 | `sudo modprobe gs_usb` |
| `candump` 无报文 | 底盘未上电、急停、线缆 | 检查供电/急停/接线 |
| 两个底盘驱动 | 手动启动 + 脚本启动 | `./scripts/cleanup_ros_stack.sh`，只保留一个 |
| 波特率错误 | 不是 500 kbit/s | `CAN_BITRATE` 确认 |

### 16.5 残留进程

```bash
./scripts/cleanup_ros_stack.sh --dry-run   # 先看要杀谁
./scripts/cleanup_ros_stack.sh             # 清理本栈
./scripts/cleanup_ros_stack.sh --driver    # 连 Livox 驱动一起
```

启动脚本（`super_lio_nav.sh`、`super_lio_explore.sh`、`explore_record.sh`）
默认会自动调用它。

---

## 17. 已知限制与安全边界

- **Super-LIO 健康度是弱信号**：不暴露收敛状态/残差，桥接层只能判断位姿是否在更新。
  与 `small_gicp`/KISS 的 converged/error 判据相比，这是安全性上的一处退步。
- **relocation 模式只初始化一次**：中途跟丢不会自动恢复；需要持续修正请用
  `tracking`/`tracking_kiss`。
- **`imu_to_base` 外参未实机标定**：默认取 URDF 的 0.32 m，残差约几厘米；
  它直接影响 `map` 系位姿精度。
- **没有回环检测/全局优化**：LIO 漂移不会被闭环修正；长距离建图建议分段或结合
  先验地图。
- **动态障碍**：`grid_mapper` 的静态确认只能过滤短时行人足迹，不能替代语义识别；
  实时避障仍依赖 `/scan` + Nav2 + `collision_monitor`。
- **软件急停不是硬件急停**：不能替代底盘急停和安全回路。
- **Web 默认无鉴权、无 TLS**：只适合可信局域网。
- **工作区切换需要重启导航栈**：`map_config.yaml` 写入后不会热加载。
- **同一时间只能运行一套 LIO/重定位/底盘驱动**：重复实例会抢 TF、`/map`、`/cmd_vel`。
- **CPU/内存**：3D 点云和代价地图较重；rosbridge 多页面同时打开会明显增加负载。

---

## 18. 与 FAST-LIO2 的对比

完整对比见 [docs/FASTLIO2_COMPARISON.md](./docs/FASTLIO2_COMPARISON.md)。一句话总结：

- **FAST-LIO2** 是成熟、广泛部署的 LIO 前端：iterated EKF + ikd-Tree，直接
  scan-to-map，精度稳定、生态成熟；但它本身**不提供全局重定位**，需要外接
  small_gicp/KISS，且地图结构随环境增长。
- **Super-LIO** 的核心是 **OctVox 紧凑地图 + HKNN 对应搜索**，论文报告相对 SOTA
  每帧约快 73%、CPU 更低，并自带重定位模式；本仓库把它接进 Nav2 后，链路从
  “FAST-LIO + lio_interface + sensor_scan_generation + small_gicp” 缩短为
  “Super-LIO + bridge”。
- **本仓库的取舍**：Super-LIO 链路更短、更省资源、自带重定位；但它的健康度输出弱，
  且核心是 GPLv3 外部工作空间。需要成熟度和丰富诊断时，仍可切回 FAST-LIO2 链路。

| 维度 | FAST-LIO2 链路 | Super-LIO 链路（本仓库主用） |
|---|---|---|
| 核心地图 | ikd-Tree 增量 k-d 树 | OctVox 八叉体素紧凑地图 |
| 对应搜索 | ikd-Tree 最近邻 | HKNN 启发式 KNN |
| 重定位 | 外接 small_gicp / KISS | Super-LIO relocation，或外接 small_gicp/KISS |
| 输出 TF | `camera_init -> body` | `world -> imu` |
| Nav2 适配 | `lio_interface` + `sensor_scan_generation` | `super_lio_bridge` |
| 雷达格式 | PointCloud2（`xfer_format=0`） | CustomMsg（`xfer_format=1`） |
| 建图存档 | PCD 保存服务 | 分片 + 退出合并 |
| 健康度 | small_gicp 可给收敛/误差 | 仅位姿新鲜度（弱） |
| 资源 | CPU，地图随环境增长 | 论文报告更低 CPU、约快 73% |
| 许可 | GPLv2（源码随仓库） | 核心 GPLv3（外部工作空间）+ 桥接 MIT |
| 成熟度 | 高，社区广泛 | 新（RA-L 2026），集成较少 |
| 推荐场景 | 追求成熟/可诊断、已有调参经验 | 资源受限、需要自带重定位/更短链路 |

---

## 19. 许可证与致谢

本仓库自有代码（`super_lio_bridge`、`nav2_safety`、`grid_mapper`、`web`、
`me_nav2_bringup` 等）采用 [MIT License](./LICENSE)。

### Super-LIO 上游

本仓库的 Super-LIO 链路依赖以下上游项目与资料：

- **代码仓库（ROS 2 分支）**：<https://github.com/Liansheng-Wang/Super-LIO/tree/ros2>
- **论文**：*Super-LIO: A Robust and Efficient LiDAR-Inertial Odometry System with a
  Compact Mapping Strategy*，RA-L 2026
  - arXiv：<https://arxiv.org/abs/2509.05723>
  - DOI：<https://doi.org/10.1109/LRA.2026.3653372>
- **许可**：GPLv3。本仓库不包含 Super-LIO 核心代码，只通过 ROS 话题/TF 与它交互；
  桥接层 `super_lio_bridge` 为本仓库自有 MIT 代码。

如果你在研究中使用了本栈，请同时引用 Super-LIO 论文：

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

### 其他第三方组件

第三方组件保留各自许可证：

- [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO) — GPLv3，独立工作空间，不包含在本仓库；
- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) — GPLv2，回退链路源码在 `src/localization/FAST_LIO`；
- [Point-LIO](https://github.com/hku-mars/Point-LIO) — 回退链路源码在 `src/localization/point_lio`；
- [KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher) — MIT；
- [small_gicp](https://github.com/koide3/small_gicp) — MIT；
- [Nav2](https://github.com/ros-planning/navigation2) — Apache 2.0；
- [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) — BSD；
- [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2) — MIT；
- [ugv_sdk](https://github.com/agilexrobotics/ugv_sdk) — Apache 2.0；
- [Sophus](https://github.com/strasdat/Sophus) — MIT。

如果要把 Super-LIO 核心代码合入本仓库，请注意 GPLv3 与 MIT 的兼容性；
当前做法是独立工作空间 + ROS 话题/TF 交互，两者不混编。

---

## 20. 常用命令速查

```bash
# 编译
source /opt/ros/humble/setup.bash
./scripts/build.sh

# 编译 Super-LIO（独立工作空间）
source /opt/ros/humble/setup.bash
source install/setup.bash
cd "$SUPERLIO_WS" && colcon build --symlink-install

# 底盘
./scripts/chassis_start.sh

# Super-LIO
./scripts/super_lio_nav.sh mapping
./scripts/super_lio_nav.sh relocation
./scripts/super_lio_nav.sh tracking false
./scripts/super_lio_nav.sh tracking_kiss false
./scripts/super_lio_nav.sh exploration false

# 探索（带自检）
./scripts/super_lio_explore.sh rviz

# Web
./scripts/web_start.sh
./scripts/web_backend.sh --port 5500 --token '...'

# 旧 FAST-LIO 链路
./scripts/nav2_real.sh

# 清理
./scripts/cleanup_ros_stack.sh --dry-run
./scripts/cleanup_ros_stack.sh --driver

# 诊断
ros2 run tf2_ros tf2_echo map odom
ros2 topic hz /lio/odom
ros2 topic echo /robot/status --once
ros2 topic echo /safety/estop_active --once
ros2 topic info /cmd_vel -v

# 测试
python3 -m pytest src/super_lio/super_lio_bridge/tests -q
python3 scripts/test_mission_executor.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/nav2_safety/test -q
python3 -m pytest src/grid_mapper/test -q
```

---

如果这份手册和代码行为不一致，以代码和实际运行日志为准；欢迎在 issue 里附上
`/robot/status`、`ros2 topic info -v` 和相关节点日志。
