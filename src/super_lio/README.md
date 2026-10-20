# Super-LIO 接入（替代 FAST-LIO 链路）

把本工程的里程计与重定位换成 [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO)
（RA-L 2026），并使用它**自带的**重定位模式。原 FAST-LIO 那一套保留不动，随时可切回。

## 这个文件夹里有什么

```
src/super_lio/
└── super_lio_bridge/          ← 桥接包（本工程自己的代码，MIT）
    ├── super_lio_bridge/
    │   ├── tf_bridge.py        world->imu 拆成 map->odom + odom->base_footprint
    │   └── transforms.py       SE(3) 工具（可脱离 ROS 单测）
    ├── config/
    │   ├── bridge.yaml                        桥接参数
    │   ├── super_lio_mid360_mapping.yaml      Super-LIO 建图配置（适配后）
    │   └── super_lio_mid360_relocation.yaml   Super-LIO 重定位配置（适配后）
    ├── launch/                  桥接单独启动 / 整套 bringup
    └── tests/                   数学单测 + 桥接集成测试
```

**Super-LIO 上游代码不在本仓库内**，它在你自己的独立工作空间
`/home/demo/superlio_ws/Super-LIO-ros2/`。这样做有两个好处：一是它的 GPLv3 与本仓库
的 MIT 不混在一起（本仓库不包含它的任何代码）；二是上游更新不用改动本仓库。

## 为什么必须有个桥接层

Super-LIO 是**单坐标系**的 LIO：只广播一个 TF `world -> imu`，位姿发在 `/lio/odom`
（frame_id = `world`）。重定位模式下 `world` 帧**就是先验地图的帧**，也就是它直接给
绝对定位，没有"里程计 + 独立修正"两段。

而 Nav2 要的是三段式 `map -> odom -> base_footprint`（局部代价地图以 `odom` 为参考做
滚动窗口）。所以 `tf_bridge` 做的映射是：

```
odom -> base_footprint = 相对启动位姿的运动（连续、无跳变）
map  -> odom           = 启动那一刻在 map 中的位姿（常量）
```

于是 `map` 系下的位姿 = Super-LIO 的绝对定位结果，而 `odom` 系保持连续。
注意：单坐标系 LIO 的漂移会直接体现在 map 系位姿上，**没有任何独立机制去纠正**，
这是它的固有特性。

桥接层**只通过 ROS 话题/TF 与 Super-LIO 交互**，不 include 它的头文件、不链接它的库。
因此本包编译时不需要 Super-LIO 存在，许可上也彼此独立。

## 先编译 Super-LIO

它是独立工作空间，且 `package.xml` 依赖 **`livox_ros_driver2`**，所以要先 source
本工作空间（提供该驱动包），再编译它：

```bash
source /opt/ros/humble/setup.bash
source /home/demo/Lidar_nav2_ws-main/Lidar_nav2_ws/install/setup.bash   # 提供 livox_ros_driver2
cd /home/demo/superlio_ws/Super-LIO-ros2
colcon build --symlink-install
```

依赖：`libgoogle-glog-dev`、`libtbb-dev`（本机已装）。

桥接包的依赖：`numpy` + `scipy`（本机都已装）。旋转部分用
`scipy.spatial.transform.Rotation` 而不是自己手写 —— 四元数/欧拉角互转的边界情况
（万向锁、近 180°、符号约定）自己写最容易出错，而且 scipy 本来就在环境里。
欧拉角约定必须与 ROS/URDF 一致（`R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`），
`tests/test_transforms.py` 里有一条独立于库的手工断言把这条钉死。

## 怎么跑

```bash
# 1. 先建图（这是必须的第一步，重定位依赖它存下来的地图）
cd /home/demo/Lidar_nav2_ws-main/Lidar_nav2_ws
./scripts/chassis_start.sh &          # 底盘（CAN + BUNKER 驱动）
./scripts/super_lio_nav.sh mapping

# 用遥控器（遥控器 SWB 拨到"遥控模式"）把环境跑一圈，然后 Ctrl-C 退出。
# 退出时 Super-LIO 会把分片点云合并保存到：
#   /home/demo/superlio_ws/Super-LIO-ros2/src/super_lio/map/map.pcd

# 2. 重定位 + 导航
./scripts/super_lio_nav.sh relocation
# 可选带 RViz：./scripts/super_lio_nav.sh relocation true
```

脚本会自动按正确顺序 source 两个工作空间。若车不在先验地图原点附近，启动后用 RViz
或网页的**"初始位姿"**发 `/initialpose`（Super-LIO 有订阅这个话题）。

**底盘与 FAST-LIO 那套不能同时跑**：`./scripts/nav2_real.sh` 会同时起 FAST-LIO 和
`small_gicp` 重定位，与 Super-LIO 抢 `/cmd_vel` 和 `map -> odom`。用 Super-LIO 时
只起 `./scripts/chassis_start.sh`。

## 与安全层的衔接

安全层的看门狗默认盯 `/localization/tracking_ok`，所以桥接层必须发它，否则自动驾驶
会被永久锁住。但这里要清楚：**Super-LIO 的健康度是弱信号。**

Super-LIO 内部有 `flg_init_` / `flg_get_init_guess_` 等状态，也做了地图配准，但
**一个都没对外发布**，也没有配准残差输出。所以桥接层只能给出"位姿数据是否在正常更新"
这一级别的判断，**发现不了"重定位收敛到了错误位姿"**。相比之下 `small_gicp_relocalization`
和 `global_relocalization_kiss_matcher` 能给到 converged/error 级别的判据。
这是换用 Super-LIO 后在安全性上的一处**退步**，不是桥接层的实现缺陷。

## 踩过的坑（都写进配置注释了）

1. **`lio.sensor.filter_rate` 必须是整数。** 节点声明为 `<int>`，写成 `3.0` 会在启动时
   抛 `InvalidParameterTypeException` 并直接 **abort（SIGABRT）**。
   ⚠ 上游的 `config/relocation.yaml` 写的正是 `3.0` —— 那份配置开箱即崩。
   这大概也解释了为什么上游 `launch/relocation.py` 反而去加载建图那份配置。
2. **`lio.map.save_map_dir` 是后缀，不是绝对路径。** 源码是
   `g_save_map_dir = g_root_dir + save_map_dir`，而 `g_root_dir` 是编译期宏 `ROOT`
   （包源码目录）。写绝对路径会被拼成 `<源码目录>//tmp/xxx` 然后读写失败。
   所以地图实际落在 `<源码目录>/map/map.pcd`。
3. **上游建图配置默认 `save_map: false`**，也就是**不会存地图**；不打开它，重定位
   永远没有地图可用。已在本工程的配置里改成 `true`。
4. **雷达格式**：Super-LIO 在 `lidar_type: 1` 下订阅 `CustomMsg`，所以驱动必须用
   `xfer_format=1` 的 `msg_MID360_launch.py`（不是 FAST-LIO 用的那份 `xfer_format=0`）。
5. **`lio.hash_map.insert_resolution`** 在上游 relocation.yaml 里有，但节点没有声明
   这个参数，设了不生效，已从本工程配置里去掉。

## 已验证 / 未验证

**已验证（本机实测）**

- Super-LIO 上游编译通过（2 个包，`super_lio_node` + `relocation_node`）
- `relocation_node` 能加载先验地图并持续运行（用合成地图验证）
- 桥接层 9 项集成测试通过，含**组合不变式**：
  `map->base_footprint` 精确等于 `Super-LIO 位姿 ⊗ imu->base 外参`
- 位姿超时后 `/localization/tracking_ok` 正确变 false（安全层据此锁住自动驾驶）
- 整套 bringup 的节点接线（livox 驱动 / robot_state_publisher / 桥接 / Nav2）能起来
- **[2026-10-16 用真实 MID-360 录制数据包回放补齐]** 建图存图 → 用它重定位：
  全局 ICP 收敛 fitness 0.0096；与建图轨迹逐点对比，104 m 路径上位置差
  RMSE 13 mm / 最大 28 mm。探索链路（grid_mapper + explore_lite）也跑通到
  `/cmd_vel`（详见 `/home/demo/superlio_ws/known_issues.md` 与
  `scripts/super_lio_explore.sh`）

**未验证（需要实机和真实雷达）**

- ~~真实 MID-360 数据下的定位精度与重定位成功率~~ → 见上方 (2026-10-16 已用回放数据补齐)
- `imu_to_base` 外参的准确性（默认取 URDF 的 base_footprint→livox_frame = 0.32m，
  残差约几厘米，来自 MID-360 内部 IMU 与雷达的偏置；**建议实机标定后改**
  `super_lio_bridge/config/bridge.yaml`）
- 与 MPPI / 安全层的整体表现

> **排查问题先看 `/home/demo/superlio_ws/known_issues.md`**（在 Super-LIO 那个
> 工作区的顶层，不在本仓库里）：里面按"已修复 / 未定论 / 已排除的误判"整理了端到端
> 验证时遇到的问题，含现象原文、复现方式、当前判断与置信度，以及给下一步排查的建议。

## 切回 FAST-LIO

原链路完全没动。用回 `./scripts/nav2_real.sh` 即可（它会起 FAST-LIO +
`lio_interface` + `sensor_scan_generation` + `small_gicp` 重定位 + Nav2 + 安全层）。
两者不要同时跑。
