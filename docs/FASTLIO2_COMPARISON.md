# Super-LIO 链路 vs FAST-LIO2 链路

本文对比两套在本仓库中出现过的 LiDAR-惯性里程计（LIO）方案：

- **FAST-LIO2 链路**：`FAST-LIO2 -> lio_interface -> sensor_scan_generation ->
  small_gicp / KISS 重定位 -> Nav2`
- **Super-LIO 链路**（当前主用）：`Super-LIO -> super_lio_bridge ->
  small_gicp / KISS（可选） -> Nav2`

对比分三层：

1. **LIO 算法本身**：FAST-LIO2 vs Super-LIO；
2. **本仓库的集成方式**：TF、话题、重定位、启动脚本；
3. **实机取舍**：资源、健康度、成熟度、许可。

> 结论先行：Super-LIO 的优势是**更紧凑的地图结构、更低的计算/内存开销、自带重定位**
> 和更短的集成链路；FAST-LIO2 的优势是**成熟稳定、诊断信息更完整、社区经验更多**。
> 本仓库把 Super-LIO 作为主用方案，但保留 FAST-LIO2 回退链路，二者不能同时运行。

---

## 1. 两套方案是什么

### 1.1 FAST-LIO2

FAST-LIO2 是 HKU-MARS 的紧耦合 LiDAR-惯性里程计，核心特点：

- **迭代误差状态卡尔曼滤波（iterated EKF）** 融合 LiDAR 与 IMU；
- **ikd-Tree** 增量式 k-d 树维护局部地图，支持增量插入、删除和最近邻查询；
- 直接做 scan-to-map 配准，不提取特征；
- 支持多种 LiDAR（Livox、Velodyne、Ouster 等）；
- 输出高频里程计（常见 100 Hz 量级）；
- 本身**不提供全局重定位、回环检测或位姿图优化**；
- 地图随环境增长，长期大场景下内存/查询开销会上升。

在本仓库的旧链路里，FAST-LIO2 只负责里程计，`map -> odom` 由外接的
`small_gicp_relocalization` 或 `global_relocalization_kiss_matcher` 提供。

### 1.2 Super-LIO

Super-LIO 是 RA-L 2026 的 LIO 系统，论文标题为
*Super-LIO: A Robust and Efficient LiDAR-Inertial Odometry System with a Compact
Mapping Strategy*。核心特点：

- **OctVox 紧凑地图**：每个八叉体素只保留 8 个融合子体素，严格控制点密度，
  并在建图更新时做增量去噪；
- **HKNN（heuristic-guided KNN）**：利用空间局部性加速对应点搜索；
- 目标是**在资源受限平台（x86 / ARM）上保持精度同时降低 CPU/内存**；
- 论文报告相对 SOTA **每帧约快 73%**、CPU 占用更低；
- 上游提供 **relocalization 模式**：加载预建地图，从保存的地图中恢复定位，
  不需要重新建图；
- 支持 ROS1/ROS2、多种 LiDAR，官方声明可插拔。

Super-LIO 原生是**单坐标系** LIO：只发布 `world -> imu` 一个 TF，位姿在
`/lio/odom`（`frame_id=world`）。它没有“里程计 + 独立修正”两段式结构，
所以本仓库需要桥接层把它拆成 Nav2 需要的 `map -> odom -> base_footprint`。

---

## 2. 本仓库的集成差异

### 2.1 数据流

**FAST-LIO2 链路：**

```
Livox (PointCloud2, xfer_format=0)
  -> FAST-LIO2
  -> /Odometry (camera_init)
  -> lio_interface
  -> /odom + odom->base_footprint (sensor_scan_generation)
  -> /registered_scan
  -> pointcloud_to_laserscan -> /scan
  -> small_gicp / KISS -> map->odom
  -> Nav2
```

**Super-LIO 链路：**

```
Livox (CustomMsg, xfer_format=1)
  -> Super-LIO
  -> /lio/odom + /lio/cloud_world (world)
  -> super_lio_bridge
  -> /odom + odom->base_footprint
  -> /registered_scan
  -> pointcloud_to_laserscan -> /scan
  -> [internal] map->odom 由桥接层锚定
     [external] map->odom 由 small_gicp / KISS 发布
  -> Nav2
```

### 2.2 组件替换关系

| FAST-LIO2 链路 | Super-LIO 链路 |
|---|---|
| FAST-LIO2 | Super-LIO |
| `lio_interface` | `super_lio_bridge`（合并） |
| `sensor_scan_generation` | `super_lio_bridge`（合并） |
| `small_gicp_relocalization` | Super-LIO relocation，或 external 模式继续用 small_gicp/KISS |
| Livox `xfer_format=0` | Livox `xfer_format=1` |

Super-LIO 链路把 4 个组件压缩成 2 个（LIO + bridge），启动和排障面更短。

### 2.3 启动方式

| 任务 | FAST-LIO2 链路 | Super-LIO 链路 |
|---|---|---|
| 建图 | `./scripts/mapping_real.sh` | `./scripts/super_lio_nav.sh mapping` |
| 导航 | `./scripts/nav2_real.sh` | `./scripts/super_lio_nav.sh relocation/tracking/tracking_kiss` |
| 探索 | 手动组合 slam_toolbox + Nav2 | `./scripts/super_lio_explore.sh` |
| 底盘 | `./scripts/chassis_start.sh` | `./scripts/chassis_start.sh` |

注意：

- `nav2_real.sh` 会启动 FAST-LIO、small_gicp 和 Nav2；
- `super_lio_nav.sh` 会启动 Super-LIO、桥接、2D 切片、Nav2 和安全层；
- **两者不能同时运行**，会抢 `/cmd_vel` 和 `map -> odom`。

### 2.4 重定位策略

| 场景 | FAST-LIO2 链路 | Super-LIO 链路 |
|---|---|---|
| 起点大致已知 | `small_gicp_relocalization` | `relocation`（Super-LIO 自带）或 `tracking`（small_gicp） |
| 起点未知 | `global_relocalization_kiss_matcher`（KISS + GICP） | `tracking_kiss`（KISS + GICP + 自动恢复） |
| 运行中持续纠偏 | small_gicp 约 2 Hz | `tracking`（small_gicp）或 `tracking_kiss` |
| 跟丢后自动恢复 | 取决于 KISS 实现 | `tracking_kiss` 在 GICP 连续失败后切回 KISS |

Super-LIO 自带的 `relocation` 是**一次性初始化**：加载先验地图后给出绝对位姿，
之后不再用外部观测持续修正。如果中途跟丢，需要重新启动或发 `/initialpose`。

### 2.5 TF 与话题

| 项 | FAST-LIO2 链路 | Super-LIO 链路 |
|---|---|---|
| LIO 里程计 | `/Odometry`（`camera_init`） | `/lio/odom`（`world`） |
| TF 原始输出 | `camera_init -> body` | `world -> imu` |
| Nav2 适配 | `lio_interface` + `sensor_scan_generation` | `super_lio_bridge` |
| 点云 | `/cloud_registered` | `/lio/cloud_world` -> `/registered_scan` |
| 2D 激光 | `/scan` | `/scan` |
| 定位健康 | `/localization/tracking_ok`（small_gicp/KISS） | `/localization/tracking_ok`（桥接层，弱信号） |

两套最终都给 Nav2 同一条 TF 链：`map -> odom -> base_footprint`。

### 2.6 地图与建图

| 项 | FAST-LIO2 | Super-LIO |
|---|---|---|
| 地图结构 | ikd-Tree | OctVox（8 子体素/体素） |
| 点云保存 | PCD 服务/手动保存 | 分片保存 + 退出合并 |
| 保存位置 | 可配置 | `$SUPERLIO_WS/src/super_lio/map/` |
| 地图命名 | 手动 | `map_explore.pcd` / `map.pcd` |
| 默认是否保存 | 需主动触发 | 上游默认 `false`，本仓库改为 `true` |
| 大场景内存 | 随地图增长 | 严格体素密度，内存更可控 |

Super-LIO 的 `save_map_dir` 是**后缀**，不是绝对路径；写成 `/tmp/foo` 会被拼成
`<源码目录>//tmp/foo`。这是接入时最容易踩的坑之一。

### 2.7 传感器格式

| 项 | FAST-LIO2 链路 | Super-LIO 链路 |
|---|---|---|
| Livox 驱动 launch | `fast_lio_msg_MID360_launch.py`（`xfer_format=0`） | `msg_MID360_launch.py`（`xfer_format=1`） |
| 消息类型 | `sensor_msgs/PointCloud2` | `livox_ros_driver2/CustomMsg` |
| 配置 | `mid360.yaml` | `super_lio_mid360_mapping.yaml` / `relocation.yaml` |

驱动和 LIO 的消息格式必须匹配；用错 launch 会出现“话题有数据但 LIO 不动”。

### 2.8 健康度与安全

这是两套方案在**实机安全**上最重要的差异。

- **FAST-LIO2 链路**：`small_gicp_relocalization` 和
  `global_relocalization_kiss_matcher` 能提供收敛状态、配准误差等判据，
  `localization_watchdog` 可以据此锁住自动驾驶；
- **Super-LIO 链路**：上游**没有对外发布** `flg_init_`、`flg_get_init_guess_`
  或配准残差；桥接层只能根据 `/lio/odom` 是否持续更新给出“位姿新鲜度”级别的
  `tracking_ok`。它能发现“位姿不再更新”，但**发现不了“重定位收敛到错误位姿”**。

因此：

- 对定位可信度要求高、需要诊断信息的场景，FAST-LIO2 + small_gicp/KISS 更稳；
- 使用 Super-LIO relocation 时，必须保留人工确认（RViz/网页初始位姿、现场观察）；
- 这也是本仓库把 Fast-LIO2 链路保留为回退方案的原因之一。

### 2.9 性能与资源

| 项 | FAST-LIO2 | Super-LIO |
|---|---|---|
| 计算平台 | CPU | CPU，x86 / ARM |
| 地图查询 | ikd-Tree 最近邻 | HKNN |
| 地图内存 | 随环境增长 | OctVox 限制点密度 |
| 论文/README 声称 | 高频、成熟 | 每帧约快 73%，CPU 更低 |
| 本仓库实测 | 未在本文重新基准测试 | 104 m 轨迹对比：RMSE 13 mm / 最大 28 mm；全局 ICP fitness 0.0096 |

> 13 mm / 28 mm 来自 `src/super_lio/README.md` 中记录的回放验证，
> 不是本手册重新做的基准测试；测试数据和场景不同，数字不能直接横向比较。

### 2.10 许可与工程形态

| 项 | FAST-LIO2 | Super-LIO |
|---|---|---|
| 许可证 | GPLv2 | GPLv3 |
| 是否在本仓库内 | 是（`src/localization/FAST_LIO`） | 否（独立工作空间） |
| 与 MIT 代码的关系 | 源码同仓库，注意 GPL 约束 | 通过 ROS 话题/TF 交互，桥接层 MIT |
| 上游更新 | 手动同步 | 独立工作空间更新，不影响本仓库 |

---

## 3. 逐项对比表

| 维度 | FAST-LIO2 链路 | Super-LIO 链路 |
|---|---|---|
| LIO 核心 | iterated EKF + ikd-Tree | LIO + OctVox + HKNN |
| 地图结构 | 增量 k-d 树 | 八叉体素（8 子体素/体素） |
| 对应搜索 | ikd-Tree 最近邻 | 启发式 KNN |
| 回环检测 | 无 | 无（LIO 本身） |
| 全局重定位 | 外接 small_gicp/KISS | 自带 relocation，或外接 |
| 持续纠偏 | small_gicp 约 2 Hz | tracking 模式 |
| 跟丢自动恢复 | 取决于外接实现 | tracking_kiss |
| 雷达格式 | PointCloud2 | CustomMsg |
| 原始 TF | `camera_init -> body` | `world -> imu` |
| Nav2 适配 | 两个中间包 | 一个桥接包 |
| 建图保存 | PCD 服务 | 分片 + 退出合并 |
| 健康度 | 有收敛/误差判据 | 仅新鲜度（弱） |
| 计算/内存 | 成熟稳定，地图增长 | 更紧凑，论文报告更快/更省 |
| 成熟度 | 高，社区大 | 新（RA-L 2026），集成少 |
| 许可证 | GPLv2 | GPLv3 |
| 本仓库角色 | 回退链路 | 主用链路 |

---

## 4. 选型建议

### 4.1 优先选 Super-LIO

- 计算资源受限（ARM、小内存）；
- 希望缩短链路，减少 `lio_interface` / `sensor_scan_generation` 的排障面；
- 需要 Super-LIO 自带的重定位；
- 能接受“健康度较弱”，并保留人工确认流程；
- 愿意维护一个独立的 GPLv3 工作空间。

### 4.2 优先选 FAST-LIO2

- 已经在 FAST-LIO2 上调过参、有现场经验；
- 需要收敛/配准误差等诊断信息，安全看门狗要据此锁车；
- 希望所有源码在一个仓库里，便于审计和修改；
- 场景/雷达型号在 FAST-LIO2 社区有成熟配置；
- 对最新算法的性能收益不敏感，更看重稳定。

### 4.3 本仓库的默认选择

- **默认主用 Super-LIO**：`./scripts/super_lio_nav.sh ...`
- **回退链路**：`./scripts/nav2_real.sh`
- **两者不要同时运行**。

---

## 5. 迁移与回退

### 5.1 FAST-LIO2 -> Super-LIO

1. 编译 Super-LIO 独立工作空间；
2. 用 `./scripts/super_lio_nav.sh mapping` 建图，得到 `map_explore.pcd`；
3. 把地图改名为 `map.pcd` 放回 Super-LIO 的 `map/` 目录，或配置
   `map_config.yaml` 的 `prior_pcd_file` 给 tracking/tracking_kiss；
4. 选择 `relocation` / `tracking` / `tracking_kiss`；
5. 检查 `/lio/odom`、`map -> odom`、`/scan_reliable`、`/localization/tracking_ok`；
6. 确认 `localization_watchdog` 没有因为 `tracking_ok` 缺失而锁车。

### 5.2 Super-LIO -> FAST-LIO2

1. 停止 Super-LIO 链路；
2. `./scripts/nav2_real.sh`；
3. 确认 `map_config.yaml` 的 `prior_pcd_file` 指向旧链路可用的 PCD；
4. 确认 `small_gicp_relocalization` 或 KISS 节点在发布 `map -> odom`；
5. 确认 `lio_interface` / `sensor_scan_generation` 正常。

### 5.3 切换时的检查清单

- [ ] 只有一个 `map -> odom` 发布者；
- [ ] 只有一个 `odom -> base_footprint` 发布者；
- [ ] `/cmd_vel` 只有一个发布者（通常是 `cmd_vel_deadman`）；
- [ ] `/scan` 有数据，`/scan_reliable` 有数据；
- [ ] `/localization/tracking_ok` 为 true；
- [ ] 安全层未锁自动驾驶（`/safety/autonomy_blocked` 为 false）；
- [ ] 急停状态正常（`/safety/estop_active` 为 false）；
- [ ] 遥控话题使用 `/cmd_vel_teleop`。

---

## 6. 常见误区

1. **“Super-LIO 一定更准”**：论文强调的是效率/紧凑地图，精度是“competitive”；
   实际精度取决于外参、标定、场景和重定位质量。
2. **“Super-LIO 自带重定位，所以不用管初始位姿”**：relocation 仍需要先验地图和
   合理的初始猜测；它只初始化一次，跟丢不会自动恢复。
3. **“两个 LIO 可以同时跑，互为备份”**：不行。它们会同时发布 TF 和消耗
   `/cmd_vel`，导致位姿跳变和控制混乱。
4. **“健康度接口一样”**：不一样。Super-LIO 的 `tracking_ok` 只是新鲜度，
   FAST-LIO 链路的 small_gicp/KISS 能给出收敛/误差判据。
5. **“换地图只改 `map_config.yaml` 就行”**：Web 工作区切换会写该文件，
   但需要重启导航栈才会加载；Super-LIO relocation 用的是它自己的 `map.pcd`，
   不是 `map_config.yaml`。

---

## 7. 参考

- Super-LIO 论文：<https://arxiv.org/abs/2509.05723>
- Super-LIO DOI：<https://doi.org/10.1109/LRA.2026.3653372>
- Super-LIO 上游（ROS 2 分支）：<https://github.com/Liansheng-Wang/Super-LIO/tree/ros2>
- FAST-LIO2：<https://github.com/hku-mars/FAST_LIO>
- Point-LIO：<https://github.com/hku-mars/Point-LIO>
- 本仓库 Super-LIO 接入说明：[`src/super_lio/README.md`](../src/super_lio/README.md)
- 本仓库安全层说明：[`src/nav2_safety/README.md`](../src/nav2_safety/README.md)
- 本仓库 Web 文档：[`src/web/README.md`](../src/web/README.md)
