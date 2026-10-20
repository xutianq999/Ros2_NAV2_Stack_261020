# nav2_safety

BUNKER MINI 2.0 导航栈的**安全层**。补的是原系统里两个真缺口，外加两个可选增强：

1. `/cmd_vel` 有多个发布者（Nav2、网页遥控、`gui_teleop`），**没有任何优先级仲裁**。
   两路同时发时指令交错；而且正因为一直有指令在流，底盘自带的 500ms 通讯保护不会
   介入，底盘会跟着一个混乱的混合指令走。→ `twist_mux`
2. 避障完全依赖规划器内部（现在是 MPPI 的 ObstaclesCritic）的判断，**没有独立于规划器的第二道障碍闸**。
   指令一直在流，固件保护从不介入。→ `collision_monitor`

另外两个是可选增强，**重要性低于我最初的说法**（原因见下一节，是我先前的判断错误）：

3. `localization_watchdog`：定位丢失时停得更快、更明确，并阻止 Nav2 拿着不可信位姿
   继续规划。
4. `cmd_vel_deadman`：底盘本身就有 500ms 通讯保护，所以它只把停车从 500ms 缩到
   300ms，价值有限。

本包的节点、配置、启动文件全部是新增的。启用时需要改动的现有文件只有三处：
遥控话题两行 + 导航启动文件一行，见 [docs/ENABLING.md](docs/ENABLING.md)。

## 话题链

```
Nav2 controller_server ──> cmd_vel_nav
velocity_smoother      ──> cmd_vel_nav_smoothed
collision_monitor      ──> cmd_vel_nav_checked   障碍停车/减速（独立于规划器）
twist_mux              ──> cmd_vel_muxed         优先级仲裁 + 锁
cmd_vel_deadman        ──> /cmd_vel              静默即归零
                            └─> bunker_base（无需改动）
```

单独用 `ros2 launch nav2_safety safety_layer_launch.py` 可以只起安全层，
配合自己的导航启动；`safe_nav_bringup_launch.py` 则是 `my_nav2_launch.py` 的
整体替代品。

## 为什么是这几个组件（设计决策都来自实测或读码，且明确区分"已核实"与"未核实"）

### twist_mux 一个人不够：它在静默时不发零速

实测（把 twist_mux 4.3.0 单独跑起来、按阶段喂输入并观测输出）：

| 场景 | twist_mux 输出 |
|---|---|
| 输入正常 | 0.250，50Hz，正常透传 |
| 输入停止发布（超时） | **整段时间 0 条消息，完全不发** |
| 所有输入被 255 优先级的锁挡住 | **除切换瞬间 1 条残留外，完全不发** |

也就是说 twist_mux 的语义是"没有有效指令就不说话"，而不是"没有有效指令就下令停车"。
（上游 `twist_mux_locks.yaml` 里那句 "timeout ... use 0.0 speed instead" 容易被
误读成后者。）

### 底盘其实会自己停 —— 手册确认，并推翻了我最初的判断

`src/ugv_sdk/docs/protocol_v1/BUNKER用户手册.pdf` 的 Q&A 里厂家原话：

> BUNKER 的内部有通讯保护机制，底盘在处理来自外部的 CAN 控制指令时存在超时保护
> 机制，假设小车收到一帧通讯协议以后，**小车超过 500MS 未收到下一帧控制指令，
> 小车会进入通讯保护，速度为 0**，所以来自上位机的指令必须是周期性发布的。

也就是说 `/cmd_vel` 断掉时，底盘会在 500ms 内自己停车。**这一点推翻了我最初的说法**
（我曾断言底盘会保留最后一条运动指令继续行驶——那是错的，我只核了 ROS 节点没有
超时保护，没有去查固件，却把它当成了结论）。

于是各组件的重要性必须重排：

| 组件 | 面对的真实缺口 | 重要性 |
|---|---|---|
| `twist_mux` 仲裁 | 遥控和 Nav2 同时发 `/cmd_vel` 导致指令交错。注意两路都在发，所以**不会**触发那个 500ms 保护，底盘跟着一个混乱的混合指令走 | **高**（真缺口） |
| `collision_monitor` | 避障只有规划器一个判断来源。指令一直在流，固件保护从不介入 | **高**（真缺口） |
| `localization_watchdog` | 定位丢失时 Nav2 会因 TF 查询失败而停发，底盘 500ms 后停。所以这不是"会不会停"的缺口，而是**停得更快、更明确**，并且阻止 Nav2 拿着不可信位姿继续规划 | 中 |
| `cmd_vel_deadman` | 上面的 500ms 保护已经覆盖了"发布者挂掉"。它只把 500ms 缩短到 300ms（0.26 m/s 下约 5cm），并让停车不依赖固件配置 | 低（可选） |

⚠ 500ms 这个数值出自 **BUNKER（protocol v1）** 手册；Bunker Mini 2.0 用哪个协议
版本、超时值是否相同，仓库里没有对应文档。**建议上台架实测确认**：低速下发指令后
停止发布，测实际停车延迟。

### 遥控与 CAN 的模式关系（建图时会不会互相干扰）

同一手册 2.2 节：底盘的控制模式由**遥控器上的 SWB 硬件开关**选择 ——
最上 = 指令控制模式，中间 = 遥控控制模式，最下（配导航系统时）= 导航控制模式。

所以建图时把 SWB 拨到遥控模式，底盘就由遥控器控制，ROS 侧的 CAN 指令不参与。
这意味着**在建图流程里同时运行安全层不会干扰遥控驾驶**（此结论依据手册的模式
定义，未做实机验证）。

### collision_monitor 是生命周期节点

不配 `lifecycle_manager` 的话它会一直停在 unconfigured：参数照收、日志不报错、
但**完全不工作**。所以 `safety_layer_launch.py` 里必须同时起 lifecycle_manager。

## 组件

| 组件 | 来源 | 职责 | 关键参数 |
|---|---|---|---|
| `collision_monitor` | Nav2 官方 | 按真实车身预测未来 1s 的扫掠碰撞（方向相关），独立于规划器 | `config/collision_monitor.yaml` |
| `twist_mux` | 官方 | 多发布者优先级仲裁 + 锁 | `config/twist_mux.yaml` |
| `cmd_vel_deadman` | 本包 | 上游静默 → 持续零速 | `config/deadman.yaml` |
| `localization_watchdog` | 本包 | TF/点云/跟踪健康度 → 锁住自动驾驶 | `config/localization_watchdog.yaml` |
| `estop_latch` | 本包 | 软件急停锁存成可持久读取的状态 | `config/estop_latch.yaml` |

优先级分层：**255 急停锁 > 100 遥控 > 50 看门狗锁 > 10 自动驾驶**。
看门狗锁特意放在遥控和自动驾驶之间：定位丢失时只掐自动驾驶，遥控保留——
现场最需要的是操作员能把车开回可重定位的位置，把遥控一起锁死等于没有恢复手段。

## 已验证的行为

`twist_mux + deadman` 端到端（真实进程 + 真实话题）：

| 场景 | 期望 | 结果 |
|---|---|---|
| 仅 Nav2 在发 | 0.25 | ✅ |
| **Nav2 停止发布** | **0.00（靠 deadman 补）** | ✅ |
| Nav2 + 遥控 | 0.10（遥控优先） | ✅ |
| 锁自动驾驶 + 遥控仍在 | 0.10（遥控不受影响） | ✅ |
| 锁自动驾驶 + 无遥控 | 0.00 | ✅ |
| 急停锁 | 0.00 | ✅ |

> 第 2 行要注意：这一格验的是**链路行为**（静默时确实会下发零速）。但仅就
> "会不会停"而言，底盘那 500ms 通讯保护本来就能兜住；这一项体现的是兜底级的
> 确定性，不是它不可替代。真正不可替代的是第 3～6 行——仲裁和锁，底盘固件
> 完全没有这个能力。

`localization_watchdog`：无 TF→锁住、TF 正常→放行、`map->odom` 断掉→重新锁住、
恢复→放行，4/4 通过。

`collision_monitor`：`FootprintApproach` 区域创建成功，在 lifecycle_manager 下走到 active。
（早期版本的 StopZone/SlowZone 因方向无关，已删除：贴墙时会把正常行驶和倒车一起零速覆盖。）

`safety_layer_launch.py` 整层冒烟（`twist_mux` 用系统安装的 4.3.0，真实
`ros2 launch`）：

| 检查项 | 结果 |
|---|---|
| 5 个节点（4 个工作节点 + 生命周期管理器）全部起来 | ✅ |
| `collision_monitor` 激活 | ✅ |
| 看门狗在 TF 齐备后放行自动驾驶 | ✅ |
| `/cmd_vel` 上恰好只有 `cmd_vel_deadman` 一个发布者 | ✅ |

网页安全面板与急停（真实浏览器 + rosbridge + 安全层一起跑）：

| 检查项 | 结果 |
|---|---|
| 面板显示真实状态（已锁住 / 正在停车 / 未知 / 未触发 + 明细） | ✅ |
| 点"🛑 急停"（确认框）→ 面板变"已锁存"，`/safety/estop_status` = `estop_latched=True` | ✅ |
| 急停锁存期间发遥控 0.20 → `/cmd_vel` 中位 = **0.000**（被压制） | ✅ |
| 点"解除急停" → 同样遥控 0.20 → `/cmd_vel` 中位 = **0.200**（恢复放行） | ✅ |
| 置急停后**后启动**仲裁层，仍受急停约束（`/cmd_vel` = 0.000） | ✅ |

看门狗消费重定位健康度：

| 场景 | 结果 |
|---|---|
| TF 正常 + `tracking_ok=true` | 放行 ✅ |
| `tracking_ok=false`，而 `map->odom` 仍然新鲜（0.02s） | **锁住自动驾驶** ✅ |
| 跟踪恢复 | 重新放行 ✅ |

第二行是新增检查项存在的理由：GICP 跟丢时不更新位姿，但 TF 还在按老位姿发布，
靠 TF 新鲜度完全发现不了。

`small_gicp_relocalization` 的健康度发布也实跑验证过（合成 PCD + 合成点云）：
确认发布 `tracking_ok=True`、`tracking_error≈0.238`。

## 操作员界面

网页控制台（`src/web/navigation.html`）侧栏有一块"安全状态"面板和急停按钮：

- 面板显示**自动驾驶（放行/已锁住）、停止指令、重定位跟踪、急停**四项状态，
  外加看门狗判定的明细（各项检查的年龄）。目的是让"车为什么不动"当场可见——
  否则现场只能看到车不动，第一反应会去重启设备，而真实原因可能是定位跟丢。
- "🛑 急停"按钮（带确认框）和"解除急停"按钮，操作 `/safety/estop`。

### 三个踩过的坑（都让安全功能静默失效）

1. **急停不能只发一条消息**。原先网页直接往 `/safety/estop` 发一次 true 就完事，
   但 rosbridge 的发布是易失的（volatile），没有锁存语义：**仲裁层一重启，锁就没了**，
   机器人可以动，而界面上可能还显示"已锁存"。所以加了 `estop_latch` 节点，把锁存
   状态变成持续发布、`transient_local` 的 `/safety/estop_active`，锁和界面都读它。
2. **QoS 方向搞反会让急停彻底收不到指令**。给 `estop_latch` 的输入订阅设
   `transient_local` 时，网页按钮和 `ros2 topic pub` 这些易失发布者**一条都发不进来**
   ——ROS 2 要求"发布端持久性 ≥ 订阅端"，只会在日志里留一行 QoS 不兼容的警告。
   输入必须用默认的 volatile，只有输出才需要 `transient_local`。
3. **`behavior_server` 会绕过整条安全链**。spin/backup/drive_on_heading 这些恢复行为
   默认**直接发布 `/cmd_vel`**（上游 Nav2 不重映射，因为它默认没有中间层）。
   插了这个链路之后后果很严重：恢复行为既不受急停约束、也不受 `collision_monitor`
   约束，还会和兜底级的输出在底盘上交错。已在 `navigation_safe_launch.py` 里把它
   重映射到 `/cmd_vel_recovery`，经 `recovery_guard` 做方向性放行后再进入仲裁层
   （不并入导航指令流：脱困需要"贴着的障碍允许保持间隙或远离"的宽松策略）。
   ——这个问题是 `cmd_vel_deadman` 的"外来发布者自检"报出来的，那个自检值得保留。

这三条都不是看代码能发现的，是实际跑起来才暴露的。

- **`collision_monitor.yaml` 的预测窗口**：轮廓来自 `/local_costmap/published_footprint`
  （±0.345 × ±0.285 m），`time_before_collision: 1.0` 在 0.26 m/s 下前探约 0.26 m，
  制动距离只有约 1.4 cm，余量充足；但**如果之后放大 `max_vel_x`，必须同步加大该窗口**。
- **`deadman.yaml` 的 `timeout`（0.3s）**：要小于上游正常发布周期，否则正常行驶
  时会误触发停车。当前 LIO/规控链路远快于这个值。
- **看门狗的 `tf_timeouts`（1.0s）**：要大于 LIO 和重定位节点的正常发布周期。

## 已知边界

- 只做**运动指令层面**的安全，不涉及功能安全认证（ISO 3691-4 那类需要安全 PLC、
  安全激光雷达、急停双回路等硬件冗余，软件层替代不了）。
- 急停是**软件急停**（`/safety/estop` 话题），和底盘硬件急停是两回事，两者都应保留。
- `localization_watchdog` 只看 TF/话题**是否新鲜**，不做位姿跳变或配准残差判别。
  重定位"收敛后又缓慢跟丢"这种情况它发现不了——需要重定位节点暴露收敛度指标才
  能做到，见 [docs/roadmap.md](docs/roadmap.md)。
- `twist_mux` 需要单独安装（`sudo apt install ros-humble-twist-mux`）。
## 目录

```
config/    四个节点的参数
launch/    safety_layer（只起安全层）/ navigation_safe（安全版 Nav2 导航）/
           safe_nav_bringup（完整替代 my_nav2_launch.py）
nav2_safety/  deadman 与 localization_watchdog 节点实现
docs/      ENABLING.md 启用步骤；roadmap.md 后续改进清单
```
