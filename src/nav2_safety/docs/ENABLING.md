# 安全层：已落地的改动与使用方法

## 已经改了什么

安全层的节点和配置全是新增文件。为了让整条链真的接上，改了四处现有文件：

| 文件 | 改动 |
|---|---|
| `src/web/app.js` | 遥控发布话题 `/cmd_vel` → `/cmd_vel_teleop` |
| `src/gui_teleop/gui_teleop/gui_teleop_node.py` | 同上（含状态栏文案） |
| `scripts/nav2_real.sh` | 导航启动换成 `nav2_safety/safe_nav_bringup_launch.py`；清理列表加上 `ros2 launch nav2_safety` |
| `scripts/mapping_real.sh` | 增加"安全层"终端（`use_collision_monitor:=false`） |

**为什么遥控必须换话题**：启用安全层后 `/cmd_vel` 由 `cmd_vel_deadman` 独占
（底盘把这个话题名硬编码在 C++ 里，改不了）。遥控如果还发在 `/cmd_vel`，就会和
兜底级的指令在底盘上交错，出现抖动。兜底级启动后会自检 `/cmd_vel` 上的外来发布者
并报错，所以即使漏改也会立刻在日志里看到，不会静默出问题。

**为什么建图也要起安全层**：因为遥控话题已经改了，建图流程如果不跑仲裁层，
网页和 `gui_teleop` 发的 `/cmd_vel_teleop` 就没人接收，遥控会失效。建图时关掉
`collision_monitor`（没有 Nav2，它没有输入可监控）。

## 怎么跑

```bash
cd scripts && ./build.sh      # 或 colcon build --packages-select nav2_safety --symlink-install
./nav2_real.sh                # 导航（含安全层）
./mapping_real.sh             # 建图（含轻量安全层）
```

遥控话题变了，`gui_teleop` 和网页不需要额外操作，它们现在发到 `/cmd_vel_teleop`。

只想单独起安全层配合自己的导航启动：

```bash
ros2 launch nav2_safety safety_layer_launch.py
ros2 launch nav2_safety safety_layer_launch.py use_collision_monitor:=false
```

## 启动顺序与预期现象

- `collision_monitor` 需要 `/scan` 和 TF 才能激活。整栈一起起时它可能先失败几次，
  lifecycle_manager 会重试，等 `pointcloud_to_laserscan` 和 LIO 起来后自动激活。
  想要日志干净就先起传感器/LIO，再起安全层。
- `localization_watchdog` 有 5 秒启动宽限，宽限期内按"锁住自动驾驶"处理（会打印
  一条 info 说明），等 `map -> odom` 出现后自动放行。
- 急停锁存后，`ros2 topic pub --once /safety/estop std_msgs/msg/Bool "{data: false}"`
  可以解除。

## 验证

```bash
ros2 topic echo /safety/watchdog_status     # 各项 TF/点云的年龄与健康判定
ros2 topic echo /safety/deadman_status      # stopping=True 表示正在下发零速
ros2 topic echo /safety/estop_status        # 急停是否锁存
ros2 topic info /cmd_vel -v                 # 应只有 cmd_vel_deadman 一个发布者
ros2 topic hz /cmd_vel_teleop               # 遥控时能看到
```

最直观的是**网页控制台**（`scripts/web_start.sh`）侧栏的"安全状态"面板：
它把这四项状态和看门狗明细直接显示出来，并带急停/解除按钮。

人工验证两条关键行为（**先把车架起来、轮子离地**）：

```bash
# 1) 仲裁：自动导航跑起来后动一下遥控 —— 遥控应当立刻接管
# 2) 急停：点网页的"🛑 急停"，或在终端里发：
ros2 topic pub --once /safety/estop std_msgs/msg/Bool "{data: true}"
ros2 topic echo /safety/estop_status
# 此时发遥控指令，/cmd_vel 应恒为 0（被急停压制）
```

## 上线前必须先在台架上确认的两件事

1. **底盘 500ms 通讯保护的实际值**。BUNKER 手册（protocol v1）写了"超过 500MS 未
   收到下一帧控制指令，速度为 0"，但 Bunker Mini 2.0 用哪个协议版本、超时是否相同
   仓库里没有文档。测法：低速下发 `/cmd_vel_teleop`，然后停止发布，用
   `/bunker_status` 的 `linear_velocity` 看多久归零。
2. **`collision_monitor.yaml` 的预测窗口**。轮廓来自
   `/local_costmap/published_footprint`（±0.345 × ±0.285 m），`time_before_collision: 1.0`
   在 0.26 m/s 下前探约 0.26 m，制动距离约 1.4 cm，余量充足。**但如果以后放大
   `max_vel_x`，必须同步加大该窗口**，否则余量会不够。

## 回滚

四步改动逐条反向即可：导航启动换回 `my_nav2_launch.py`、删掉 `mapping_real.sh` 里
的安全层块、遥控话题换回 `/cmd_vel`、`launch_manager_gui.py` 里那条任务命令换回。
`nav2_safety` 包留着不启动就没有任何影响。

## 如果暂时不想要仲裁（只保留独立障碍闸）

不启动 `twist_mux`、让 Nav2 输出直接进 deadman 也可以，但那样**遥控必须留在
`/cmd_vel`**（因为 mux 不在了，`/cmd_vel_teleop` 没人接）。此时把
`src/web/app.js` 和 `gui_teleop` 的话题改回 `/cmd_vel`、并给
`safety_layer_launch.py` 加一个跳过 mux 的开关。这条路目前没有实现，需要的话告诉我。
