# Web 控制台（ROS2 / Nav2 版）

基于 rosbridge + Robot Web Tools 的机器人网页控制台，适配本工作区
（Livox MID360 + Super-LIO / FAST-LIO + small_gicp 重定位 + Nav2）的导航链路。

页面只使用 Nav2 的标准话题和 TF（`/map`、`/goal_pose`、`/initialpose`、
`map -> odom -> base_footprint`），因此 Super-LIO tracking 模式和旧的
FAST-LIO 链路都可以直接使用，不需要按定位方案改页面。

## 页面

| 文件 | 功能 |
|------|------|
| `workbench.html` | **新版导航工作台（第一版）**：同一 App Shell 下的页面 —— 工作区 / 导航 / 建图 / 探索 / 任务 / 机器人 / 工具 / 系统 / 设置 / 三维点云；含 ETA、实际与指令速度、健康链、系统诊断、电量/BMS、长按软件急停、手动控制 |
| `navigation.html` | **导航主页面（旧版）**：2D 地图显示（栅格 + 全局/局部代价地图 + 全局路径）、点击单点导航、多点航点顺序导航（支持循环）、设置初始位姿、取消目标、**安全状态面板与软件急停按钮** |
| `index.html` | Robot Center 仪表盘：3D 点云（/registered_scan）、IMU（/livox/imu）、手动遥控（/cmd_vel_teleop） |
| `lib/` | 本地化的 roslib / three / ros3d 库（离线可用，加载失败自动回退 CDN） |

## 环境准备（一次性）

机器人端需要 rosbridge（网页与 ROS2 的 WebSocket 桥）：

```bash
sudo apt install ros-humble-rosbridge-suite
```

## 启动步骤

### 1. 启动导航栈

Super-LIO（当前推荐，tracking 模式，不开 RViz）：

```bash
./scripts/super_lio_nav.sh tracking false
```

旧的 FAST-LIO 链路：

```bash
./scripts/nav2_real.sh
```

### 2. 启动网页服务

```bash
./scripts/web_start.sh
```

该脚本会：检测并启动 rosbridge（端口 9090）→ 在 5500 端口起静态文件服务。
也可以手动分步执行：

```bash
# rosbridge
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 静态文件服务
cd src/web
python3 -m http.server 5500 --bind 0.0.0.0
```

### 3. 浏览器访问

同一局域网的任意机器打开：

- 工作台：`http://<机器人IP>:5500/workbench.html`
- 导航页：`http://<机器人IP>:5500/navigation.html`
- 仪表盘：`http://<机器人IP>:5500/index.html`

rosbridge 地址默认自动取当前主机名（`ws://<机器人IP>:9090`），也可以在页面侧栏手动修改后点「连接」。

## 导航页话题（本项目默认值）

| 用途 | 接口 | 类型 | 说明 |
|------|------|------|------|
| 静态地图 | `/map` | nav_msgs/OccupancyGrid | map_server 发布，transient_local 锁存 |
| 全局代价地图 | `/global_costmap/costmap` | nav_msgs/OccupancyGrid | 含静态层+障碍层+膨胀层 |
| 局部代价地图 | `/local_costmap/costmap` | nav_msgs/OccupancyGrid | 默认不显示，图层里可打开 |
| 全局路径 | `/plan` | nav_msgs/Path | planner_server 实际发布的规划结果（旧文档里的 `/planner_server/plan` 没有发布者） |
| 机器人位姿 | TF: `map → odom → base_footprint` | tf2_msgs/TFMessage | 网页内自解析，**无需 tf2_web_republisher** |
| 单点目标 | `/goal_pose` | geometry_msgs/PoseStamped | Humble bt_navigator 原生订阅，等同 RViz "2D Goal Pose" |
| 初始位姿 | `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | 供 small_gicp 重定位，等同 RViz "2D Pose Estimate" |
| 取消目标 | `/navigate_to_pose/_action/cancel_goal` 服务 | action_msgs/srv/CancelGoal | 空 uuid 取消全部目标 |
| 航点完成检测 | `/navigate_to_pose/_action/status` | action_msgs/GoalStatusArray | SUCCEEDED(4) 自动发下一航点 |

> 话题名可以在页面侧栏修改，改完自动重订阅。`/goal_pose`、`/initialpose`、取消服务为固定接口（与 RViz/Nav2 标准一致）。

## 安全状态面板（nav2_safety）

侧栏「安全状态」面板订阅下列话题，把安全层的判断直接显示出来，并带急停/解除按钮。
目的是让"车为什么不动"当场可见——否则现场只能看到车不动，容易误判为设备故障。

| 用途 | 接口 | 类型 | 说明 |
|------|------|------|------|
| 自动驾驶 | `/safety/autonomy_blocked` | std_msgs/Bool | true = 看门狗锁住了自动驾驶（定位不可信） |
| 停止指令 | `/safety/deadman_stopping` | std_msgs/Bool | true = 兜底级正在下发零速 |
| 重定位跟踪 | `/localization/tracking_ok` | std_msgs/Bool | false = GICP 跟丢（位姿已不可信，但 TF 仍在发） |
| 急停状态 | `/safety/estop_active` | std_msgs/Bool | 锁存状态，由 `estop_latch` 持续发布（读这个） |
| 判定明细 | `/safety/watchdog_status` | std_msgs/String | 各项检查的年龄与健康判定 |
| 急停写入 | `/safety/estop` | std_msgs/Bool | 发 true 锁存、false 解除（按钮就是发这个话题） |

> 急停读的是 `/safety/estop_active`（锁存、transient_local）而不是那条一次性指令，
> 否则页面重连后会显示"未触发"，而急停其实还锁着。详见 `src/nav2_safety/README.md`。

## 电池 / BMS

工作台的“电池”卡片读的是 `/robot/status` 里的 `battery` 字段（由
`robot_status_monitor` 聚合），不直接订阅驱动消息。数据链路：

```
bunker_base → /bunker_status (bunker_msgs/BunkerStatus)
            → robot_status_monitor → /robot/status (JSON) → workbench.html
```

`battery` 字段：

| 字段 | 说明 |
|------|------|
| `online` | 是否收到新鲜的 `/bunker_status` |
| `voltage` | 电池电压 V |
| `soc` | BMS 荷电状态 %；未知时为 `null` |
| `soh` | BMS 健康度 % |
| `current` | 电流 A；正=放电，负=充电（BMS 约定） |
| `temperature` | 电池温度 °C |
| `charging` | 是否在充电（由电流符号和阈值判断） |
| `source` | `bms` / `voltage`（BMS 不发 SOC 时按电压估算）/ `none` |
| `low` / `critical` | 低电量 / 严重低电量阈值标记 |

相关参数在 `src/nav2_safety/config/robot_status_monitor.yaml` 的 `battery_*`。
电压估算的满/空电压默认按 7S 锂电（29.4 / 21.0 V）填写，实际车型请以说明书为准。

## 使用说明

- **单点导航**：默认「🎯 目标」模式下点击地图只会放置一个“待下发”目标（橙色虚线箭头），
  机器人不会立即移动。此时可以按住箭头拖动、或在地图工具栏的“朝向”里填写目标车头朝向
  （度，0=东，90=北；留空则自动朝向目标点），确认后再点“下发”或按回车。点“取消”放弃本次选点。
  下发后「取消目标」按钮停止当前导航。
- **多点航点**：切到「🧭 航点」模式，依次点击添加航点 → 「▶️ 开始导航」。机器人按顺序逐点导航，
  到达一点自动发下一点；勾选「循环导航」则跑完一圈后循环。单点超过 120 秒未到达会超时停止并自动取消目标。
  「⏸️ 停止」会同时取消正在执行的当前目标；导航进行中删除航点也会先终止任务。
  航点默认朝向为其相对机器人当前位置的方向；可以在地图上按住航点拖动，或在航点列表里输入角度，
  设置每个航点到达后的具体车头朝向。
  航点到达判定基于 action 状态话题（对周期性重发的状态数组做了去重，不会跳点）。
- **设置初始位姿**：切到「📍 初始位姿」模式点击地图，向重定位节点发送先验位姿
  （重定位未收敛、GICP 跟丢时使用）。这也是**定位跟丢后的恢复动作**。
- **安全状态 / 急停**：侧栏面板显示自动驾驶是否被锁、兜底级是否在下发零速、重定位
  是否跟丢、急停是否锁存。"🛑 急停"会锁存并压制所有速度来源（含遥控），需点
  "解除急停"才能恢复。
- **手动遥控**（仪表盘页）：发布 `/cmd_vel_teleop`，经安全层按优先级放到 `/cmd_vel`。
  安全层未启动时该话题无人接收，遥控会失效——所以建图和导航脚本都会启动安全层。

## 常见问题

- **地图一直不显示**：确认导航栈已启动（`ros2 topic echo /map --once` 有输出）；
  页面 3 秒收不到锁存 `/map` 时会自动改用 `/map_server/load_map` 服务拉取，若仍失败检查 rosbridge 是否运行。
- **机器人箭头变半透明**：TF 超过 2 秒未更新（定位或里程计断流），检查 Super-LIO / 重定位节点。
- **航点卡住不动**：查看 Nav2 日志是否有规划失败；航点在障碍物内时 Smac 2D 会在 0.5 m 容差内就近选点。
- **点云很卡**：仪表盘页点云已限 2 Hz + 48000 点；如仍卡可在 Configuration 里换话题或降低带宽。

## 工作台子页面（第一版）

`workbench.html` 已从单页扩成多页面，顶部/左侧切换：

| 页面 | 当前实现 |
|------|----------|
| 工作区 | 多工作区与巡检方案管理；地图上传/加载、点选点位（停留时间、yaw）、文本标注、先验 PCD 上传都在这一页完成 |
| 导航 / 建图 / 探索 | 实时 2D 地图、路径、机器人位姿、健康链；建图/探索按钮会向 `/exploration/mode`、`/explore/resume` 发布请求 |
| 任务 | 航点表格、当前目标、剩余距离、循环、巡检方案切换、停止/清空/载入配置点位；真实复用现有航点状态机 |
| 机器人 | 电池电压/SOC/电流/温度/SOH、IMU 姿态与加速度、手动控制；遥控走 `/cmd_vel_teleop` |
| 工具 | 启动模块状态来自实时话题；启动/停止、保存地图、PCD 转栅格、TF、录制仍提示终端命令 |
| 系统 | 运行链、健康检查、安全状态、恢复行为；数据来自 `/robot/status` |
| 设置 | 连接、地图话题、图层、显示与控制上限；连接/话题/图层已接入现有逻辑 |
| 三维点云 | 内嵌 `index.html`（ros3d，2 Hz / 48000 点上限） |

说明：地图保存、PCD 转栅格、启动管理器的启停、点位文件写入仍需要终端脚本或后端，页面目前给出命令提示，不会假装已经执行。建图/探索页的地图预览使用同一份 2D 栅格，前沿和边界的彩色叠加尚未接入。

页面末尾暴露了 `window.__wb` / `window.__state`，仅用于自动化/调试，不影响正常使用。

## 后端（第一版）

`./scripts/web_start.sh` 现在启动 rosbridge（9090）和 `scripts/web_backend.py`（5500）。
后端同时托管 `src/web` 静态页面和 `/api`，所以页面和接口同源。
只想单独启动后端时用 `./scripts/web_backend.sh --port 5500`；如果页面由别的静态服务器托管，
可以在 URL 上加 `?api=http://<机器人IP>:5500/api` 指定后端地址：

```text
http://<机器人IP>:5500/workbench.html
http://<机器人IP>:5500/api/mission/state
```

已接的能力：

| 能力 | 接口 / 存储 |
|------|-------------|
| 点位持久化 | `GET/POST/DELETE /api/points`、`POST /api/points/save` → 当前工作区/当前巡检方案的点位表；默认方案会镜像到 `src/me_nav2_bringup/config/waypoints.yaml`；每个点位可带 `dwell`（到达后停留秒数） |
| 地图加载 | `GET /api/map/sources` 扫描工作区地图；`POST /api/map/upload` 接收浏览器选中的 PGM+YAML；`POST /api/map/load` 校验并返回尺寸/分辨率/origin；`GET /api/map/image` 渲染为 PNG |
| 工作区 | `GET/POST /api/workspaces`、`POST /api/workspaces/{id}/activate`、`DELETE /api/workspaces/{id}`、`POST /api/workspaces/{id}/pcd`、`POST /api/workspace/map`；点位/标注接口自动按当前工作区隔离 |
| 巡检方案 | `GET/POST /api/plans`、`PUT /api/plans/{id}`、`POST /api/plans/{id}/activate`、`DELETE /api/plans/{id}`；同一工作区共用地图和标注，每个方案有独立点位表 |
| 地图标注 | `GET/POST/PUT/DELETE /api/annotations`、`POST /api/annotations/save` → 当前工作区的 `annotations.yaml`；导航与各子页面地图都会叠加显示 |
| 任务持久化 | `GET/POST/PUT/DELETE /api/missions` → `web_backend_data/missions.db` |
| 服务端任务执行 | `POST /api/mission/start`、`/pause`、`/resume`、`/cancel`、`/reset`；rclpy NavigateToPose action |
| 任务状态恢复 | 状态写入 `web_backend_data/runtime_mission.json`；后端重启后标记 `interrupted`，可点“继续任务” |
| 任务状态 | `GET /api/mission/state`（索引、距离、失败原因） |
| 任务历史 | `GET /api/mission/history?limit=50`、`DELETE /api/mission/history` → `mission_runs` 表；成功/失败/取消/中断都会落库，恢复后再次结束会更新原记录 |
| 鉴权（可选） | 启动时加 `--token` 或设置 `BUNKER_WEB_TOKEN`；启用后 `/api/*` 需要 `Authorization: Bearer <token>`，页面会弹出登录框 |
| 保存二维地图 | `POST /api/map/save2d` → `nav2_map_server/map_saver_cli` |
| 地图文件列表 | `GET /api/map/files` |
| PCD 转栅格 | `POST /api/tools/pcd2grid` → `scripts/pcd_to_occupancy_map.py` |
| 机器人状态 | `GET /api/status`（后端订阅 `/robot/status`） |
| 健康检查 | `GET /api/health` |

任务由后端执行，**关闭浏览器不会中断任务**。任务执行依赖 Nav2 的 `navigate_to_pose` action；
没有启动 Nav2 时任务会进入 `failed` 并返回原因。

3D PCD 仍由 Super-LIO 在退出建图时保存，`/api/map/save3d` 只返回说明，不会强行停止建图进程。

### 地图编辑（选点 / 停留 / 标注）

左侧“工作区”页的地图编辑区用于在离线地图上直接准备导航数据：

1. 点“选择 PGM”和“选择 YAML”分别选中两个文件（也可以点“一次多选”同时选，或“选择文件夹”
   让页面按 YAML 的 `image` 字段自动配对）；文件上传到 `web_backend_data/map_uploads/` 后加载。
   也可以继续在下面填服务器端路径，点“加载地图”。
2. 切到“选点”后点击地图，点位立即写入 `waypoints.yaml`；名称留空自动编号，
   “到达后停留”会写入点位的 `dwell` 字段（秒），“朝向”写入 `yaw`（度，页面内部转成弧度）。
   点位在地图上会画出朝向箭头；也可以按住点位拖出方向，或在点位列表里直接改角度。
3. 点“载入为任务队列”把当前点位送到任务页即可开始导航。服务端执行器下发的目标位姿会带上
   该点 `yaw`，到达后等待 `dwell` 秒，再前往下一个点位。
4. 切到“文本标注”，填写文字、颜色和字号后点击地图，标注写入
   `config/map_annotations.yaml`；导航主地图和各子页面地图都会叠加显示。

工作区页的地图编辑区支持滚轮缩放、“浏览/平移”模式下拖动，以及点列表/标注列表里的删除、前往和加入队列按钮。

### 工作区（多地图 / 多套巡检点）

顶部“工作区：…”或左侧“工作区”页用于管理多套地图。每套工作区包含自己的二维地图、
先验 PCD、巡检点位和文本标注，互不影响：

- 每个工作区的点位表、文本标注、二维地图和先验 PCD 都存放在
  `web_backend_data/workspaces/ws_<id>_<slug>/` 下，工作区之间完全隔离。
- 默认工作区首次启动会自动把原来的 `config/waypoints.yaml`、`map_config.yaml` 指向的
  二维地图和 PCD 复制进来，并把 `map_config.yaml` 改指向工作区内的副本；原文件保留作备份。
  旧的 `config/waypoints.yaml` 仍会同步一份镜像，供 `voice_agent`、`waypoint_recorder`
  等继续读取全局路径的节点使用。
- 文本标注存放在工作区的 `annotations.yaml`；默认工作区启动时会从旧的
  `config/map_annotations.yaml` 自动迁移，旧文件保留作备份。工作区页可以直接查看和删除标注。
- 工作区目录结构：`points.yaml`（默认方案）、`annotations.yaml`、`maps/`、`plans/`、`prior_pcd/`。
- 点位和标注接口始终作用于“当前工作区”，切换后任务页和工作区页会一起切换。
- 保存的任务模板（`missions`）和任务历史（`mission_runs`）也绑定工作区：切换后只看到
  当前工作区的记录，跨工作区按 mission id 启动会返回 404；删除工作区会一并删除其任务和历史。
- 上传二维地图时会自动把它记为当前工作区地图；“设为当前工作区地图”可以把扫描到的
  现有地图绑定到工作区。
- 先验 PCD 在工作区页上传，供 Super-LIO 的 tracking / tracking_kiss 模式使用。
- 同一个工作区内还可以有多个**巡检方案**（不同点位表）：地图和文本标注共用，
  每个方案独立保存点位和停留时间。顶部下拉、任务页和“工作区”页都可以切换；
  新建方案后会立即切过去，工作区页新增的点位写入当前方案。
- 方案点位文件：默认方案沿用工作区原来的 `points.yaml`，其他方案放在
  `web_backend_data/workspaces/ws_<id>_<slug>/plans/plan_<id>_<slug>/points.yaml`。
- 切换工作区会立即把该工作区的二维地图和先验 PCD 写入 `config/map_config.yaml`，
  **需要重启导航栈才会真正加载新地图**；有任务在执行时不允许切换工作区或方案。

安全说明：默认**没有鉴权**，只适合可信局域网。需要给接口加一道门时，用固定令牌启动：

```bash
BUNKER_WEB_TOKEN='换成足够长的随机串' ./scripts/web_backend.sh --host 0.0.0.0 --port 5500
# 或
./scripts/web_backend.sh --token '换成足够长的随机串'
```

启用后静态页面仍能打开，但页面会弹出登录框；令牌保存在浏览器 localStorage。
也可以直接调用 `POST /api/auth/login`（`{"token": "..."}`）后用 Bearer 头访问。
鉴权只是访问控制，不是 TLS；如果链路不可信，仍应通过反向代理加 HTTPS。

### 任务执行器测试

不需要真车，也不需要启动真实 Nav2；脚本会起一个假 `navigate_to_pose` action server：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/test_mission_executor.py
```

覆盖：成功到达、到达后停留、暂停/继续、取消、点位失败、reset，以及终态写入任务历史（含点位数和耗时）。脚本使用 `ROS_DOMAIN_ID=97`，
不要和正在运行的导航栈共用这个 domain。
