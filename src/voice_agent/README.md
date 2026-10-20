# voice_agent — 小车语音对话与动作 Agent

给 BUNKER MINI 2.0 + Nav2 导航栈加一层中文语音交互：

- **语音对话**：浏览器麦克风 → MiMo ASR → MiMo LLM → MiMo TTS；
- **口头遥控**：前进、后退、原地左转/右转、停止、急停；
- **联合导航**：说“去 1 号点位”，通过 Nav2 `navigate_to_pose` 自动规划并行驶；
- **安全约束**：确定性口令优先于大模型、速度/距离上限、硬超时、里程计闭环、
  `/scan` 障碍门控、急停锁存、手动运动走 `twist_mux` 遥控通道。

```
浏览器麦克风
   │  WAV
   ▼
FastAPI /api/asr ──► MiMo ASR
   │ 文本
   ▼
确定性口令解析 ──命中──► RobotAgent ──► /cmd_vel_teleop 或 /cmd_vel
   │未命中                    │
   ▼                          └──────► Nav2 navigate_to_pose（去 N 号点位）
MiMo LLM + tool calling
   │ 文本
   ▼
FastAPI /api/tts ──► 浏览器播放
```

确定性解析器负责“停/急停/前进/后退/转向/去 N 号点位”这类明确口令，避免把
停车动作交给大模型碰运气；其他自由对话、复杂说法交给 MiMo 的 tool calling。

## 1. 目录

```text
src/voice_agent/
├── voice_agent/
│   ├── agent_node.py      # ROS 2 节点：/cmd_vel、Nav2 action、航点、急停、状态
│   ├── intent_parser.py   # 中文口令的确定性解析
│   ├── waypoints.py       # waypoints.yaml 读取与别名解析
│   ├── mimo_api.py        # MiMo LLM / ASR / TTS 异步封装
│   └── server.py          # FastAPI + rclpy 后台线程
├── config/
│   ├── voice_agent.yaml   # ROS 参数
│   └── waypoints.yaml     # 备用航点（默认优先用 me_nav2_bringup 的那份）
├── launch/voice_agent_launch.py
├── web/                   # 浏览器界面
├── test/                  # 口令解析 / 航点解析单元测试
├── requirements.txt
└── .env.example
```

## 2. 前置条件

- ROS 2 Humble（本工作区已验证）；
- 底盘驱动：`./scripts/chassis_start.sh`，订阅 `/cmd_vel`、发布 `/wheel_odom`；
- Nav2 + 定位（需要点位导航时）：按现有流程启动，例如
  `./scripts/super_lio_nav.sh relocation`；
- 可选安全层：`nav2_safety`（`twist_mux` / `collision_monitor` / 急停）。没有
  安全层也能用，但强烈建议实车开启。

Python 依赖需要装到 **`ros2` 同一个 Python 解释器**里。最省事的是装到用户目录：

```bash
cd /home/demo/Lidar_nav2_ws-main/Lidar_nav2_ws
python3 -m pip install --user -r src/voice_agent/requirements.txt
```

也可以用带 `--system-site-packages` 的虚拟环境做开发/测试；但用
`ros2 launch` 启动时必须确保该环境里的 `python3` 能被节点入口调用，否则会
`ModuleNotFoundError: fastapi`。实车部署建议直接用上面的 `--user` 方式。

## 3. 配置 MiMo Key

```bash
cp src/voice_agent/.env.example src/voice_agent/.env
# 编辑 src/voice_agent/.env，填入 MIMO_API_KEY
```

服务启动时按以下顺序查找 `.env`（先找到的先加载）：

1. `$VOICE_AGENT_ENV`
2. `~/.config/voice_agent/.env`
3. 当前目录 `.env`
4. `src/voice_agent/.env`
5. 工作区根目录 `.env`

## 4. 编译与启动

```bash
cd /home/demo/Lidar_nav2_ws-main/Lidar_nav2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select voice_agent --symlink-install

# 1) 底盘
./scripts/chassis_start.sh

# 2) 导航栈（需要点位导航时）
./scripts/super_lio_nav.sh relocation

# 3) 语音 Agent
source install/setup.bash
./scripts/voice_agent_start.sh
```

浏览器打开：

- 小车本机：<http://localhost:8000>
- 同一局域网：`http://<小车IP>:8000`

> **麦克风安全上下文**：浏览器只允许 `localhost` 或 HTTPS 页面调用
> `getUserMedia`。手机通过局域网 HTTP 访问时录音按钮会失败；三种解决方式：
> 1. 在小车本机浏览器里打开 `localhost`；
> 2. 给服务配置 `SSL_CERTFILE` / `SSL_KEYFILE`（见 `.env.example`），用 HTTPS 访问；
> 3. 用 Chrome 的 “unsafely-treat-insecure-origin-as-secure” 启动参数临时放行
>    （仅调试）。

## 5. 支持的语音/文本指令

| 说法示例 | 行为 |
| --- | --- |
| 前进 / 往前走 / 直行 | 默认 0.5 m |
| 前进两米 / 后退一米五 / 后退 50 厘米 | 指定距离（上限 2 m） |
| 左转 / 右转 | 默认 45°，原地转向 |
| 左转九十度 / 右转 30 度 | 指定角度（上限 180°） |
| 慢一点前进 / 快一点后退 | 速度缩放 0.6× / 1.3× |
| 停止 / 停下 / 刹车 / 别动 | 零速 + 取消导航 |
| 急停 | 零速 + 写 `/safety/estop` 锁存 |
| 解除急停 | 释放软件急停锁存 |
| 取消导航 | 取消 Nav2 目标 |
| 去 1 号点位 / 导航到 point_1 / 去三号点 | Nav2 `navigate_to_pose` |
| 有哪些点位 | 播报可用点位 |
| 状态 / 你在哪 / 电量 | 播报模式、位姿、传感器健康度 |
| 其他自然语言 | MiMo 自由对话；涉及动作时由 LLM 调用工具 |

差速底盘不能横向平移，说“向左平移”会明确提示只能左转/右转。

## 6. 航点

默认读取：

```text
src/me_nav2_bringup/config/waypoints.yaml
```

格式与现有录制脚本一致：

```yaml
frame: map
points:
- id: 1
  name: point_1
  x: 0.5904
  y: 0.2367
  yaw: -0.2533
  # 可选：给语音解析用的别名
  aliases: ["大门口", "入口"]
```

无需重启即可用现有的 RViz 2D Goal Pose 录制流程更新文件；本节点在启动时读取，
改完点位后重启 `voice_agent` 即可。语音别名自动包含：

```text
1 / 1号点 / 1号点位 / 第1个点 / 一号点 / point_1 / point 1 / 自定义 aliases
```

## 7. 安全设计（实车务必读）

1. **手动运动话题自动选择**：`cmd_vel_topic: auto` 时，如果检测到
   `/cmd_vel_teleop` 有订阅者（说明 `twist_mux` 正在运行），就发布到
   `/cmd_vel_teleop`；否则直接发布到 `/cmd_vel`。绝不双发。
2. **不绕过安全层**：语音手动运动走遥控优先级（100），急停（255）始终能压制；
   导航走 Nav2 → velocity_smoother → collision_monitor → twist_mux。
3. **急停**：语音“急停”/网页急停按钮会发布 `/safety/estop=true` 并由
   `estop_latch` 锁存，同时持续补发零速。恢复必须显式说“解除急停”。
4. **障碍门控**：`collision_monitor` 只约束导航指令，不约束遥控。本节点额外订阅
   `/scan`，前进/后退时检查前方/后方 ±35°、0.35 m 内是否有障碍；有就停。
   该门控是补充，不是功能安全认证的替代品。
5. **运动上限**：默认最大线速度 0.25 m/s、角速度 0.8 rad/s、单次距离 2 m、
   单次转角 180°，都有硬超时。
6. **里程计闭环**：`/wheel_odom` 新鲜时按实际位移/转角提前停止；没有里程计时
   按时间估算。里程计显示“有数据但不动”超过 1.5 倍预计时间会判定受阻并停车。
7. **HTTP 暴露面**：默认 `0.0.0.0:8000`，同一局域网的人都能发指令。建议设置
   `VOICE_AGENT_TOKEN`，并在网页右下角“令牌”里填入相同值；或只绑定
   `127.0.0.1`。

## 8. 话题与接口

### ROS 话题

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 发布 | `/cmd_vel_teleop` 或 `/cmd_vel` | `geometry_msgs/Twist` | 手动语音运动 |
| 发布 | `/navigate_to_pose`（action） | `nav2_msgs/NavigateToPose` | 点位导航 |
| 发布 | `/safety/estop` | `std_msgs/Bool` | 软件急停锁存 |
| 发布 | `/voice_agent/reply` | `std_msgs/String` | 最近一条语音回复 |
| 发布 | `/voice_agent/status` | `std_msgs/String` | JSON 状态，2 Hz |
| 订阅 | `/wheel_odom` | `nav_msgs/Odometry` | 里程计闭环 |
| 订阅 | `/scan` | `sensor_msgs/LaserScan` | 障碍门控 |
| 订阅 | `/voice_agent/text_command` | `std_msgs/String` | 其他节点/脚本下发文本口令 |
| 订阅 | `/safety/estop_active` | `std_msgs/Bool` | 急停锁存状态 |
| 订阅 | `/safety/autonomy_blocked` | `std_msgs/Bool` | 自动驾驶是否被锁 |

其他节点也可以直接发文本口令：

```bash
ros2 topic pub --once /voice_agent/text_command std_msgs/String "{data: '去1号点位'}"
```

### HTTP API

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET | `/api/info` | 模型、语音配置、机器人状态、点位列表 |
| POST | `/api/chat` | SSE 流式对话/工具调用；事件含 `delta`、`tool_call`、`robot`、`error` |
| POST | `/api/asr` | multipart `file=@recording.wav` |
| POST | `/api/tts` | `{"text": "...", "voice": "冰糖"}` |
| GET | `/api/robot/status` | 当前状态 JSON |
| POST | `/api/robot/command` | `{"text": "前进半米"}`，确定性口令 |
| POST | `/api/robot/navigate` | `{"point": "1号点位"}` |
| POST | `/api/robot/stop` | 立即停止 |
| POST | `/api/robot/estop` | `{"engage": true/false}` |

## 9. 参数

见 `config/voice_agent.yaml`，常用项：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `cmd_vel_topic` | `auto` | `auto` / `/cmd_vel` / `/cmd_vel_teleop` / 自定义 |
| `use_scan_guard` | `true` | 是否用 `/scan` 做障碍门控 |
| `require_scan_for_motion` | `false` | 没有扫描数据时是否禁止运动 |
| `scan_stop_distance` | `0.35` | 障碍停车距离（m） |
| `max_linear_speed` | `0.25` | 线速度上限（m/s） |
| `max_distance_m` | `2.0` | 单次前进/后退上限（m） |
| `max_turn_angle_deg` | `180.0` | 单次转角上限（°） |
| `default_turn_angle_deg` | `45.0` | “左转/右转”未说角度时的默认值 |
| `waypoints_file` | 空 | 留空自动查找 me_nav2_bringup 的航点文件 |
| `port` | `8000` | HTTP 端口 |

## 10. 测试

```bash
cd src/voice_agent
PYTHONPATH=. python3 -m unittest discover -s test -v
```

实车测试建议先用隔离 DDS 域，避免误动真车：

```bash
export ROS_DOMAIN_ID=77
ros2 run voice_agent voice_agent_node --ros-args \
  -p cmd_vel_topic:=/test_cmd_vel -p use_scan_guard:=false
# 另一个终端（同一 ROS_DOMAIN_ID）
ros2 topic pub --once /voice_agent/text_command std_msgs/String "{data: '前进0.2米'}"
ros2 topic echo /test_cmd_vel
```

## 11. 排障

- **网页能开但录音失败**：`localhost` 或 HTTPS 才能用麦克风，见第 4 节。
- **说“去1号点位”回复 Nav2 未就绪**：先启动 Nav2；确认
  `ros2 action list | grep navigate_to_pose`。
- **导航目标接受但车不动**：看 `/safety/autonomy_blocked` 是否为 true（定位/看门狗
  锁住），以及 `map->odom`、`odom->base_footprint` TF 是否正常。
- **手动前进不动**：确认底盘在跑、`/cmd_vel` 有订阅者；如果开了安全层，确认
  急停未锁存，并检查 `/cmd_vel_teleop` 是否有 `twist_mux` 订阅者。
- **语音识别/合成都报 503**：`.env` 里没有有效的 `MIMO_API_KEY`。
- **TTS 失败但文字正常**：前端会自动回退浏览器 `speechSynthesis`。
- **航点文件读不到**：`/api/waypoints` 会返回 `file` 和 `error`；用
  `waypoints_file:=/abs/path.yaml` 显式指定。
