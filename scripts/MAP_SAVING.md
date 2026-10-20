# 探索地图保存功能说明

## 功能概述

`explore_record.sh` 脚本现在在探索结束时（Ctrl-C）会自动保存两个地图文件：

1. **2D 栅格地图** (`map.pgm` + `map.yaml`) - 用于 Nav2 全局路径规划
2. **3D 点云地图** (`global_map.pcd`) - 用于 Super-LIO 重定位

## 使用方法

### 默认行为（保存地图）
```bash
./scripts/explore_record.sh
```

探索结束后，地图会保存到 `runs/explore_<timestamp>/maps/` 目录：
```
runs/explore_20261018_143022/
├── maps/
│   ├── map.pgm              # 2D 栅格地图（灰度图像）
│   ├── map.yaml             # 2D 地图元数据（分辨率、原点等）
│   ├── global_map.pcd       # 3D 点云地图
│   └── map_saver.log        # 地图保存日志
├── rosbag/
├── config/
└── ...
```

### 禁用地图保存
如果你只想录制 rosbag 而不需要保存地图：
```bash
./scripts/explore_record.sh --no-save-maps
```

### 强制启用地图保存
```bash
./scripts/explore_record.sh --save-maps
```

## 技术细节

### 2D 栅格地图
- **来源**: `/map` 话题（Nav2 map_server 发布的占据栅格地图）
- **工具**: `nav2_map_server` 的 `map_saver_cli`
- **超时**: 30秒（如果地图数据不可用会跳过）
- **用途**: 
  - 用于 Nav2 的全局规划器（NavFn/Smac）
  - 可在 RViz 中查看 `.pgm` 文件
  - 后续导航时加载为先验地图

### 3D 点云地图
- **来源**: Super-LIO 节点退出时自动保存
- **配置文件**: `super_lio_mid360_mapping.yaml`
  - `lio.map.save_map: true`
  - `lio.map.map_name: "map_explore.pcd"`
  - `lio.map.save_map_dir: "map"` (相对于 Super-LIO 源码目录)
- **原始路径**: `/home/demo/superlio_ws/Super-LIO-ros2/src/super_lio/map/map_explore.pcd`
- **复制延迟**: 脚本会等待 3 秒让 Super-LIO 完成保存
- **用途**:
  - 用于 Super-LIO 的重定位（在已知地图中重新定位机器人位姿）
  - 可使用 PCL 工具或 CloudCompare 查看
  - 后续导航时加载为先验地图

## 故障排查

### 2D 地图保存失败
检查 `maps/map_saver.log` 日志文件：
```bash
cat runs/explore_<timestamp>/maps/map_saver.log
```

常见原因：
- `/map` 话题没有数据（map_server 未启动或探索未产生地图）
- map_saver_cli 超时（地图数据太大或服务未响应）

### 3D 点云地图未找到
检查 Super-LIO 配置：
```bash
grep -E "save_map|map_name" src/super_lio/super_lio_bridge/config/super_lio_mid360_mapping.yaml
```

确保：
- `lio.map.save_map: true`
- `lio.map.map_name: "map_explore.pcd"`
- Super-LIO 节点正常关闭（不是被 SIGKILL）

手动检查 Super-LIO 地图目录：
```bash
ls -lh /home/demo/superlio_ws/Super-LIO-ros2/src/super_lio/map/
```

## 后续使用地图

### 使用 2D 地图进行导航
将地图复制到配置目录：
```bash
cp runs/explore_<timestamp>/maps/map.* src/me_nav2_bringup/maps/
```

修改 Nav2 启动配置，加载先验地图：
```yaml
map_server:
  ros__parameters:
    yaml_filename: "maps/map.yaml"
```

### 使用 3D 地图进行重定位
将点云地图复制到 Super-LIO 配置目录：
```bash
cp runs/explore_<timestamp>/maps/global_map.pcd \
   /home/demo/superlio_ws/Super-LIO-ros2/src/super_lio/map/map.pcd
```

启动 Super-LIO 时会自动加载 `map.pcd` 进行重定位。

## 相关文件

- 脚本: `scripts/explore_record.sh`
- Super-LIO 配置: `src/super_lio/super_lio_bridge/config/super_lio_mid360_mapping.yaml`
- Nav2 地图服务: `src/me_nav2_bringup/launch/my_nav2_launch.py`
