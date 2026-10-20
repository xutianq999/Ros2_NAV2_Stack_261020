# 先验点云目录（prior PCD）

`tracking` / `tracking_kiss` 模式使用的先验点云放在这里，默认路径：

```yaml
# src/me_nav2_bringup/config/map_config.yaml
prior_pcd_file: "pcd/map.pcd"
```

使用步骤：

1. 先用 `./scripts/super_lio_nav.sh mapping` 建图，退出时得到
   `$SUPERLIO_WS/src/super_lio/map/map_explore.pcd`；
2. 把它复制为本目录下的 `map.pcd`（或改成你自己的文件名）；
3. 确认 `map_config.yaml` 的 `prior_pcd_file` 指向它；
4. 启动 `./scripts/super_lio_nav.sh tracking` 或 `tracking_kiss`。

> PCD 文件较大，已被 `.gitignore` 排除，不会提交到仓库。
> Super-LIO 自带的 `relocation` 模式读取的是它自己工作空间里的
> `$SUPERLIO_WS/src/super_lio/map/map.pcd`，不是这个目录。
