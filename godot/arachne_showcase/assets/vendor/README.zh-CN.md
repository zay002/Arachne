# Vendor Mesh 链接

该目录由 `tools/link_assets.sh` 或 `scripts/godot_showcase.sh` 填充。

这些链接指向现有 Arachne 资产，而不是复制大型第三方 mesh：

- `scout`：来自 `third_party/scout_ros2` 的 Scout 2.0 底盘和轮子 mesh。
- `aubo_i5`：来自 `third_party/aubo_description` 的 Aubo i5 视觉 link。
- `ms42dc`：来自 `src/arachne_description`、由项目作者手动拆分的 MS42DC 可动 STL 零件。
- `ag95`：来自 `third_party/dh_ag95_gripper_ros2` 的可选 AG95 视觉 mesh。
- `props`：当可用时，来自 `third_party/LARA_AUBOi5_AG95` 的简单物体 mesh。

当 `assimp` 可用时，DAE 资产也会转换为 `assets/generated/` 下的本地 GLB 缓存文件。符号链接和生成缓存都会被 git 忽略。
