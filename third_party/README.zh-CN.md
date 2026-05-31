# 第三方资产

Arachne 使用上游模型、驱动和素材包来支撑真实 Scout 2.0 与 Aubo i5 系统。只包含 `.gitkeep` 的空目录用于标记预期的本地下载/解压目标：

- `aubo_description`：来自 `https://github.com/AuboRobot/aubo_description`
- `aubo_ros2_driver`：来自 `https://github.com/AuboRobot/aubo_ros2_driver`，并会为 Arachne 当前 Aubo bringup 流程应用本地补丁
- `scout_ros2`：来自 `https://github.com/agilexrobotics/scout_ros2`
- `ugv_sdk`：来自 `https://github.com/agilexrobotics/ugv_sdk`
- `MS42DC.step`：当前柔性夹爪的本地 CAD 来源。提交到运行时的 mesh 由该文件生成，并存放在 `src/arachne_description/meshes/gripper/ms42dc/`。
- `dh_ag95_gripper_ros2`：可选 AG95 模型包，来自 `https://github.com/ian-chuang/dh_ag95_gripper_ros2`
- `ms42dc_step_motor_ros2`：需要厂家官方示例时，从 MS42DC 厂家 ROS2 zip 本地解压生成
- `kenney`、`LARA_AUBOi5_AG95`、`scout_ros`：Godot 展示前端可选使用的本地素材/模型来源

后续如果添加带许可证的 CAD、STL、SDK 或说明书文件，需要在这里记录来源、许可证、版本和 checksum。如果某个 vendor 文件不能再分发，应把它放在仓库外部，并在 `docs/references.md` 中记录预期本地路径。
