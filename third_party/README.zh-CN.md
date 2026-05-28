# 第三方资产

Arachne 使用上游模型包来描述真实 Scout 2.0 和 Aubo i5：

- `aubo_description`：来自 `https://github.com/AuboRobot/aubo_description`
- `scout_ros2`：来自 `https://github.com/agilexrobotics/scout_ros2`
- `MS42DC.step`：当前柔性夹爪的本地 CAD 来源。提交到运行时的 mesh 由该文件生成，并存放在 `src/arachne_description/meshes/gripper/ms42dc/`。
- `dh_ag95_gripper_ros2`：可选 AG95 模型包，来自 `https://github.com/ian-chuang/dh_ag95_gripper_ros2`

后续如果添加带许可证的 CAD、STL、SDK 或说明书文件，需要在这里记录来源、许可证、版本和 checksum。如果某个 vendor 文件不能再分发，应把它放在仓库外部，并在 `docs/references.md` 中记录预期本地路径。
