# 参考资料

开发计划中使用的主要参考来源：

- Aubo 模型描述：https://github.com/AuboRobot/aubo_description
- Aubo ROS1 支持：https://github.com/AuboRobot/aubo_robot
- Aubo ROS2 驱动：https://github.com/AuboRobot/aubo_ros2_driver
- Scout ROS2 支持：https://github.com/agilexrobotics/scout_ros2
- AgileX UGV SDK：https://github.com/agilexrobotics/ugv_sdk
- ROS2 control：https://github.com/ros-controls/ros2_control
- ROS2 controllers：https://github.com/ros-controls/ros2_controllers
- MoveIt2：https://github.com/moveit/moveit2
- 易爪机器人 MS42DC 产品信息：http://www.yizhuarobot.com/
- MS42DC 厂家 ROS2 源码：`third_party/MS42DC步进电机版柔性机械爪用户资料_V2.2_2024.08.28/5.ROS例程与教程/源码/ROS2.zip`
- 可选 AG95 ROS2 描述和驱动：https://github.com/ian-chuang/dh_ag95_gripper_ros2

第三方模型和运行时资产放在 `third_party/` 下；当它们是 ROS 包时，通过 `src/vendor/` 符号链接暴露给 workspace。仓库只保留可复现运行需要的小型子集，大型手册、视频、安装包和完整素材包由脚本或来源链接下载。

## 第三方运行时来源

- `third_party/aubo_description`：来自 `AuboRobot/aubo_description`，固定到 `47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1`，包内声明 BSD；git 中仅保留 Aubo i5 必要运行子集。
- `third_party/scout_ros2`：来自 `agilexrobotics/scout_ros2`，固定到 `bdbb90471613831fb0b2ec01fecac043445313c4`，根许可证为 Apache-2.0，`scout_description/package.xml` 声明 BSD。
- `third_party/ugv_sdk`：来自 `agilexrobotics/ugv_sdk`，固定到 `c3dfaf444f9bae10757e546acae055aaf4a13de7`，供 `scout_base` 进行 CAN 通信；git 中不保留大型 `docs/` 手册。
- `third_party/aubo_ros2_driver`：来自 `AuboRobot/aubo_ros2_driver`，固定到 `85684075d6ff06c5385e39611208e99ebf0f94c6`，用于 Aubo i5 官方 TCP/IP 与 ros2_control 集成。
- `third_party/dh_ag95_gripper_ros2`：来自 `ian-chuang/dh_ag95_gripper_ros2`，固定到 `fc4f80fdfb3acae5626df4359aec1401cb71a9a3`；`dh_ag95_description/package.xml` 声明 Apache-2.0。
- `third_party/MS42DC.step`：当前易爪机器人 MS42DC 二指柔性伺服电机夹爪模型的本地 CAD 来源。
- `third_party/MS42DC_SPLIT/*.stl`：由项目作者手动拆分的 MS42DC 可动运行时零件，并复制到 `src/arachne_description/meshes/gripper/ms42dc/split/`。
- `third_party/ms42dc_step_motor_ros2`：易爪机器人 MS42DC 厂家 ROS2 示例源码，提供 `serial`、`step_motor` 和演示键盘包；也可由 `scripts/prepare_ms42dc_ros2.sh` 从厂家 ROS2.zip 重新解压刷新。

`src/arachne_description/urdf/` 中的 Arachne 包装文件会把这些模型组合成一棵移动机械臂 URDF 树。`arachne_hardware` 使用官方/厂家 ROS 包作为运行时依赖，而不是复制底层硬件协议。
