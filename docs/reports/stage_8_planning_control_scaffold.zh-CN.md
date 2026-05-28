# Stage 8：规划与控制骨架

## 目标

在真实硬件到齐前，先准备可以提前开发的部分：MoveIt2、ros2_control、Nav2、安全状态、mock 硬件、自动检查和轻量 operator 面板。

## 核心文件

- `src/arachne_control/`：统一控制器命名、`ros2_controllers.yaml`、sim/mock/real profile、`mock_ros2_control.launch.py`，以及组合启动用的 `prehardware_control.launch.py`。
- `src/arachne_moveit_config/`：MS42DC 和 AG95 的 MoveIt2 起步 SRDF、Aubo 命名姿态、KDL IK、OMPL 规划和控制器映射。
- `src/arachne_nav/`：Nav2 起步参数、空地图，以及带 mock 底盘和 mock `map -> odom` 支持的 `nav2_sim.launch.py`。
- `src/arachne_hardware/arachne_hardware/safety_state_machine.py`：manual/autonomous/disabled/estop 状态服务。
- `src/arachne_hardware/arachne_hardware/safety_cmd_vel_gate.py`：可选 `/cmd_vel` 安全门控路径。
- `src/arachne_hardware/arachne_hardware/hardware_mock.py`：无硬件时发布 odom、joint state 和硬件状态。
- `src/arachne_operator/`：Tk operator 状态面板，以及用于机械臂预设、夹爪命令、demo 序列和 Nav2 目标的 `sequence_executor.py`。
- `scripts/check_workspace.sh`：一条命令完成语法、Xacro、SRDF、构建和 launch smoke check。

## 文件关系

组合 prehardware launch 会启动 mock 硬件、Nav2、MoveIt2、sequence executor 和可选 operator 面板。MoveIt2 和 ros2_control 共享 `arachne_description` 中的 Aubo 与夹爪关节名。Nav2 使用与 RViz、Gazebo 和真实 Scout bringup 相同的 `/cmd_vel` 与 `/odom` 契约；其仿真 launch 会在定位或 SLAM 接入前临时提供 mock `map -> odom` 变换。Operator 面板监听这些共享状态话题，sequence executor 则用带状态、停止、超时和 Nav2 结果处理的方式，把高层任务命令映射到同一套底层契约。

## 下一步

下一轮应在 RViz 中验证 MoveIt2 planning group，调 ros2_control 控制器行为，用真实或仿真 scan 数据确认 Nav2 costmap，并决定真实硬件运动前哪些命令必须经过安全门控。
