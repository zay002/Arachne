# Stage 8：规划与控制骨架

## 目标

在真实硬件到齐前，先准备可以提前开发的部分：MoveIt2、ros2_control、Nav2、安全状态、mock 硬件、自动检查和轻量 operator 面板。

## 核心文件

- `src/arachne_control/`：统一控制器命名、`ros2_controllers.yaml`、sim/mock/real profile，以及 `mock_ros2_control.launch.py`。
- `src/arachne_moveit_config/`：MS42DC 和 AG95 的 MoveIt2 起步 SRDF、Aubo 命名姿态、KDL IK、OMPL 规划和控制器映射。
- `src/arachne_nav/`：Nav2 起步参数、空地图和 `nav2_sim.launch.py`。
- `src/arachne_hardware/arachne_hardware/safety_state_machine.py`：manual/autonomous/disabled/estop 状态服务。
- `src/arachne_hardware/arachne_hardware/safety_cmd_vel_gate.py`：可选 `/cmd_vel` 安全门控路径。
- `src/arachne_hardware/arachne_hardware/hardware_mock.py`：无硬件时发布 odom、joint state 和硬件状态。
- `src/arachne_operator/`：带 safety、stop 和夹爪控制的 Tk operator 状态面板。
- `scripts/check_workspace.sh`：一条命令完成语法、Xacro、SRDF、构建和 launch smoke check。

## 文件关系

Mock 硬件会发布与真机 bringup 相同的高层状态。MoveIt2 和 ros2_control 共享 `arachne_description` 中的 Aubo 与夹爪关节名。Nav2 使用与 RViz、Gazebo 和真实 Scout bringup 相同的 `/cmd_vel` 与 `/odom` 契约。Operator 面板监听这些共享状态话题，因此可用于 mock、仿真或真机会话。

## 下一步

下一轮应在 RViz 中验证 MoveIt2 planning group，调 ros2_control 控制器行为，用真实或仿真 scan 数据确认 Nav2 costmap，并决定真实硬件运动前哪些命令必须经过安全门控。
