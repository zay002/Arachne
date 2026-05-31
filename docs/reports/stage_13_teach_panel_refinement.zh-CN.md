# Stage 13：示教面板增强

## 目标

让真机示教面板更适合演示编排：能手动进入/退出 Aubo 示教模式，能复用已有 waypoint，并能微调末端姿态。

## 核心文件

- `src/arachne_operator/arachne_operator/teach_panel.py`：新增 Aubo `Teach On/Off`、RX/RY/RZ 腕部姿态 jog、底盘长按连续遥控与相对移动段记录，以及 `Duplicate` 路点复用。
- `src/arachne_operator/launch/teach_panel.launch.py`：暴露示教命令 topic 和姿态 jog 参数。
- `src/arachne_hardware/arachne_hardware/aubo_tcp_driver.py`：新增 `aubo_teach_command_bridge`，把 `/arachne/aubo/teach_command` 转成 Aubo 30004 JSON-RPC 的 `RobotManage.freedrive(true/false)` 调用。
- `src/arachne_hardware/arachne_hardware/hardware_mock.py`：mock 接收同一示教命令并在状态中显示 teach on/off。
- `scripts/fetch_third_party.sh`：为固定版本 Aubo driver 增加示教门控补丁，避免 ros2_control 在手拖时继续 `servoJoint` 保持。

## 文件关系

示教 UI 只发布统一 ROS 命令：底盘走 `/cmd_vel`，夹具走 `/arachne/gripper/command`，Aubo 示教模式走 `/arachne/aubo/teach_command`，机械臂运动仍走 trajectory action/topic。真机 bringup 中的 bridge 负责把 Arachne 命令适配到 Aubo JSON-RPC；mock bringup 使用同一 topic 做无硬件验证。底盘手动控制在按下和松开之间记录成相对移动 waypoint，例如前进/后退距离或左/右转角，并从当前 odom 状态回放。

## 说明

RX/RY/RZ 当前是保守的腕部关节增量控制，不是完整 6D 姿态 IK。它适合示教时小范围微调；复杂姿态规划仍应交给 MoveIt2 或后续完整末端位姿 IK。
