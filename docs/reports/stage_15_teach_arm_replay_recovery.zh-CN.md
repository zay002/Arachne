# Stage 15：示教机械臂回放恢复

## 目标

让 Aubo 手拖示教后的回放能稳定恢复，并在保持保守的前提下稍微提高 Scout 示教速度。

## 核心文件

- `src/arachne_hardware/arachne_hardware/aubo_tcp_driver.py`：Teach Off 后等待 Aubo 真正退出 freedrive，再清除本地 ros2_control teach gate。
- `third_party/aubo_ros2_driver/aubo_ros2_driver/src/aubo_hardware_interface.cpp`：如果 Aubo servo mode 在示教/预启动后短暂不可用，硬件接口保持当前实测关节并重试，不再返回 `ERROR` 导致 controller 被停用。
- `src/arachne_operator/arachne_operator/teach_panel.py`：新的机械臂 jog 会清除旧的取消状态；回放 action 成功后继续检查关节反馈；底盘示教默认速度提高到手动 `0.08 m/s`、`0.30 rad/s`，回放 `0.04 m/s`、`0.14 rad/s`。
- `scripts/real_bringup.sh` / `scripts/real_aubo_bringup.sh`：启动时清理遗留的 `/tmp/arachne_aubo_teach_mode`，除非显式保留用于调试。

## 文件关系

这次问题不在 waypoint 文件，而在 Aubo 控制器链路：Teach Off 后 servo mode 尚未准备好，ros2_control 把 hardware 和 `joint_trajectory_controller` 停用了，后续 replay goal 都会被拒绝。新流程会保持 controller 存活，并等待/重试 Aubo 重新进入 servo mode。
