# 排障

## Aubo

- `RobotMode=PowerOff`：只读正常但不能运动；需要现场或 remote start。
- `SafetyMode!=Normal`：停止，不进入 hold 或 motion。
- TCP `30004` 不通：检查 IP、网线、控制柜网络。
- `/joint_states` 缺失：检查 `real_bringup.sh`、controller、Aubo driver。
- `/arachne/aubo/move_joint` action 缺失：检查 `aubo_move_joint_action_server` 是否随 bringup 启动。

## Gate / Owner

只读脚本只报告 `/tmp/arachne_aubo_teach_mode` 和 `/tmp/arachne_aubo_control_owner`，不自动删除。删除前需要人工确认当前没有任务占用机械臂。

## rcl shutdown

如果退出时报 `rcl_shutdown already called`，只修 shutdown/destroy 重复调用，不改 action 行为。

## Base / Gripper waiting

- 底盘看 `/arachne/hardware/base_status`、`/odom`、`/cmd_vel` 订阅。
- 夹具看 `/arachne/hardware/gripper_status` 和 `/arachne/gripper/command`。
- 如果 topic 有 publisher 但 UI waiting，优先查状态映射，不要先改硬件 driver。
