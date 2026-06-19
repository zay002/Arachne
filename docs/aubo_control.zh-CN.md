# Aubo 控制

Aubo 当前采用“双通道”策略：

- ROS2 driver：连接控制柜，发布 `/joint_states` 和硬件状态。
- Aubo SDK：执行真实 `moveJoint`、`speedJoint`、stop/hold 等动作。

## `/arachne/aubo/move_joint`

`/arachne/aubo/move_joint` 是真机上层任务的首选机械臂关节目标入口。它支持 dry-run，也支持真实 SDK 执行。

真实 goal 只能在现场安全确认后发送。dry-run 成功只说明 ROS action graph 可用，不说明真实 motion 已验证。

## Teach / speedJoint / moveJoint

- Teach Panel jog 使用受控 velocity bridge。
- `moveJoint` 用于离散目标姿态。
- `speedJoint` 只用于受 watchdog/gate 保护的示教 jog。
- freedrive/backdrive/handguide 不作为默认自动流程。

## Fallback

保留 fallback 是当前安全策略的一部分。上层优先 action，action 不可用时才走既有 guarded SDK 路径；不要为了“清爽”删掉 fallback。
