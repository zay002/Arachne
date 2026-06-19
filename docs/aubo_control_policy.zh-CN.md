# Aubo 控制策略

本文记录 Arachne 对 Aubo SDK/ROS 控制的边界和安全门。Phase 2 已把重复
JSON-RPC、control_owner、teach_gate 和部分 motion safety helper 收敛到
`arachne_hardware.aubo_sdk` 公共库；现有 ROS executable 和用户入口保持不变。

## 职责边界

- ROS2 driver 负责 `/joint_states`、RViz、MoveIt2 状态同步，以及官方 ros2_control action/topic 接口。
- SDK bridge 负责需要直接调用 Aubo JSON-RPC/SDK 的动作，包括 freedrive、`speedJoint`、`moveJoint`、`stopJoint`。
- `arachne_hardware.aubo_sdk` 是无 ROS Node 依赖的 SDK library，提供 JSON-RPC client、owner/gate、安全检查和 moveJoint/speedJoint helper。
- `aubo_teach_command_bridge`、`aubo_sdk_velocity_bridge`、`teach_panel` 仍是 ROS/user-facing 层，负责 topic、GUI、状态发布和入口语义。

## 必须保持的安全门

所有 SDK 控制必须经过以下条件：

- `control_owner`：同一时间只有一个高层组件持有 Aubo SDK 控制权。
- `teach_gate`：进入/退出 teach/freedrive 或速度控制时必须尊重 teach flag。
- Running/Normal 检查：执行运动前确认机器人处于可控运行状态，SafetyMode 为 Normal/ReducedMode 等允许状态。
- `stopJoint`：速度控制或异常退出时必须停止关节运动。
- watchdog：速度流、远程启动和执行路径必须有超时/看门狗，避免无人持有时继续运动。

## 当前主要路径

- `/joint_states` 来自 Aubo driver，用于面板、MoveIt、预览同步。
- `/joint_trajectory_controller/follow_joint_trajectory` 是官方 trajectory action，仍作为验收/示教等待接口。
- `/arachne/aubo/teach_command` 由 teach command bridge 转换为 freedrive/teach 控制。
- `/arachne/aubo/joint_velocity_command` 由 SDK velocity bridge 转换为受限 `speedJoint`。
- `/arachne/aubo/move_joint` 是 Phase 3A 新增的 guarded SDK `moveJoint` action，上层 replay/orchestrator 应优先通过该 action 调用，不应直接持有 JSON-RPC client。
- `aubo_move_joint_action_server` 支持 `dry_run:=true` 做 ROS action 链路验证；该模式不连接 SDK、不写 gate/owner、不代表真实机械臂运动成功。默认 `dry_run:=false`。
- `grasp_preview_real_sync.sh --execute-real` 和 `grasp_task_server` 的真实执行路径使用 guarded SDK/moveJoint 语义。
- Phase 3C 起，grasp task 真实 `sdk_move_joint` 路径优先调用 `/arachne/aubo/move_joint`；旧 SDK JSON-RPC 路径只作为 guarded fallback。

## Phase 2 模块结构

SDK library：

- `arachne_hardware/aubo_sdk/client.py`：`AuboDirectJsonRpc`，只负责 connect、close、call、robot_call。
- `arachne_hardware/aubo_sdk/ownership.py`：`/tmp/arachne_aubo_control_owner` 兼容 owner 文件、payload、stale pid 检查、claim/release。
- `arachne_hardware/aubo_sdk/teach.py`：`/tmp/arachne_aubo_teach_mode` 兼容 teach gate，以及 freedrive/backdrive/handguide RPC helper。
- `arachne_hardware/aubo_sdk/safety.py`：Running/Normal 检查、mode polling、`stopJoint`、exit servo、exec/arrival wait。
- `arachne_hardware/aubo_sdk/velocity.py`：`speedJoint` helper，保留 busy 后 stop/retry 语义。
- `arachne_hardware/aubo_sdk/move_joint.py`：guarded `moveJoint` internal helper，按 owner/gate/stop/wait/release 顺序执行。
- `arachne_hardware/aubo_sdk/lifecycle.py`：生命周期 mode wait helper 的导出点。

仍然是 ROS node 或用户入口的文件：

- `src/arachne_hardware/arachne_hardware/aubo_tcp_driver.py`：保留 `aubo_teach_command_bridge`、`aubo_sdk_velocity_bridge`、`aubo_official_status_probe` executable 语义。
- `src/arachne_hardware/arachne_hardware/aubo_move_joint_action_server.py`：Phase 3A 新增 action server，默认 action 名 `/arachne/aubo/move_joint`。
- `src/arachne_operator/arachne_operator/teach_panel.py`：保留 GUI、waypoint、回放、Visual Grasp、Road Cleanup 按钮逻辑；SDK moveJoint replay 优先调用 action，server 不可用时可 fallback 到 internal helper。
- `src/arachne_operator/arachne_operator/aubo_move_joint_client.py`：Phase 3C 新增 action client helper，供任务链路调用 `/arachne/aubo/move_joint`，不包含 JSON-RPC fallback。
- `scripts/operator/teach_panel.sh`、`scripts/hardware/real_teach_demo.sh`、`scripts/vision/grasp_task_server.sh`、`scripts/vision/road_cleanup_task_server.sh`：用户入口不变。

## Phase 4A dry-run 边界

- `real_bringup.launch.py` 新增 `aubo_move_joint_dry_run:=false` 参数，默认关闭。
- `aubo_move_joint_dry_run:=true` 时，action server 仅模拟 `accepted -> checking_state -> motion_started -> waiting_arrival -> completed` feedback，并返回 `success=true`、`message="dry-run completed"`、`final_error_rad=0.0`。
- dry-run 只用于确认 ROS graph、action type、client/server wiring；不得作为真机 moveJoint 成功依据。
- 上层仍应通过 `/arachne/aubo/move_joint` 请求 Aubo joint execution，不应绕过 action server 直接调用 JSON-RPC。
- teach panel、grasp task server 和 grasp preview pipeline 的 fallback 继续保留，等待真机验证后再讨论是否调整默认值。

## Phase 4B 只读硬件检查

- `scripts/hardware/check_aubo_readonly.sh` 只做 ping、TCP 30004、只读 JSON-RPC、ROS interface 和 ROS graph 存在性检查。
- 该脚本不会发送 `/arachne/aubo/move_joint` goal，不会调用 `speedJoint`、`moveJoint`、`freedrive(true)`、`backdrive(true)` 或 `handguideMode`。
- 该脚本只列出 `/tmp/arachne_aubo_teach_mode` 和 `/tmp/arachne_aubo_control_owner` 是否残留，不自动删除、不 claim owner、不写 teach gate。
- Phase 4B 不验证抓取、不验证 speedJoint jog、不验证 teach mode、不关闭 fallback。
- 详细流程见 `docs/aubo_readonly_check.zh-CN.md`。

## Phase 3A 不做的事

- 不删除旧文件或移动 ROS package。
- 不改变 `real_bringup.launch.py` 默认行为。
- 不改变 `aubo_teach_command_bridge` 和 `aubo_sdk_velocity_bridge` 行为。
- 不把 grasp task / road cleanup 迁到新 action。
- 不改变 topic 名称或 demo 语义。

## Phase 3B 重构目标

- 让 road cleanup task server、demo orchestrator、agent bridge 逐步依赖 `/arachne/aubo/move_joint` action。
- 将 control_owner、teach_gate、Running/Normal 检查和 watchdog 做成单一执行门。
- Phase 4 再评估移除 task pipeline 内的旧 SDK fallback。
