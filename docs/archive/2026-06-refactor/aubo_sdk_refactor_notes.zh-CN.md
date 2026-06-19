# Aubo SDK Phase 2 重构记录

本文记录 Phase 2 的最小可行迁移：收敛 Aubo SDK 控制边界，但不改变现有真机 demo 语义。

## 迁移前

- `teach_panel.py` 内部直接包含 `AuboDirectJsonRpc`、`control_owner` 文件读写、teach gate 文件读写、Running/Normal 检查、exit servo、`stopJoint`、`moveJoint`、exec/arrival wait。
- `aubo_tcp_driver.py` 内部也包含一份 `AuboDirectJsonRpc`、`control_owner`、teach gate、teach mode RPC、`speedJoint`/`stopJoint` 辅助逻辑。
- 这些逻辑服务于同一组真机安全门，但分散在 GUI 和 ROS bridge 两侧。

## 迁移后

公共 SDK library 位于 `src/arachne_hardware/arachne_hardware/aubo_sdk/`：

- `client.py`：无 ROS 依赖的 `AuboDirectJsonRpc`。
- `ownership.py`：兼容 `/tmp/arachne_aubo_control_owner` 的 owner claim/release、payload 和 stale pid 检查。
- `teach.py`：兼容 `/tmp/arachne_aubo_teach_mode` 的 teach gate，以及 freedrive/backdrive/handguide RPC。
- `safety.py`：Running/Normal、SafetyMode、`stopJoint`、exit servo、mode polling、exec complete wait、arrival wait。
- `velocity.py`：`speedJoint` helper，保留 busy 后 stop/retry。
- `move_joint.py`：guarded `moveJoint` internal helper。
- `lifecycle.py`：生命周期 mode wait 的导出点。

`teach_panel.py` 仍然是 operator GUI。它的 waypoint、回放、Visual Grasp、Road Cleanup、按钮和参数语义不变；SDK replay 底层改为调用 `execute_move_joint()`。

`aubo_tcp_driver.py` 仍然提供原 ROS executable：`aubo_teach_command_bridge`、`aubo_sdk_velocity_bridge`、`aubo_official_status_probe`。它保留 topic 参数和状态发布，底层 JSON-RPC/owner/gate/safety helper 改为复用 SDK library。

## guarded moveJoint 顺序

`execute_move_joint()` 保持原真机 replay 的安全顺序：

1. 检查 Aubo `Running` 且 SafetyMode 为 `Normal` 或 `ReducedMode`。
2. claim `control_owner`。
3. 设置 teach gate。
4. 退出 servo mode。
5. `stopJoint` pre-cleanup。
6. 调用 `MotionControl.moveJoint`。
7. 等待 execution complete。
8. 等待关节到位并稳定。
9. `stopJoint` post-cleanup。
10. 释放 teach gate 和 `control_owner`。

## 未改变的行为

- 用户入口不变：`teach_panel.sh`、`real_teach_demo.sh`、`grasp_task_server.sh`、`road_cleanup_task_server.sh` 仍按原路径启动。
- ROS topic 名称不变：`/arachne/aubo/teach_command`、`/arachne/aubo/joint_velocity_command`、`/arachne/hardware/aubo_status`、`/joint_states`。
- `real_bringup.launch.py` 默认行为不变。
- `aubo_teach_command_bridge`、`aubo_sdk_velocity_bridge`、`teach_panel` executable 名称不变。
- teach/freedrive、speedJoint jog、moveJoint replay 仍需要 owner/gate/Running safety/stop/watchdog 或 timeout。

## Phase 3 建议

- 将 `moveJoint` 从 internal helper 升级为 `/arachne/aubo/sdk_move_joint` action 更合适；它有执行时间、取消、超时和到位反馈，service 只适合短同步命令。
- demo orchestrator 和 Agent Bridge 应只请求高层 action/service，不直接持有 JSON-RPC client。
- action server 内部继续复用 `execute_move_joint()`，并把 owner/gate/stop/watchdog 保持在同一个执行边界内。

## Phase 3A 结果

新增接口：

- `src/arachne_hardware/action/AuboMoveJoint.action`
- action name：`/arachne/aubo/move_joint`
- executable：`aubo_move_joint_action_server`

Phase 3A 调用链：

1. `teach_panel.py` replay 仍使用 `arm_replay_backend=sdk_move_joint`。
2. `_send_arm_sdk_move_joint()` 先等待 `/arachne/aubo/move_joint` action server。
3. server 可用时，teach panel 发送 `AuboMoveJoint` goal。
4. `aubo_move_joint_action_server.py` 校验 6 关节目标，构造 `MoveJointConfig`。
5. action server 调用 `aubo_sdk.move_joint.execute_move_joint()`。
6. `execute_move_joint()` 继续执行 Phase 2 的 guarded 顺序。
7. action result 返回 `success/message/final_error_rad`；feedback 返回 `state/elapsed_sec/max_error_rad`。
8. 如果 action server 不可用且 `aubo_move_joint_fallback_internal=true`，teach panel 回退到 Phase 2 internal helper。

未改变：

- `/arachne/aubo/teach_command`
- `/arachne/aubo/joint_velocity_command`
- `/arachne/hardware/aubo_status`
- `/joint_states`
- `aubo_teach_command_bridge`
- `aubo_sdk_velocity_bridge`
- `teach_panel.sh`、`real_teach_demo.sh`、`grasp_task_server.sh`、`road_cleanup_task_server.sh`

Phase 3B 建议：

- demo orchestrator 可以把“移动到某个 Aubo joint waypoint”封装成 `AuboMoveJoint` goal，并监听 feedback 做 UI/日志状态。
- grasp task / road cleanup 迁移时应先保留现有 guarded 执行路径作为 fallback，再逐步切到 `/arachne/aubo/move_joint`。
- Agent Bridge 不应直接调用 JSON-RPC；它应请求 action，由 action server 统一处理 owner/gate/stop/cancel。

## Phase 3C 结果

新增 `arachne_operator.aubo_move_joint_client.AuboMoveJointClient`，作为任务链路调用 `/arachne/aubo/move_joint` 的统一客户端 helper。该 helper 不包含 JSON-RPC fallback，调用方决定是否回退旧路径。

迁移范围：

- `grasp_preview_pipeline.py` 的真实 `sdk_move_joint` 后端优先调用 `/arachne/aubo/move_joint` action。
- `grasp_task_server.py` 新增 action/fallback 参数，并通过环境变量传给 runner。
- `road_cleanup_task_server.py` 未改核心逻辑，因为它不直接执行 Aubo joint target，只调用 grasp task server。
- `grasp_preview_real_sync.sh` 保持 CLI 语义不变，仅补充 action/fallback 环境变量说明。

仍保留：

- `ARACHNE_CONFIRM_GRASP_EXECUTE_REAL=YES` / `confirm_execute_real:=true` 保护。
- 旧 guarded SDK JSON-RPC path 作为 fallback。
- `follow_joint_trajectory` 后端和 real hardware acceptance test 暂时不动。

## Phase 4A dry-run 验证

新增 `aubo_move_joint_action_server` 参数：

- `dry_run:=false`：默认值，保持真实 SDK guarded moveJoint 行为。
- `dry_run:=true`：仅模拟 ROS action 生命周期，不连接 Aubo SDK、不调用 JSON-RPC、不写 teach gate、不 claim 真实 control owner。

dry-run 调用链：

1. 上层发送 `/arachne/aubo/move_joint` goal。
2. action server 校验 `target_joints` 长度为 6。
3. server 发布 `accepted`、`checking_state`、`motion_started`、`waiting_arrival`、`completed` feedback。
4. server 返回 `success=true`、`message="dry-run completed"`、`final_error_rad=0.0`。

未改变：

- `dry_run` 默认关闭。
- Phase 2 `execute_move_joint()` 的 Running/Normal、owner、gate、stop/wait/release 安全顺序不变。
- teach panel 和 grasp task 的 action unavailable fallback 仍保留。
- Phase 4A 不发送真实 motion goal，不调用真实 Visual Grasp / Road Cleanup start service。

验证矩阵见 `docs/phase4_validation_matrix.zh-CN.md`。Phase 4B 才进入真实硬件只读检查，只观察状态、`/joint_states`、mode/safety，不发运动。

## Phase 4B 只读检查准备

新增：

- `scripts/hardware/check_aubo_readonly.sh`
- `docs/aubo_readonly_check.zh-CN.md`

Phase 4B 检查链路：

1. 加载 Arachne/ROS 环境。
2. 检查 stale teach gate / control owner 文件，仅提示，不自动删除。
3. ping `AUBO_ROBOT_IP`，默认 `192.168.127.128`。
4. 检查 TCP `30004` 可连接。
5. 调用 `real_aubo_probe.py` 的只读 JSON-RPC，读取 joint positions、tcp pose、RobotMode 和 SafetyMode。
6. 检查 `AuboMoveJoint` action interface。
7. 观察 `/joint_states`、`/arachne/hardware/aubo_status`、`/arachne/aubo/move_joint` 是否存在。

未改变：

- 不发送任何 action goal。
- 不调用真实 `moveJoint` / `speedJoint`。
- 不进入 teach/freedrive/backdrive/handguide。
- 不验证抓取、road cleanup 执行或 speedJoint jog。
- 不关闭 fallback。
- 不修改真机默认安全策略。
