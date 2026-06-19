# Aubo 任务执行路径审计

Phase 3C 只审计并迁移“真实 Aubo 关节目标执行”边界，不改变感知、规划、抓取策略、道路巡检或 basket drop-off 语义。

## scripts/vision/grasp_preview_pipeline.py

会发送 Aubo joint target 的函数：

- `_execute_real_sdk_move_joint()`：真实 `sdk_move_joint` 后端入口。它从 `GraspPreview.arm_trajectory_frames` 提取语义 waypoint，并逐个执行 joint target。
- `_execute_real_follow_joint_trajectory()`：真实 `follow_joint_trajectory` 后端入口，通过 ROS action 发送 `FollowJointTrajectory`，不是 SDK JSON-RPC。

直接调用 SDK 的函数：

- `_execute_real_sdk_move_joint()` 旧路径直接调用 `MotionControl.moveJoint`，并配套 Running/Normal、control owner、teach gate、stopJoint、arrival wait。
- `_real_sdk_joint_positions()`、`_real_sdk_require_running()`、`_real_sdk_exit_servo_mode()`、`_real_sdk_stop_joint()` 等是旧 SDK fallback 所需辅助。

保护条件：

- 必须 `--execute-real`。
- 必须 `ARACHNE_CONFIRM_GRASP_EXECUTE_REAL=YES` 或 `--execute-real-confirm YES`。
- start-state check 必须通过。

Phase 3C 处理：

- `sdk_move_joint` 后端现在优先调用 `/arachne/aubo/move_joint` action。
- action server 不可用或 action 路径失败时，若 `--aubo-move-joint-fallback-internal` 为 true，保留旧 guarded SDK fallback。
- `follow_joint_trajectory` 后端暂时不动。

## scripts/vision/grasp_preview_real_sync.sh

会发送 Aubo joint target 的函数：

- 脚本自身不发送 joint target。
- 它同步真实 Aubo 起始姿态，然后启动 `grasp_preview.sh` / `grasp_preview_pipeline.py`。

直接调用 SDK 的函数：

- 内嵌 `AuboJsonRpc` 只读取 `RobotState.getJointPositions`、mode、safety，用于同步起始姿态，不执行 motion。

保护条件：

- `--execute-real` 才会导出真实执行开关。
- 真实执行仍依赖 `ARACHNE_CONFIRM_GRASP_EXECUTE_REAL=YES`。

Phase 3C 处理：

- 补充 action/fallback 环境变量说明。
- 实际 action 接入在 `grasp_preview.sh` 和 `grasp_preview_pipeline.py`。

## src/arachne_operator/arachne_operator/grasp_task_server.py

会发送 Aubo joint target 的函数：

- 该 node 本身不直接发送 joint target。
- `_runner_command()` 启动 `scripts/vision/grasp_preview_real_sync.sh`，真实执行由下游 pipeline 负责。

直接调用 SDK 的函数：

- `_refresh_aubo_joints_from_rpc()` 只读取 `RobotState.getJointPositions` 作为 preflight/状态刷新，不执行 motion。

保护条件：

- `execute_real:=true` 必须配合 `confirm_execute_real:=true`。
- `confirm_execute_real` 会导出 `ARACHNE_CONFIRM_GRASP_EXECUTE_REAL=YES`。
- preflight 检查 control owner、teach gate、joint states、camera/gripper/odom 等要求。

Phase 3C 处理：

- 新增参数 `aubo_move_joint_action_name`、`prefer_aubo_move_joint_action`、`aubo_move_joint_fallback_internal`、`aubo_move_joint_wait_server_sec`。
- 参数通过环境变量传给 runner/pipeline。
- 不改变 preflight、planning、base recovery 或任务状态机语义。

## src/arachne_operator/arachne_operator/road_cleanup_task_server.py

会发送 Aubo joint target 的函数：

- 未发现直接发送 Aubo joint target 的函数。
- 该 node 调用 `/arachne/grasp_task/start`，由 grasp task server 触发抓取执行。

直接调用 SDK 的函数：

- 未发现。

保护条件：

- 继承 grasp task server 的真实执行确认和 preflight。

Phase 3C 处理：

- 不改核心逻辑。
- 后续如 road cleanup 增加直接机械臂执行，再统一接 `AuboMoveJointClient`。

## src/arachne_operator/arachne_operator/real_hardware_acceptance_test.py

会发送 Aubo joint target 的函数：

- `_send_arm_trajectory()` 通过 `/joint_trajectory_controller/follow_joint_trajectory` action 发送验收轨迹。

直接调用 SDK 的函数：

- 未发现直接 Aubo SDK `moveJoint`。

保护条件：

- 验收入口由 `confirm_motion`、`run_arm_test` 等参数控制。

Phase 3C 处理：

- 暂时不动。该文件是验收测试路径，不属于 grasp/road cleanup 任务链路的 SDK moveJoint 迁移范围。

## 标准化结论

- Aubo joint execution 的标准入口是 `/arachne/aubo/move_joint`。
- 任务服务器和 pipeline 不应新增 JSON-RPC motion 调用。
- 旧 guarded SDK 路径本阶段保留为 fallback，未来 Phase 4 再评估收敛/移除。
