# 2026-06 重构总览

本文归档 Phase 1 到 Phase 5A 的 Arachne 仓库整理、Aubo 控制边界收敛和离线回归固化工作。当前没有真实硬件接入，真机运动尚未验证。

## 重构前的问题

- 脚本入口、launch、ROS executable 和文档分散，真机/仿真/兼容 wrapper 边界不清。
- Aubo JSON-RPC client、`control_owner`、`teach_gate`、Running/Normal 检查、`stopJoint`、`moveJoint` 等逻辑散落在 `teach_panel.py` 和 `aubo_tcp_driver.py`。
- GUI replay、grasp pipeline、demo orchestration 直接或间接依赖底层 SDK 细节，后续 Agent / task server 接入风险较高。
- 缺少不连接硬件也能重复执行的 action/dry-run/orchestrator 回归检查。

## 阶段结果

Phase 1：入口统一和文档整理

- 审计 scripts、launch、ROS executable、topic/service/action、真机/仿真入口、deprecated wrapper。
- 给脚本按 `primary/helper/deprecated/experimental` 标注。
- 新增入口和安全标注文档，更新 `scripts/README.md`。

Phase 2：Aubo SDK 公共模块抽取

- 新增 `arachne_hardware.aubo_sdk` 公共层：
  - `client.py`
  - `ownership.py`
  - `teach.py`
  - `safety.py`
  - `velocity.py`
  - `move_joint.py`
  - `lifecycle.py`
- `teach_panel.py` 和 `aubo_tcp_driver.py` 改为复用公共 SDK helper。
- 真机语义和用户入口保持不变。

Phase 3A：AuboMoveJoint action 化

- 新增 `AuboMoveJoint.action`。
- 新增 `/arachne/aubo/move_joint` action server。
- teach panel replay 优先调用 action，action 不可用时保留 internal fallback。

Phase 3B：demo orchestrator 骨架化

- 新增 `demo_orchestrator`，提供 `/arachne/demo/*` 轻量 Trigger services。
- teach panel 优先调用 orchestrator，orchestrator 不可用时保留旧 GUI 内部编排 fallback。

Phase 3C：grasp pipeline 接入 AuboMoveJoint action

- 新增 `AuboMoveJointClient` helper。
- grasp task / grasp preview 的真实 `sdk_move_joint` 路径优先调用 `/arachne/aubo/move_joint`。
- 旧 guarded SDK path 继续作为 fallback。

Phase 4A：Aubo action dry-run 验证栈

- `aubo_move_joint_action_server` 新增 `dry_run:=false` 参数，默认关闭。
- `dry_run:=true` 时只模拟 action feedback/result，不连接 SDK、不写 gate/owner。
- 新增 Aubo action stack 静态检查和验证矩阵。

Phase 4B：真实硬件只读检查准备

- 新增 `check_aubo_readonly.sh`。
- 新增 Aubo 只读检查文档。
- 只读流程覆盖网络、TCP 30004、RobotMode/SafetyMode、ROS graph、`/joint_states`、action server 存在性。

Phase 5A：离线回归固化

- 新增 `check_offline_regression.sh`。
- 新增 dry-run AuboMoveJoint action smoke test。
- 新增 demo orchestrator offline status/preflight smoke test。
- 新增离线回归文档。

## 当前推荐主入口

- 无硬件提交前：`./scripts/build/check_offline_regression.sh`
- Aubo action dry-run smoke：`./scripts/test/smoke_aubo_move_joint_dry_run.sh`
- Demo orchestrator offline smoke：`./scripts/test/smoke_demo_orchestrator_offline.sh`
- 有硬件但不允许运动：`AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_readonly.sh`
- 真机底层启动：`./scripts/hardware/real_bringup.sh`
- 真机示教 GUI：`./scripts/operator/teach_panel.sh`
- grasp task：`./scripts/vision/grasp_task_server.sh`
- road cleanup task：`./scripts/vision/road_cleanup_task_server.sh`

## 当前 Aubo 控制架构

```text
teach_panel / demo_orchestrator / grasp_task
        ↓
/arachne/aubo/move_joint
        ↓
aubo_move_joint_action_server
        ↓
aubo_sdk.move_joint.execute_move_joint()
        ↓
Aubo SDK
```

`execute_move_joint()` 仍负责 Running/Normal、`control_owner`、`teach_gate`、exit servo、pre/post `stopJoint`、execution complete wait、arrival wait 和 release。

## 当前 Demo Orchestrator 架构

- `demo_orchestrator` 只做编排，不直接执行底层 Aubo motion。
- `/arachne/demo/status` 和 `/arachne/demo/preflight` 可用于离线/只读检查。
- `/arachne/demo/start_visual_grasp` 和 `/arachne/demo/start_road_cleanup` 保留为真实 demo 编排入口，Phase 5B 不调用。
- teach panel 保留 orchestrator unavailable fallback。

## 当前 Fallback 策略

- teach panel action unavailable 时保留 Phase 2 internal SDK fallback。
- grasp pipeline action unavailable 时保留旧 guarded SDK path fallback。
- demo orchestrator unavailable 时 teach panel 保留原内部编排 fallback。
- Phase 5B 不关闭 fallback，也不改变默认值。

## 当前不能验证的部分

- 真实 Aubo `moveJoint` 运动。
- speedJoint jog。
- freedrive/backdrive/handguide teach mode。
- Visual Grasp 真机闭环。
- Road Cleanup 真机闭环。
- MS42DC 真实夹取和 Scout 真实运动。

## 硬件可用后的下一步

1. 运行 offline regression。
2. 连接 Aubo 后运行 readonly check。
3. 确认 RobotMode/SafetyMode、`/joint_states`、`/arachne/hardware/aubo_status`。
4. 启动 dry-run action graph，仍不发送真实 goal。
5. Phase 4C 才进入 current-state hold 级别检查。
6. Phase 4D 才考虑低速小幅 motion。
7. Phase 4E 才考虑视觉抓取真机演示。

## 禁止直接运行

```bash
ros2 action send_goal /arachne/aubo/move_joint ...
ros2 service call /arachne/demo/start_visual_grasp ...
ros2 service call /arachne/demo/start_road_cleanup ...
./scripts/hardware/real_full_acceptance.sh --yes
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/hardware/real_hardware_acceptance_test.sh
```

## 给下一个 Agent 的注意事项

- 不要把 dry-run 成功解读为真实 Aubo 运动成功。
- 不要删除 fallback；先完成硬件只读和 current-state hold 验证。
- 不要绕过 `/arachne/aubo/move_joint` 直接从上层调用 JSON-RPC。
- 不要修改 YOLO、点云、抓取规划或 road cleanup 状态机来“适配”离线测试。
- 仿真应贴近真实流程，不要添加绕过真实操作顺序的 sim-only shortcut。
