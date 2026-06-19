# 硬件验证计划

本文是 Phase 5B 的 release checkpoint。当前真实硬件不可用，因此只归档已完成的离线验证和后续硬件验证顺序。

## 当前已验证

无硬件条件下已验证：

- Python compileall。
- 关键 shell 脚本语法。
- workspace contract check。
- `arachne_hardware` / `arachne_operator` 本地构建。
- `AuboMoveJoint.action` interface 可解析。
- `aubo_move_joint_action_server dry_run:=true` 可启动并接收 mock goal。
- `demo_orchestrator autostart:=false` 可启动，`/status` 和 `/preflight` 可调用。
- `check_aubo_readonly.sh` 只读检查脚本语法通过。

尚未验证：

- 真实 Aubo motion。
- 真实 speedJoint jog。
- 真实 teach/freedrive。
- 真实 Visual Grasp。
- 真实 Road Cleanup。
- 真实 MS42DC 夹取和 Scout 运动。

## Phase 4C 顺序

Phase 4C 只做真实硬件低风险 hold/current-state 检查，顺序必须是：

1. 网络只读：ping Aubo IP，检查 TCP 30004。
2. mode/safety：只读 RobotMode/SafetyMode，确认 Running/Normal 或可接受状态。
3. `/joint_states`：确认 Aubo 当前关节状态持续发布。
4. RViz 姿态同步：只观察模型姿态是否跟随当前关节，不发目标。
5. dry-run action graph：`aubo_move_joint_dry_run:=true` 下确认 `/arachne/aubo/move_joint` 存在。
6. current-state hold goal：仅在人工确认后，发送“当前关节值”作为 hold/current-state 检查，不引入新目标姿态。

## Phase 4C 禁止事项

- 不做新目标运动。
- 不做 visual grasp。
- 不做 road cleanup。
- 不做 speedJoint jog。
- 不做 freedrive teach。
- 不做夹爪闭合测试。
- 不做底盘运动测试。
- 不关闭 fallback。

## Phase 4D

Phase 4D 才考虑低速小幅 motion：

- 目标幅度必须小。
- 速度/加速度必须保守。
- 必须有人工在急停旁。
- 必须先确认 Phase 4C current-state hold 成功。

## Phase 4E

Phase 4E 才考虑视觉抓取真机演示：

- 先跑 camera/YOLO/point cloud/grasp planning 的只读或 preview 链路。
- 再单独验证 Aubo 低速动作。
- 最后才允许 Visual Grasp / Road Cleanup 真机任务。

## 进入硬件阶段前检查

```bash
./scripts/build/check_offline_regression.sh
./scripts/test/smoke_aubo_move_joint_dry_run.sh
./scripts/test/smoke_demo_orchestrator_offline.sh
AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_readonly.sh
```

缺少任一项时，不进入 Phase 4C。
