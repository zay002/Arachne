# 离线回归与 Smoke Test

本文定义 Phase 5A 的离线回归流程。它用于在没有真实硬件时确认 Aubo action、demo orchestrator、teach panel、grasp task 和 grasp preview 的接口链路没有被重构破坏。

## 边界

允许：

- 编译 Python 模块和 shell 语法。
- 构建 `arachne_hardware` / `arachne_operator`。
- 启动 `aubo_move_joint_action_server dry_run:=true`。
- 仅在 dry-run action server 下发送 mock AuboMoveJoint goal。
- 启动 `demo_orchestrator autostart:=false`。
- 只调用 `/arachne/demo/status` 和 `/arachne/demo/preflight`。

禁止：

- 连接真实 Aubo、Scout、MS42DC 或相机硬件。
- 发送真实 motion goal。
- 在 `dry_run:=false` 下发送 `/arachne/aubo/move_joint` goal。
- 调用 `/arachne/demo/start_visual_grasp` 或 `/arachne/demo/start_road_cleanup`。
- 关闭 fallback。
- 修改 YOLO、点云、抓取规划或 road cleanup 状态机语义。

## 完全离线检查

```bash
./scripts/build/check_offline_regression.sh
```

该脚本执行：

- `python3 -m compileall src/arachne_hardware/arachne_hardware src/arachne_operator/arachne_operator scripts/vision`
- 关键 shell 脚本 `bash -n`
- `./scripts/build/check_workspace.sh`
- 如果本机存在 ROS/colcon，再构建 `arachne_hardware` 和 `arachne_operator`

它不启动真实硬件，不发送 action goal。

## 需要 ROS、但不需要硬件的检查

Dry-run Aubo action smoke：

```bash
./scripts/test/smoke_aubo_move_joint_dry_run.sh
```

该脚本只启动：

```bash
ros2 run arachne_hardware aubo_move_joint_action_server --ros-args -p dry_run:=true
```

随后发送 mock goal：

```text
target_joints: [0,0,0,0,0,0]
speed_rad_sec: 0.1
accel_rad_sec2: 0.1
timeout_sec: 3.0
label: dry_run_smoke_test
```

期望 result 包含：

- `success=true`
- `dry-run completed`

该脚本不启动真实 Aubo driver，不连接 Aubo IP。

Demo orchestrator offline smoke：

```bash
./scripts/test/smoke_demo_orchestrator_offline.sh
```

该脚本启动：

```bash
ros2 launch arachne_operator demo_orchestrator.launch.py autostart:=false
```

只调用：

- `/arachne/demo/status`
- `/arachne/demo/preflight`

并检查 `/arachne/demo/state` topic 存在。离线状态下 preflight 可以返回 `success=false`，但 response 必须包含 checks payload，服务不能崩溃。

## 只允许 Dry-run 的命令

以下命令只允许在 dry-run action server 下运行：

```bash
ros2 action send_goal /arachne/aubo/move_joint arachne_hardware/action/AuboMoveJoint "{target_joints: [0,0,0,0,0,0], speed_rad_sec: 0.1, accel_rad_sec2: 0.1, blend_radius: 0.0, duration_sec: 0.0, goal_tolerance_rad: 0.04, timeout_sec: 3.0, label: 'dry_run_smoke_test'}"
```

如果 action server 不是 `dry_run:=true`，禁止发送。

## 严禁无准备运行

```bash
ros2 action send_goal /arachne/aubo/move_joint ...
ros2 service call /arachne/demo/start_visual_grasp ...
ros2 service call /arachne/demo/start_road_cleanup ...
./scripts/hardware/real_full_acceptance.sh --yes
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/hardware/real_hardware_acceptance_test.sh
```

## 每次提交前推荐顺序

```bash
./scripts/build/check_offline_regression.sh
./scripts/test/smoke_aubo_move_joint_dry_run.sh
./scripts/test/smoke_demo_orchestrator_offline.sh
```

如果没有 ROS 环境，只运行 `check_offline_regression.sh` 中能完成的静态部分；如果有 ROS 环境但无硬件，可以运行两个 smoke tests。

## 与硬件阶段的关系

- 没硬件时：运行 offline regression。
- 有硬件但不允许运动时：运行 `docs/aubo_readonly_check.zh-CN.md` 中的 readonly check。
- 只有 readonly check 通过后，才考虑 Phase 4C 的低风险 hold/current-state 检查。
