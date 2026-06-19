# 开发与提交前检查流程

本文记录 Phase 5B 后推荐的本地开发流程。当前没有真实硬件接入时，不要进入 Phase 4C。

## 每次提交前

推荐依次运行：

```bash
./scripts/build/check_offline_regression.sh
./scripts/test/smoke_aubo_move_joint_dry_run.sh
./scripts/test/smoke_demo_orchestrator_offline.sh
```

这三条命令覆盖：

- Python compileall。
- 关键 shell 脚本语法。
- workspace contract check。
- `arachne_hardware` / `arachne_operator` 本地构建。
- `/arachne/aubo/move_joint` dry-run action server 和 mock goal。
- `demo_orchestrator` offline status/preflight。

## 没硬件时不要运行

```bash
ros2 action send_goal /arachne/aubo/move_joint ...
ros2 service call /arachne/demo/start_visual_grasp ...
ros2 service call /arachne/demo/start_road_cleanup ...
./scripts/hardware/real_bringup.sh
./scripts/hardware/real_teach_demo.sh
./scripts/hardware/real_full_acceptance.sh --yes
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/hardware/real_hardware_acceptance_test.sh
```

例外：`smoke_aubo_move_joint_dry_run.sh` 可以发送 dry-run mock goal，因为它自己启动 `dry_run:=true` action server，不连接 Aubo IP。

## 有硬件但不允许运动

运行只读检查：

```bash
AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_readonly.sh
```

该脚本只做网络、TCP 30004、只读 RobotState JSON-RPC、ROS interface 和 graph 存在性检查。它不会发送 action goal，不会写 teach gate，不会 claim control owner。

只有 readonly check 通过后，才允许进入 Phase 4C。

## 真机执行确认

任何真机执行必须由人工确认安全空间、急停、控制柜状态和脚本安全变量。常见确认包括：

- `ARACHNE_CONFIRM_REAL_MOTION=YES`
- `ARACHNE_CONFIRM_GRASP_EXECUTE_REAL=YES`
- `execute_real:=true confirm_execute_real:=true`
- `ARACHNE_CONFIRM_AUBO_DRIVER=YES`
- `ARACHNE_CONFIRM_AUBO_REMOTE_START=YES`

Phase 5B 不改变这些默认策略。

## CI 边界

GitHub Actions 只做无 ROS 依赖的静态检查。完整 ROS build、action smoke 和 orchestrator smoke 仍以本地检查为准。
