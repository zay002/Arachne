# Arachne

Arachne 是一个 ROS 2 移动操作机器人 workspace，当前主线服务于 Scout 2.0 底盘、Aubo i5 机械臂、MS42DC 夹具、Gemini335 RGB-D 相机和 C16 雷达的真机上位机开发。

当前成熟链路：

- Aubo 可进入 `Running / Normal`，`/joint_states` 正常。
- `/arachne/aubo/move_joint` action 已接入 SDK `moveJoint`，保留 fallback。
- Teach Panel 可控制 Aubo、底盘和夹具。
- `grasp_task_server`、`road_cleanup_task_server` 和 `demo_orchestrator` 保留为上层任务入口。
- offline regression、Aubo dry-run smoke、demo orchestrator offline smoke 可用。

## 快速启动

```bash
source scripts/env/arachne_env.sh
./scripts/build/build_workspace.sh
./scripts/model/view_model.sh
```

真机入口：

```bash
./scripts/hardware/real_bringup.sh
./scripts/operator/teach_panel.sh
```

任务服务：

```bash
./scripts/vision/grasp_task_server.sh
./scripts/vision/road_cleanup_task_server.sh
```

## 开发检查

```bash
./scripts/build/check_offline_regression.sh
./scripts/test/smoke_aubo_move_joint_dry_run.sh
./scripts/test/smoke_demo_orchestrator_offline.sh
```

真机只读检查：

```bash
AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_readonly.sh
AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_running_readonly.sh
```

## 安全边界

任何真实运动都需要现场确认急停、空间、线缆和人员安全。不要把 dry-run、只读检查或 action server 存在等同于真实抓取已安全。

禁止在未确认时运行：

- `/arachne/aubo/move_joint` 真实 goal
- `speedJoint` jog
- freedrive / backdrive / handguide
- Visual Grasp 或 Road Cleanup 真机任务
- `grasp_preview_real_sync.sh --execute-real`

## 文档索引

- [架构](docs/architecture.zh-CN.md)
- [硬件](docs/hardware.zh-CN.md)
- [Aubo 控制](docs/aubo_control.zh-CN.md)
- [任务服务](docs/tasks.zh-CN.md)
- [开发流程](docs/development.zh-CN.md)
- [标定与导航](docs/calibration_nav.zh-CN.md)
- [排障](docs/troubleshooting.zh-CN.md)
- [脚本入口](scripts/README.md)

阶段性重构和验证记录已归档到 `docs/archive/2026-06-refactor/`。
