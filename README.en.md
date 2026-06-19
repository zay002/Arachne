# Arachne

Arachne is a ROS 2 workspace for a real mobile manipulator: Scout 2.0 base, Aubo i5 arm, MS42DC gripper, Gemini335 RGB-D camera, and C16 lidar.

Current stable path:

- Aubo reaches `Running / Normal`; `/joint_states` is available.
- `/arachne/aubo/move_joint` action drives guarded SDK `moveJoint`; fallback remains enabled.
- Teach Panel controls Aubo, base, and gripper.
- `grasp_task_server`, `road_cleanup_task_server`, and `demo_orchestrator` are the upper-layer task entries.
- Offline regression, Aubo dry-run smoke, and demo orchestrator offline smoke are available.

## Quick Start

```bash
source scripts/env/arachne_env.sh
./scripts/build/build_workspace.sh
./scripts/model/view_model.sh
```

Real hardware:

```bash
./scripts/hardware/real_bringup.sh
./scripts/operator/teach_panel.sh
```

Task servers:

```bash
./scripts/vision/grasp_task_server.sh
./scripts/vision/road_cleanup_task_server.sh
```

## Development Checks

```bash
./scripts/build/check_offline_regression.sh
./scripts/test/smoke_aubo_move_joint_dry_run.sh
./scripts/test/smoke_demo_orchestrator_offline.sh
```

Read-only Aubo checks:

```bash
AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_readonly.sh
AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_running_readonly.sh
```

## Safety

Any real motion requires operator confirmation of emergency stop, clearance, cables, and people around the robot. A dry-run, read-only check, or visible action server does not prove real grasp safety.

Do not run without explicit confirmation:

- real `/arachne/aubo/move_joint` goals
- `speedJoint` jog
- freedrive / backdrive / handguide
- real Visual Grasp or Road Cleanup
- `grasp_preview_real_sync.sh --execute-real`

## Docs

- [Architecture](docs/architecture.zh-CN.md)
- [Hardware](docs/hardware.zh-CN.md)
- [Aubo Control](docs/aubo_control.zh-CN.md)
- [Tasks](docs/tasks.zh-CN.md)
- [Development](docs/development.zh-CN.md)
- [Calibration/Nav](docs/calibration_nav.zh-CN.md)
- [Troubleshooting](docs/troubleshooting.zh-CN.md)
- [Scripts](scripts/README.md)

Phase notes are archived under `docs/archive/2026-06-refactor/`.
