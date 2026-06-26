# AGENTS.md

Rules for future agents working in this repo.

## Project Structure

- `src/arachne_operator`: teach panel, task servers, demos, operator CLI.
- `src/arachne_hardware`: real Aubo/base/gripper drivers and actions.
- `src/arachne_sensors`: Gemini camera bringup and standalone sensor debug tools.
- `src/arachne_description`: URDF, meshes, RViz assets.
- `src/arachne_nav`: lidar/nav launch and config.
- `src/arachne_moveit_config`: MoveIt config.
- `scripts`: setup/bootstrap/field helpers only. Do not create new stable runtime wrappers here.
- `docs/demo`: keep only useful demo/promo images.
- `log`, `build`, `install`: generated; do not commit.

## Run Commands

Normal full operator entrypoint:

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
source install/setup.bash
ros2 launch arachne_operator teach_panel.launch.py
```

Build after code or launch changes:

```bash
./scripts/build/build_workspace.sh
source install/setup.bash
```

Standalone depth pointcloud check, never part of teach startup:

```bash
ros2 launch arachne_sensors depth_to_pointcloud.launch.py
```

## Test Commands

Run the smallest check that covers the change.

```bash
python3 -m py_compile <changed-python-files>
ros2 run arachne_operator teach_panel --headless-check
ros2 run arachne_operator grasp_task_server --dry-run-check
ros2 run arachne_operator road_cleanup_task_server --dry-run-check
ros2 run arachne_operator step_cleanup_demo --dry-run-check
ros2 run arachne_sensors depth_to_pointcloud --dry-run-check
ros2 run arachne_operator arachne check entrypoints
```

Use full build when entrypoints, launch files, package data, or imports changed:

```bash
./scripts/build/build_workspace.sh
```

Do not run long real-hardware tests unless the user explicitly asks and confirms the robot is ready.

## Code Style

- Prefer the smallest working change. Delete or reuse before adding.
- Use existing ROS package entrypoints and launch patterns.
- Use `pathlib`, package share paths, or declared parameters; do not hardcode repo-relative paths except documented workspace defaults.
- Keep hardware calibration values as parameters.
- Keep task logic readable: one node owns one policy; do not hide motion decisions behind generic frameworks.
- Use `rg` for searches.
- Use `apply_patch` for manual edits.
- Comments only for non-obvious hardware/safety behavior.

## Prohibited

- Do not commit secrets, IP credentials, `.env.local`, logs, build artifacts, or generated caches.
- Do not add shell wrappers as stable runtime entrypoints.
- Do not make teach panel depend on debug tools such as `depth_to_pointcloud`.
- Do not put real robot motion on a default path without explicit enable/confirmation.
- Do not bypass Aubo control ownership / teach-mode gating.
- Do not revert user changes unless explicitly asked.
- Do not add dependencies for simple stdlib/ROS-native work.
- Do not run destructive git commands.

## Completion Standard

A change is done only when:

- The requested behavior is implemented in the smallest relevant surface.
- Normal operator entrypoint still remains `teach_panel.launch.py`.
- Dry-run/headless paths do not touch real hardware.
- Real motion paths still require explicit confirmation and safety gates.
- Relevant minimal checks passed, or the reason they were skipped is stated.
- `git status --short` has been inspected.

## Review Standard

Reviews must lead with concrete risks:

- Safety regressions: accidental motion, missing stop path, skipped ownership, unsafe defaults.
- Hardware regressions: wrong frame, stale calibration, blocking driver startup, command races.
- Operator regressions: teach panel waiting states, hidden startup dependency, duplicated publishers/services.
- Planning/grasp regressions: unreachable pose defaults, excessive constraints, unbounded retries, no recovery.
- Packaging regressions: missing `console_scripts`, missing launch install, entrypoint depends on current working directory.

If no issue is found, say so and list remaining untested hardware risk.
