# Arachne Godot Showcase

This is a Godot 4.x high-FPS visualization frontend for Arachne. It is meant for portfolio demos and interactive teleoperation feel, not as a replacement for Gazebo physics.

## Run

From the repository root:

```bash
./scripts/install_godot4.sh   # optional if godot4 is already installed
./scripts/fetch_third_party.sh
./scripts/godot_showcase.sh
```

If Godot is installed under another name:

```bash
GODOT_BIN=/path/to/Godot_v4.x ./scripts/godot_showcase.sh
```

The launcher creates local links under `assets/vendor/` and generated GLB cache files under `assets/generated/` so Godot can import the existing Scout, Aubo i5, MS42DC, AG95, and prop meshes without copying bulky assets into this project.

## Controls

- `W/S` or left-stick Y: forward / backward.
- `A/D` or left-stick X: turn left / right.
- `Q/E` or right-stick X: orbit the follow camera.
- `1` to `5`: arm presets `home`, `ready`, `reach`, `grasp`, `lift`.
- `Space`: toggle gripper.
- `R`: reset base, camera, arm, and gripper.

## ROS 2 Bridge Placeholder

`scripts/ros2_bridge_placeholder.gd` defines placeholder publishing methods for:

- `/cmd_vel`
- `/joint_states`
- `/odom`
- `/tf`

The current implementation stores and emits these messages inside Godot only. It is intentionally dependency-free so the showcase runs smoothly on native Linux, WSL2, or a workstation without ROS sourced. A later bridge can replace this file with WebSocket, UDP, or native Godot ROS 2 bindings while keeping the frontend scene contract stable.
