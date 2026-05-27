# Arachne Godot Showcase

This is a Godot 4.x high-FPS third-person showcase for Arachne. It is tuned like a playable robot demo: smooth follow camera, proportional gamepad driving, an office-style initial map, collision-aware movement, pushable props, visual suspension, arm presets, gripper animation, and ROS2 bridge placeholders.

Gazebo remains the contact-accurate rehearsal backend. This frontend is the portfolio/gameplay layer, with enough physical feel for driving and obstacle interaction while keeping performance high.

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

On WSL2, `scripts/godot_showcase.sh` automatically uses Mesa D3D12 OpenGL:

```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA ./scripts/godot_showcase.sh
```

This avoids Godot's Vulkan fallback to CPU `llvmpipe`. Native Linux keeps the default renderer unless overridden.

Self-test the scene without opening a window:

```bash
./scripts/test_godot_showcase.sh
```

## Controls

- `W/S` or left-stick Y: proportional forward / backward.
- `A/D` or left-stick X: proportional skid-steer turning.
- `Q/E` or right stick: orbit the follow camera. If a controller reports a nonstandard axis, set `ARACHNE_CAMERA_AXIS=2` or the axis shown in the HUD.
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

If `ROS_DISTRO` is present, the placeholder automatically switches to UDP bridge mode. Override it with:

```bash
ARACHNE_GODOT_BRIDGE=memory ./scripts/godot_showcase.sh
ARACHNE_GODOT_BRIDGE=udp ARACHNE_GODOT_UDP_PORT=8765 ./scripts/godot_showcase.sh
```
