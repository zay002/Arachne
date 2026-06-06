<p align="center">
  <img src="docs/demo/arachne.png" alt="Arachne robot system showcase" width="900">
</p>

# Arachne

[中文](README.md) · [Quick Start](#quick-start) · [Real Hardware](#real-hardware) · [Documentation](#documentation)

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20%7C%20Humble-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04%20%7C%2022.04-orange)
![License](https://img.shields.io/badge/license-MIT-green)

Arachne is a ROS 2 workspace for a mobile manipulation robot aimed at deep-reinforcement-learning joint control. The default hardware stack combines a Scout 2.0 mobile base, an Aubo i5 arm, an MS42DC two-finger flexible servo gripper, a Gemini335 RGB-D camera, a Leishen Intelligence C16 lidar, a front basket, and a rear sensor rack; AG95 remains available as an interchangeable gripper model.

The long-term goal is a real mobile manipulator that can perform precision assembly, measurement, and mobile manipulation tasks. The base, arm, gripper, vision sensors, and lidar share a common state representation, while classical control, teach data, visual perception, and deep RL policies are fused under explicit safety constraints. The current `jetson` route focuses on real-hardware streaming control, perception capture, teach-panel workflows, digital-twin validation, and edge inference. Early task tracks include trash recognition and basket placement, plus charging-gun recognition, alignment, removal, and insertion.

<table>
  <tr>
    <td width="50%" align="center"><img src="docs/demo/realbot_1.jpg" alt="Arachne physical robot front view" width="100%"></td>
    <td width="50%" align="center"><img src="docs/demo/realbot_2.jpg" alt="Arachne physical robot side view" width="100%"></td>
  </tr>
</table>

## Features

- Unified digital twin for Scout 2.0, Aubo i5, MS42DC, AG95, Gemini335, Leishen Intelligence C16, rear rack, and front basket in one TF/URDF tree.
- Real-hardware streaming control for base hold-to-drive, Aubo SDK velocity control, MS42DC serial control, Aubo remote power/startup, and payload setup.
- Perception and edge inference with Gemini335 RGB-D capture, Leishen Intelligence C16 environment sensing, a YOLO26/TensorRT workspace, live annotated preview, and INT8 calibration directories.
- Teach and data loop with a window-friendly operator panel, home/install poses, local configuration, hold-to-jog controls, recording, and replay.
- Simulation and visualization through RViz checks, Gazebo validation, a Godot high-FPS showcase, and future reinforcement-learning simulation interfaces.
- Control scaffold for MoveIt2, Nav2, ros2_control, sequence executor, VLA/WAM action-chunk translation, and future DRL policy nodes.

## Quick Start

Supported environments are Ubuntu 24.04 + ROS 2 Jazzy and Ubuntu 22.04 + ROS 2 Humble.

```bash
git clone https://github.com/zay002/Arachne.git
cd Arachne

./scripts/build/setup_ubuntu.sh
./scripts/hardware/fetch_third_party.sh

source scripts/env/arachne_env.sh
./scripts/build/build_workspace.sh
./scripts/model/view_model.sh
```

The repository vendors a small runnable third-party subset: required Aubo i5 meshes, Scout ROS2, UGV SDK source, the Aubo ROS2 driver, AG95 description, and MS42DC ROS2 examples. Bulky material such as the full Aubo model collection, vendor videos/installers, large UGV PDFs, and external Godot asset packs remains script- or link-downloaded. `fetch_third_party.sh` reuses the vendored content by default and creates the needed symlinks; to refresh full pinned upstream checkouts, run `ARACHNE_REFRESH_THIRD_PARTY=true ./scripts/hardware/fetch_third_party.sh`.

`arachne_env.sh` pins the current shell to the Ubuntu system Python used by ROS, such as `/usr/bin/python3.12` on Ubuntu 24.04 + Jazzy, so conda/pyenv Python 3.13 cannot hijack ROS Python modules.

`view_model.sh` starts the default MS42DC model, base teleop GUI, Aubo joint sliders, and gripper Open/Close controls.

Prefer `./scripts/model/view_model.sh` for model inspection; the script loads the ROS and workspace environment automatically. If you run `ros2 launch` manually or open RViz directly, run:

```bash
source scripts/env/arachne_env.sh
source install/setup.bash
```

Without that environment, RViz may fail to resolve `package://...` mesh paths, causing white meshes, stacked parts, or missing materials.

## Common Commands

| Goal | Command |
| --- | --- |
| View the default MS42DC model | `./scripts/model/view_model.sh` |
| View the AG95 model | `./scripts/model/use_gripper.sh ag95 view` |
| Check URDF and core interfaces | `./scripts/build/check_workspace.sh` |
| Gazebo gamepad demo | `./scripts/sim/switch_demo.sh` |
| Gazebo autonomous pick validation | `./scripts/sim/gazebo_autopick_demo.sh` |
| Godot showcase | `./scripts/godot/godot_showcase.sh` |
| Gemini335 YOLO live preview | `./scripts/vision/gemini_yolo_live.sh` |
| Bottle grasp-to-basket path preview | `./scripts/vision/grasp_preview.sh` |
| Real-pose synchronized grasp preview | `./scripts/vision/grasp_preview_real_sync.sh` |
| Real-pose synchronized grasp execution | `ARACHNE_CONFIRM_GRASP_EXECUTE_REAL=YES ./scripts/vision/grasp_preview_real_sync.sh --execute-real` |
| Grasp task server | `./scripts/vision/grasp_task_server.sh` |
| Agent Bridge | `./scripts/agent/agent_bridge.sh` |
| Real-hardware environment check | `./scripts/hardware/check_real_hardware_env.sh` |
| Real one-command bringup | `./scripts/hardware/real_bringup.sh` |
| Real teach demo | `./scripts/hardware/real_teach_demo.sh` |
| Read-only Aubo connectivity probe | `./scripts/hardware/real_aubo_probe.sh` |
| Aubo startup state check | `./scripts/hardware/real_aubo_prepare.sh` |
| Aubo real driver bringup | `ARACHNE_CONFIRM_AUBO_DRIVER=YES ./scripts/hardware/real_aubo_bringup.sh` |
| Blocking Aubo remote startup | `ARACHNE_CONFIRM_AUBO_REMOTE_START=YES ./scripts/hardware/real_aubo_remote_start.sh` |
| Small Aubo Z test | `ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/hardware/real_aubo_z_test.sh` |
| Real teach and replay panel | `./scripts/operator/teach_panel.sh` |

<p align="center">
  <img src="docs/demo/gazebo.png" alt="Arachne Gazebo demo" width="48%">
  <img src="docs/demo/godot.png" alt="Arachne Godot showcase" width="48%">
</p>

## Real Hardware

Arachne keeps the real-hardware layer ROS-facing and uses official or vendor routes where they fit the deployed hardware.

### Coordinate Frames

Arachne follows the normal ROS mobile-base convention: `base_link` has +X toward the front of the vehicle, +Y toward the vehicle left, and +Z upward. `odom -> base_link` comes from base odometry, while `map -> odom` belongs to localization and is intentionally outside the URDF. The arm chain is mounted on the vehicle as `base_link -> arm_mount_link -> aubo_base_link -> ... -> tool0 -> gripper_adapter_link -> grasp_frame`; `aubo_base_link` is the Aubo base, `tool0` is the flange center, and `grasp_frame` is the gripper-center grasp TCP. The end-effector RGB-D camera is mounted under `tool0`; depth ROI points are first projected in the camera depth frame, then transformed into `base_link` before grasp planning. The grasp-preview correction `ARACHNE_GRASP_BASE_OFFSET` is a meter-scale `(x,y,z)` offset in `base_link`; the current measured default is `0.06,0.09,-0.10`, meaning 6 cm forward, 9 cm left, and 10 cm downward. Real grasp execution returns to the home pose from `scripts/env/arachne_real_defaults.sh` after opening over the basket by default.

| Device | Default interface | Notes |
| --- | --- | --- |
| Scout 2.0 | `scout_waveshare_serial_driver` | `/cmd_vel` to Scout v2 CAN frames through Waveshare USB-CAN-A, CH340 serial, default `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` |
| MS42DC | `ms42dc_direct_serial_driver` | `/arachne/gripper/command` to Type-C serial frames. The gripper controller is CH91xx/CH343-family; the current unit is treated as the CH9012 path. Recommended alias: `/dev/motor_serial` |
| Aubo i5 | `AuboRobot/aubo_ros2_driver` | TCP/IP + ros2_control, launched with the robot IP |
| Gemini335 | `arachne_sensors` | End-effector RGB-D camera for detection, depth ROI, grasp-pose estimation, and teach observations |
| Leishen Intelligence C16 | `arachne_description` / future lidar driver | Rear-rack lidar model is already in the TF tree; it is planned for obstacle sensing, localization support, and mobile-manipulation safety constraints |

Prepare real-hardware ROS packages:

```bash
./scripts/hardware/prepare_real_hardware_ros.sh
./scripts/hardware/real_aubo_probe.sh
./scripts/hardware/real_aubo_prepare.sh
```

Prefer completing connect -> power on -> start from the teach pendant/control cabinet. If ROS-side remote startup is needed, use only the blocking startup script: it first confirms active controllers, reads the measured joint angles, sends a hold-position command, then runs power on -> Aubo `RobotManage.startup` lifecycle startup -> post-Running steady-state and hold verification. The script never calls `releaseRobotBrake` directly; any protective state, timeout, or controller error aborts the flow.

Remote startup uses two terminals:

```bash
# Terminal 1: start the driver and allow pre-power controller activation
ARACHNE_CONFIRM_AUBO_DRIVER=YES ARACHNE_AUBO_ALLOW_PRESTART=YES ./scripts/hardware/real_aubo_bringup.sh

# Terminal 2: run the blocking remote-start state machine
ARACHNE_CONFIRM_AUBO_REMOTE_START=YES AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/real_aubo_remote_start.sh
```

For day-to-day real-hardware work, use the automatic entry. It selects the Scout and MS42DC `/dev/serial/by-id` ports, checks that Aubo is Running / Normal, then starts the full bringup:

```bash
./scripts/hardware/real_bringup.sh
```

For WSL2, [hurry-porter](https://github.com/zay002/hurry-porter) is recommended for USB handoff, serial discovery, and Waveshare USB-CAN-A diagnostics. When `real_bringup.sh` cannot find serial ports, it first tries to auto-attach the CH9102/CH340 devices; if Windows has not shared them yet, follow the printed manual attach hint.

```bash
hurry scan
hurry waveshare-can-a recv \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --can-bitrate 500000 \
  --frame-type standard \
  --duration 2
```

Real motion tests are dry-run by default. Enable motion only after power, emergency stop, and clearance are confirmed:

```bash
./scripts/hardware/real_hardware_acceptance_test.sh
./scripts/hardware/real_aubo_z_test.sh
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/hardware/real_hardware_acceptance_test.sh
```

For demonstrations, use the one-command teach entry:

```bash
./scripts/hardware/real_teach_demo.sh
```

It starts real bringup, waits for `/odom`, `/joint_states`, the Aubo trajectory action, and gripper status, then opens the teach panel. Closing the panel stops the background bringup. The panel manually controls the base, Aubo tool, and MS42DC gripper, supports Aubo Teach On/Off, RX/RY/RZ wrist jogs, automatic relative base-motion waypoints when a hold-to-drive button is released, wait steps, single-waypoint updates, and waypoint duplication, saves recordings under the local `recordings/teach/` directory, and replays the sequence with one button.

## Project Layout

| Path | Purpose |
| --- | --- |
| `src/arachne_description` | Unified robot model, RViz config, gripper variants, and sensor frames |
| `src/arachne_sensors` | Gemini335 RGB-D camera nodes, reserved C16 lidar integration, and sensor launch files |
| `src/arachne_demo` | Switch Pro controller, Gazebo showroom, autonomous pick validation |
| `src/arachne_hardware` | Real bringup, Scout/MS42DC wrappers, safety state, command gating |
| `src/arachne_control` | ros2_control names, mock controllers, hardware profiles |
| `src/arachne_moveit_config` | MoveIt2 starter config for Aubo i5 with MS42DC or AG95 |
| `src/arachne_nav` | Nav2 starter config for Scout |
| `src/arachne_operator` | Operator panel, grasp task server, sequence executor, VLA/WAM action-chunk translator |
| `src/arachne_agent_bridge` | Safe external-agent tool whitelist, teach-style control bridge, and state snapshot |
| `scripts/env` / `scripts/build` | ROS environment and colcon build entrypoints |
| `scripts/hardware` / `scripts/operator` | Real bringup, acceptance, Aubo helpers, and teach-panel entrypoints |
| `scripts/vision` | Gemini335, YOLO26, TensorRT, INT8 calibration, and live detection entrypoints |
| `scripts/model` / `scripts/sim` / `scripts/godot` | Model checks, simulation demos, and Godot showcase scripts |
| `yolo_workspace` | YOLO venv, weights, engines, datasets, and calibration images |
| `godot/arachne_showcase` | Godot 4.x third-person showcase frontend |
| `docs` | Modeling, control, hardware, calibration, and references |

The `scripts/` root no longer carries old top-level script entrypoints. Use the categorized paths directly, such as `./scripts/hardware/real_full_teach.sh` and `source scripts/env/arachne_env.sh`.

## Documentation

- [Modeling](docs/modeling.md)
- [Control](docs/control.md)
- [Hardware](docs/hardware.md)
- [Calibration](docs/calibration.md)
- [Grasp Task Server](docs/grasp_task_server.md)
- [Agent Bridge](docs/agent_platform.md)
- [References](docs/references.md)

Chinese versions are available as matching `*.zh-CN.md` files.

## Roadmap

- Real-hardware reliability: stabilize one-command bringup, remote startup, payload setup, streaming velocity control, and safe stop for Scout, Aubo, MS42DC, Gemini335, and Leishen Intelligence C16.
- Perception and tasks: collect Gemini335 RGB-D and C16 lidar data, tune YOLO26 detection for trash, workpieces, and charging guns, and build INT8 TensorRT, depth ROI localization, the grasp task server, and local dataset loops.
- Static manipulation: with the base parked, advance two task tracks in parallel: recognize trash, pick it, and place it into the front basket; recognize the charging gun, precisely align, remove it, and insert it. Then extend to workpiece detection, measurement-point localization, and simple assembly pose generation.
- Mobile manipulation: combine base localization, arm reachability, vehicle pose, visual observations, and C16 environment sensing for move-stop-observe-pick, charging-gun insertion/removal, and measurement workflows.
- Deep RL joint control: train base-arm-gripper policies from simulation and real teach data, starting with precision alignment, charging-gun insertion/removal, assembly, and measurement before transferring to the real robot.
- Evaluation and safety: track task success, trajectory smoothness, localization error, contact/jitter behavior, safety-zone violations, and recovery quality through repeatable acceptance tests.

## License

Repository code is released under the [MIT License](LICENSE). Third-party models, CAD files, SDKs, and manuals retain their original licenses; sources are tracked in [third_party/README.md](third_party/README.md) and [docs/references.md](docs/references.md).
