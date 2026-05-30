<p align="center">
  <img src="docs/demo/arachne.png" alt="Arachne robot system showcase" width="900">
</p>

# Arachne

[中文默认 README](README.md)

Arachne is a ROS2 workspace for a Scout 2.0 mobile base carrying an Aubo i5 arm and a selectable gripper. The two model variants share the same base, arm, mount, sensor frames, launch flow, and gripper interface; the only model difference is the gripper.

The default hardware model is Scout 2.0 + Aubo i5 + Yizhua Robot MS42DC two-finger flexible servo gripper. AG95 is kept as an open-source gripper variant for comparison and demos. Both grippers are exposed through the same `Open` / `Close` GUI and service interface.

The current milestone is a reliable robot description plus interactive demos: one connected TF tree, real upstream Scout/Aubo/AG95 descriptions, a user-created movable MS42DC split mesh model, lightweight gripper open/close simulation, and a playable Gazebo showroom.

<table>
  <tr>
    <td width="50%" align="center">
      <img src="docs/demo/realbot_1.jpg" alt="Arachne physical robot front view" width="100%">
    </td>
    <td width="50%" align="center">
      <img src="docs/demo/realbot_2.jpg" alt="Arachne physical robot side view" width="100%">
    </td>
  </tr>
</table>

## Manual Map

- [What Is Included](#what-is-included): package-level project map.
- [Current State](#current-state): what is already working.
- [Roadmap](#roadmap): near-term engineering direction.
- [Quick Start](#quick-start): build and open the default RViz model.
- [Planning And Control Skeleton](#planning-and-control-skeleton): MoveIt2, Nav2, ros2_control, sequence execution, and VLA/WAM action chunks.
- [Real Hardware ROS Bringup](#real-hardware-ros-bringup): Scout, MS42DC, and Aubo vendor ROS integration.
- [Switch Demo](#switch-demo), [Gazebo Autonomous Pick](#gazebo-autonomous-pick), and [Godot Showcase](#godot-showcase): interactive demos.
- [Useful Commands](#useful-commands), [Key Frames](#key-frames), and [Reports](#reports): maintenance references.

Related documents: [modeling](docs/modeling.md), [control](docs/control.md), [hardware](docs/hardware.md), [calibration](docs/calibration.md), and [references](docs/references.md).

## What Is Included

- [src/arachne_description](src/arachne_description): unified Xacro/URDF, RViz config, model variants, mount frames, sensor frames, and MS42DC/AG95 gripper adapters.
- [src/arachne_sim](src/arachne_sim): RViz-oriented base simulation, `/cmd_vel` integration, odometry TF, wheel joint states, and a small base teleop GUI.
- [src/arachne_gripper](src/arachne_gripper): simulated gripper controller, joint-state mux, and a small `Open` / `Close` GUI.
- [src/arachne_demo](src/arachne_demo): Nintendo Switch Pro controller teleop, RViz demo launch, Gazebo showroom launch, and Gazebo autonomous pick validation.
- [src/arachne_gazebo](src/arachne_gazebo): Gazebo helper nodes for smooth GUI camera tracking and demo arm/gripper commands.
- [src/arachne_hardware](src/arachne_hardware): real-hardware bringup wrapper package. It delegates device control to official/vendor ROS packages for Scout 2.0, Aubo i5, and MS42DC, while keeping Arachne-specific status and command bridges.
- [src/arachne_control](src/arachne_control): shared ros2_control controller names, mock controller launch, and sim/mock/real hardware profiles.
- [src/arachne_moveit_config](src/arachne_moveit_config): MoveIt2 starter configuration for Aubo i5 with MS42DC or AG95 end-effectors.
- [src/arachne_nav](src/arachne_nav): Nav2 starter configuration for Scout navigation over the shared `/cmd_vel` and `/odom` contract.
- [src/arachne_operator](src/arachne_operator): lightweight Tk operator panel, sequence executor, and VLA/WAM action-chunk translator for safety state, hardware status, odometry, base stop, gripper Open/Close, and external policy integration.
- [godot/arachne_showcase](godot/arachne_showcase): Godot 4.x high-FPS showcase frontend with visual teleop, follow camera, arm presets, pickable-object demo logic, and ROS2 bridge placeholders.
- [scripts](scripts): setup, third-party fetch, gripper switching, model visualization, URDF check, and gripper smoke-test helpers.
- [docs](docs): hardware/modeling/control/calibration notes and stage reports, with matching `*.zh-CN.md` Chinese versions.
- [docs/demo/arachne.png](docs/demo/arachne.png): project showcase image for the repository front page.
- [docs/demo/realbot_1.jpg](docs/demo/realbot_1.jpg) and [docs/demo/realbot_2.jpg](docs/demo/realbot_2.jpg): current physical Arachne robot photos.
- [docs/demo/model_compare.png](docs/demo/model_compare.png): current MS42DC and AG95 model showcase.
- [third_party/MS42DC.step](third_party/MS42DC.step) and [third_party/MS42DC_SPLIT](third_party/MS42DC_SPLIT): source CAD and user-created movable split parts for the MS42DC gripper.

External dependencies are restored by `scripts/fetch_third_party.sh`, with pinned revisions for reproducible setup. `build/`, `install/`, and `log/` are standard colcon outputs generated during local builds.

## Current State

- Scout 2.0, Aubo i5, MS42DC, AG95, lidar, and optional end-effector camera are composed into one robot model.
- The MS42DC and AG95 variants differ only at the gripper under `gripper_adapter_link`.
- Aubo is mounted at the current intended Scout top-deck pose.
- MS42DC uses user-created split CAD meshes with revolute left/right finger links.
- MS42DC close target is calibrated to `0.6 rad` by default.
- RViz starts through `scripts/view_model.sh`, which cleans stale visualization nodes and launches base teleop, arm joint sliders, gripper simulator, and gripper Open/Close GUI.
- The arm slider GUI starts from the current user-confirmed display pose; pressing `Center` returns to that pose.
- `scripts/switch_demo.sh` starts an interactive Nintendo Switch Pro controller demo with Gazebo physics, a smoothed third-person camera, body-relative Scout driving, Aubo joint nudging, and gripper commands. Gazebo uses a physics-specific Scout wheel setup so forward input drives all four wheels in the same direction.
- `scripts/gazebo_autopick_demo.sh` starts a Gazebo validation run where the Scout plans around known showroom obstacles, approaches a visible ground target, and runs realtime Aubo/MS42DC pick control.
- `scripts/godot_showcase.sh` starts a separate Godot 4.x third-person showcase with collision-aware driving, painted materials, visual suspension, smooth camera follow, manual arm nudging, gripper open/close, pickable bottles/balls, and ROS2/UDP bridge placeholders.
- `scripts/use_gripper.sh` is the preferred entry for switching between MS42DC and AG95 across visualization, demos, MoveIt2, ros2_control, Nav2, and pre-hardware launch flows.
- Real hardware is being aligned around ROS interfaces: Scout 2.0 defaults to the Waveshare USB-CAN-A serial bridge driver, MS42DC defaults to the Type-C USB direct serial driver, and Aubo i5 keeps the `AuboRobot/aubo_ros2_driver` TCP/IP + ros2_control path.
- Pre-hardware development can now run against mock nodes, safety state services, ros2_control controller names, MoveIt2 planning config, and a Nav2 starter config.

## Roadmap

1. Finalize physical calibration: tool adapter pose, sensor poses, and collision simplification for planning.
2. Validate the new MoveIt2 and ros2_control starter configs in RViz/Gazebo.
3. Replace the Gazebo auto-pick validation planner with MoveIt2 and ros2_control controllers.
4. Bring up Nav2 against the simulated Scout odometry, then later swap in real odometry/localization.
5. Upgrade object grasping from command-level validation to contact-validated or attach-aware Gazebo tasks.
6. Connect the Godot showcase to ROS2 or MuJoCo through the prepared bridge interface.
7. Continue Aubo real-hardware TCP/IP validation, then stabilize the combined Scout, Aubo, and MS42DC bringup with safety gating.
8. Build the full operator Web UI after the model, controllers, and launch contracts are stable.

## Quick Start

Recommended environments:

- Ubuntu 24.04 + ROS2 Jazzy
- Ubuntu 22.04 + ROS2 Humble

```bash
cd Arachne
./scripts/setup_ubuntu.sh
./scripts/fetch_third_party.sh

# If Conda is active, deactivate it before building ROS packages.
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash  # use /opt/ros/humble/setup.bash on Ubuntu 22.04

colcon build --base-paths src --packages-select \
  aubo_description scout_description dh_ag95_description \
  arachne_sim arachne_gripper arachne_hardware arachne_control arachne_moveit_config \
  arachne_nav arachne_operator arachne_description arachne_gazebo arachne_demo \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

source install/setup.bash
./scripts/view_model.sh
```

`view_model.sh` launches the normal development view: MS42DC model, base teleop GUI, Aubo joint sliders, gripper open/close simulator, and the `Arachne Gripper` Open/Close window. The base GUI publishes `/cmd_vel`; `base_sim_controller` publishes `/odom`, `odom -> base_link`, and wheel joint states.

If the Aubo appears folded into the Scout body, rebuild and relaunch with the helper script so the installed launch file includes the current user-confirmed display pose:

```bash
colcon build --base-paths src --packages-select arachne_description
source install/setup.bash
./scripts/view_model.sh
```

The base can also be commanded from the terminal:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25}, angular: {z: 0.0}}"
```

To view AG95 with the same Open/Close controls:

```bash
./scripts/use_gripper.sh ag95 view
```

The same entry can switch the gripper for other stacks:

```bash
./scripts/use_gripper.sh ms42dc prehardware launch_rviz:=false
./scripts/use_gripper.sh ag95 moveit launch_rviz:=true
./scripts/use_gripper.sh ms42dc gazebo
```

## Planning And Control Skeleton

The pre-hardware planning/control stack can be checked without any physical devices:

```bash
./scripts/check_workspace.sh
```

More detail lives in [docs/control.md](docs/control.md). The main code entry points are [prehardware_control.launch.py](src/arachne_control/launch/prehardware_control.launch.py), [sequence_executor.py](src/arachne_operator/arachne_operator/sequence_executor.py), and [action_chunk_translator.py](src/arachne_operator/arachne_operator/action_chunk_translator.py).

One-command pre-hardware control bringup:

```bash
ros2 launch arachne_control prehardware_control.launch.py launch_rviz:=false
```

Mock hardware bringup:

```bash
ros2 launch arachne_hardware mock_bringup.launch.py
ros2 launch arachne_operator operator_panel.launch.py
ros2 launch arachne_operator sequence_executor.launch.py
```

ros2_control mock controller launch:

```bash
ros2 launch arachne_control mock_ros2_control.launch.py gripper_type:=ms42dc
```

MoveIt2 starter launch:

```bash
ros2 launch arachne_moveit_config moveit_planning.launch.py gripper_type:=ms42dc
```

Nav2 starter launch:

```bash
ros2 launch arachne_nav nav2_sim.launch.py
```

By default this uses the lightweight base simulator and a mock `map -> odom` transform, so Nav2 can become active before lidar/localization hardware is available. When a real localization or SLAM stack provides `map -> odom`, launch with `with_mock_map_odom:=false`.

The sequence executor is a small high-level command surface. It runs task steps with status, timeouts, stop handling, and Nav2 result checks through `/arachne/sequence/command`; for example `ready`, `open`, `demo_pick`, `demo_nav_pick`, or `goto 1.0 0.0 0.0`.

External VLA/WAM policies can use the action-chunk translator. It accepts JSON on `/arachne/vla/action_chunk` and converts each step into `/cmd_vel`, Aubo joint trajectories, and `/arachne/gripper/command`:

```bash
ros2 launch arachne_operator action_chunk_translator.launch.py
ros2 topic pub --once /arachne/vla/action_chunk std_msgs/msg/String \
  "{data: '{\"action\":[0.15,0.0,0,0,0,0,0,0,1],\"duration\":0.3}'}"
```

These entries are intended for interface validation before real hardware arrives. The next tuning pass is to validate planning groups, controller behavior, Nav2 costmaps, and safety gating under RViz/Gazebo.

## Real Hardware ROS Bringup

Arachne uses official/vendor ROS packages where they are stable and keeps this repository as the integration layer:

- Scout 2.0: default `scout_waveshare_serial_driver`, which maps `/cmd_vel` to Scout v2 CAN frames through a Waveshare USB-CAN-A CH340 serial adapter. The official AgileX `scout_base`/SocketCAN path remains available with `scout_driver:=official`.
- MS42DC: default `ms42dc_direct_serial_driver`, which maps `/arachne/gripper/command` to the documented Type-C USB serial frames. The vendor `step_motor` path is still available with `ms42dc_driver:=vendor`.
- Aubo i5: `AuboRobot/aubo_ros2_driver`, launched with `aubo_type:=aubo_i5`, `robot_ip:=...`, and `use_fake_hardware:=false`.

See [docs/hardware.md](docs/hardware.md), [real_bringup.launch.py](src/arachne_hardware/launch/real_bringup.launch.py), and [real_hardware.yaml](src/arachne_hardware/config/real_hardware.yaml) when wiring the physical devices.

Prepare the vendor packages:

```bash
./scripts/prepare_real_hardware_ros.sh
```

Check the host before connecting real hardware:

```bash
./scripts/check_real_hardware_env.sh
```

The check supports both native Linux and WSL2. Aubo TCP/IP works in either environment when the robot network is reachable. MS42DC Type-C serial should appear as `/dev/ttyACM*` or `/dev/ttyCH343USB*`; on WSL2, attach the CH9102 USB device with `usbipd-win` first and point `/dev/motor_serial` at it. Scout uses the Waveshare USB-CAN-A as a CH340 serial device by default; SocketCAN `can0` is optional for Linux adapters supported by the kernel.

Recommended helper: [hurry-porter](https://github.com/zay002/hurry-porter) is optional but useful for WSL2/Windows USB handoff and Waveshare USB-CAN-A diagnostics:

```bash
hurry scan
hurry waveshare-can-a recv \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --can-bitrate 500000 \
  --frame-type standard \
  --duration 2
```

The MS42DC bringup defaults to a conservative `30 deg` relative open/close test at `6 rad/s`. The vendor full-stroke reference is `18720` tenths of a degree (`1872 deg`, about `5.2` turns), but it should only be used after physical travel and homing behavior are confirmed.

Build the core hardware bringup packages:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select \
  ugv_sdk scout_msgs scout_base serial step_motor arachne_hardware \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

Launch any available hardware subset by toggling components:

```bash
source install/setup.bash
ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=true \
  scout_driver:=waveshare \
  scout_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  use_ms42dc:=true ms42dc_port:=/dev/motor_serial \
  use_aubo:=false
```

For a native SocketCAN setup, use `scout_driver:=official scout_port:=can0`. When the Aubo driver and its SDK dependencies are installed:

```bash
ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=true scout_driver:=waveshare \
  use_ms42dc:=true use_aubo:=true \
  aubo_robot_ip:=192.168.127.128
```

After the real hardware is powered and the bringup is stable, run the guarded acceptance sequence from [docs/hardware.md](docs/hardware.md#real-hardware-acceptance-test):

```bash
./scripts/real_hardware_acceptance_test.sh  # dry run
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/real_hardware_acceptance_test.sh
```

Subsystem-only entries are available for staged bringup:

```bash
./scripts/real_base_test.sh
./scripts/real_arm_test.sh
./scripts/real_gripper_test.sh
```

## Switch Demo

Connect the Nintendo Switch Pro Controller over Bluetooth, then run the playable Gazebo showroom demo:

```bash
./scripts/switch_demo.sh
```

Relevant files: [scripts/switch_demo.sh](scripts/switch_demo.sh), [switch_gazebo_demo.launch.py](src/arachne_demo/launch/switch_gazebo_demo.launch.py), [switch_teleop.py](src/arachne_demo/arachne_demo/switch_teleop.py), and [arachne_showroom.sdf](src/arachne_demo/worlds/arachne_showroom.sdf).

On native Linux, `switch_demo.sh` uses `/dev/input/js0` when it exists. In WSL2, or when no joystick device is available, it automatically starts a browser bridge:

```bash
./scripts/switch_demo.sh
# then open http://127.0.0.1:8787 in the Windows or Linux browser
```

You can force an input backend when needed:

```bash
INPUT_BACKEND=joy JOY_DEV=/dev/input/js1 ./scripts/switch_demo.sh
INPUT_BACKEND=web ./scripts/switch_demo.sh
```

For the Switch Pro Controller, the WSL2/browser backend is usually the most reliable path because the controller stays visible to Windows Bluetooth while the browser forwards its standard Gamepad state into ROS2.

<p align="center">
  <img src="docs/demo/Bridge.png" alt="Arachne browser gamepad bridge" width="720">
</p>

For the lightweight RViz-only control view:

```bash
DEMO_MODE=rviz ./scripts/switch_demo.sh
```

Default controls:

- Left stick: proportional body-frame driving; joystick radius controls instantaneous speed, vertical direction controls forward/back, and horizontal direction controls turning.
- Right stick: orbit the smoothed Gazebo chase camera around the robot.
- Hold `ZL` + D-pad up/down: move the selected Aubo joint.
- `L` / `R`: select previous/next Aubo joint.
- `B`: open gripper. `A`: close gripper.
- `+` or the browser `RESET` button: reset the base, arm, gripper, and Gazebo demo pose. `-`: stop base motion.

The default Gazebo version opens only the Gazebo showroom window: it uses the real robot meshes, a lighter physics world, dynamic props, a diff-drive physics plugin, Gazebo `/gz/odom`, a controller-driven third-person camera, and direct demo bridges for Aubo joint nudging plus MS42DC open/close control. RViz remains available as a separate lightweight control view while the full ros2_control/Gazebo stack is developed.

## Gazebo Autonomous Pick

Run the known-world autonomy validation:

```bash
./scripts/gazebo_autopick_demo.sh
```

Relevant files: [scripts/gazebo_autopick_demo.sh](scripts/gazebo_autopick_demo.sh), [gazebo_autopick_demo.launch.py](src/arachne_demo/launch/gazebo_autopick_demo.launch.py), and [gazebo_autopick_planner.py](src/arachne_demo/arachne_demo/gazebo_autopick_planner.py).

This launches Gazebo without the manual teleop node. The planner uses the showroom's known obstacle map to continuously refresh a 2D A* route for Scout, follows it with a turn-then-drive pure-pursuit controller, parks the robot about `0.78 m` in front of the visible ground `pick_bottle` near `(3.4, -2.35)`, then computes Aubo joint targets every control tick with damped least-squares position IK from the current base-to-object pose. Arm commands are sent both as `/arachne/gui_joint_states` and as direct Gazebo joint-position topics bridged through `ros_gz_bridge`; MS42DC open/close still goes through the Gazebo demo bridge. It is a validation step toward MoveIt2 and full ros2_control, not the final hardware planner.

<p align="center">
  <img src="docs/demo/gazebo.png" alt="Arachne Gazebo showroom demo" width="900">
</p>

Camera distance can be tuned without rebuilding:

```bash
GAZEBO_CAMERA_DISTANCE=1.7 ./scripts/switch_demo.sh
```

If another controller reports the left-stick Y axis in the opposite direction, flip it without rebuilding:

```bash
FORWARD_AXIS_SIGN=1.0 ./scripts/switch_demo.sh
```

To manually tune the MS42DC close angle with sliders:

```bash
WITH_GRIPPER_SIM=false WITH_GRIPPER_GUI=false ./scripts/view_model.sh
```

Drag `ms42dc_left_finger_joint`; the right finger follows through the URDF mimic joint. The normal default is already `0.6 rad`, but a one-off launch override is available:

```bash
GRIPPER_CLOSED_POSITION=0.58 ./scripts/view_model.sh
```

## Godot Showcase

The Godot frontend is a high-FPS third-person playable demo for presentations and portfolio videos. It loads the existing Scout 2.0, Aubo i5, MS42DC, AG95, and prop meshes through local links, then runs a flat office-style map with proportional keyboard/gamepad driving, collision-aware Scout movement, pushable props, visual suspension, follow-camera smoothing, MS42DC open/close animation, and Aubo preset interpolation. The Aubo arm uses an orange/black showcase finish, and the map includes reproducibly scattered pickable bottles and balls.

Relevant files: [godot/arachne_showcase](godot/arachne_showcase), [scripts/godot_showcase.sh](scripts/godot_showcase.sh), [scripts/fetch_godot_assets.sh](scripts/fetch_godot_assets.sh), and [scripts/test_godot_showcase.sh](scripts/test_godot_showcase.sh).

<p align="center">
  <img src="docs/demo/godot.png" alt="Arachne Godot showcase frontend" width="900">
</p>

```bash
./scripts/install_godot4.sh   # optional if godot4 is already installed
./scripts/fetch_third_party.sh
./scripts/fetch_godot_assets.sh   # optional CC0 office props
./scripts/godot_showcase.sh
```

If Godot is not on `PATH`, set `GODOT_BIN=/path/to/godot4`. The launcher prepares local mesh links and generated GLB cache files before opening Godot. It automatically uses standalone mode, or UDP bridge mode when a ROS2 environment is sourced. The robot visual meshes and mount dimensions come from the same Scout/Aubo/MS42DC sources used by the URDF/Gazebo model; Godot uses simplified collision proxies and a separate showcase office map for performance.

On WSL2, the launcher automatically selects Mesa D3D12 OpenGL rendering so the window uses the Windows GPU instead of CPU `llvmpipe`, and starts a browser Gamepad API bridge for Switch Pro controllers connected on the Windows side. Open the printed `http://127.0.0.1:8790` page and press any controller button. To prefer a discrete GPU, set:

```bash
MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA ./scripts/godot_showcase.sh
```

Controls: left stick or `WASD` drives the Scout, right stick or `Q/E` orbits the third-person camera, `A/B` or `C/O` closes/opens the gripper, `1..5` selects arm presets, `LB/RB` or `H/K` selects a joint, and D-pad up/down or `U/J` nudges the selected Aubo joint. Hold the right-stick button, press `P`, or use the browser bridge `Auto Pick` button to run the nearest-object demo: select target, drive near it, move the Aubo through a lightweight IK/interpolation path, close the gripper, lift, and return home. The D-pad is deliberately not used for base motion.

If the native Godot controller path reports an unusual camera axis, force the camera axis:

```bash
ARACHNE_CAMERA_AXIS=2 ./scripts/godot_showcase.sh
```

Headless self-test:

```bash
./scripts/test_godot_showcase.sh
```

## Useful Commands

Validate the generated URDF:

```bash
./scripts/check_model.sh
```

Smoke-test both gripper simulation profiles:

```bash
./scripts/test_gripper_sim.sh
```

Reset the simulated base pose:

```bash
ros2 service call /arachne/base/reset std_srvs/srv/Trigger {}
```

Direct launch equivalent:

```bash
ros2 launch arachne_description display.launch.py \
  gripper_type:=ms42dc \
  use_gui:=true \
  with_base_gui:=true \
  with_gripper_sim:=true \
  with_gripper_gui:=true \
  gripper_sim_profile:=ms42dc
```

If RViz opens with only a grid, use `./scripts/view_model.sh` rather than a bare launch command; it clears stale ROS visualization nodes before starting. Wait a few seconds for meshes to load, then check that RViz `Fixed Frame` is `odom`.

## Key Frames

```text
base_link
└── arm_mount_link
    └── aubo_base_link
        └── ... └── tool0
            └── gripper_adapter_link
                └── ms42dc_body_link  # or ag95_base_link
                    ├── ms42dc_base_link
                    ├── ms42dc_left_finger_link
                    ├── ms42dc_right_finger_link
                    └── grasp_frame
```

`map -> odom -> base_link` is not part of this URDF; it will come from localization and odometry later.

## Reports

- [stage 0: repository foundation](docs/reports/stage_0_repository_foundation.md)
- [stage 1: unified robot model](docs/reports/stage_1_unified_robot_model.md)
- [stage 2: gripper sim control](docs/reports/stage_2_gripper_sim_control.md)
- [stage 3: joint sim control](docs/reports/stage_3_joint_sim_control.md)
- [stage 4: Switch demo](docs/reports/stage_4_switch_demo.md)
- [stage 5: Godot showcase](docs/reports/stage_5_godot_showcase.md)
- [stage 6: Gazebo autonomy](docs/reports/stage_6_gazebo_autonomy.md)
- [stage 7: real hardware ROS bringup](docs/reports/stage_7_real_hardware_ros_bringup.md)
- [stage 8: planning/control scaffold](docs/reports/stage_8_planning_control_scaffold.md)

Chinese versions are stored beside each maintained document as `*.zh-CN.md`.
