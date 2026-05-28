<p align="center">
  <img src="docs/demo/arachne.png" alt="Arachne robot system showcase" width="900">
</p>

# Arachne

[中文文档](README.zh-CN.md)

Arachne is a ROS2 workspace for a Scout 2.0 mobile base carrying an Aubo i5 arm and a selectable gripper. The two model variants share the same base, arm, mount, sensor frames, launch flow, and gripper interface; the only model difference is the gripper.

The default hardware model is Scout 2.0 + Aubo i5 + Yizhua Robot MS42DC two-finger flexible servo gripper. AG95 is kept as an open-source gripper variant for comparison and demos. Both grippers are exposed through the same `Open` / `Close` GUI and service interface.

The current milestone is a reliable robot description plus interactive demos: one connected TF tree, real upstream Scout/Aubo/AG95 descriptions, a user-created movable MS42DC split mesh model, lightweight gripper open/close simulation, and a playable Gazebo showroom.

## What Is Included

- `src/arachne_description`: unified Xacro/URDF, RViz config, model variants, mount frames, sensor frames, and MS42DC/AG95 gripper adapters.
- `src/arachne_sim`: RViz-oriented base simulation, `/cmd_vel` integration, odometry TF, wheel joint states, and a small base teleop GUI.
- `src/arachne_gripper`: simulated gripper controller, joint-state mux, and a small `Open` / `Close` GUI.
- `src/arachne_demo`: Nintendo Switch Pro controller teleop, RViz demo launch, Gazebo showroom launch, and Gazebo autonomous pick validation.
- `src/arachne_gazebo`: Gazebo helper nodes for smooth GUI camera tracking and demo arm/gripper commands.
- `src/arachne_hardware`: real-hardware bringup wrapper package. It delegates device control to official/vendor ROS packages for Scout 2.0, Aubo i5, and MS42DC, while keeping Arachne-specific status and command bridges.
- `godot/arachne_showcase`: Godot 4.x high-FPS showcase frontend with visual teleop, follow camera, arm presets, pickable-object demo logic, and ROS2 bridge placeholders.
- `scripts`: setup, third-party fetch, model visualization, URDF check, and gripper smoke-test helpers.
- `docs`: hardware/modeling/control/calibration notes and stage reports.
- `docs/demo/arachne.png`: project showcase image for the repository front page.
- `docs/demo/model_compare.png`: current MS42DC and AG95 model showcase.
- `third_party/MS42DC.step` and `third_party/MS42DC_SPLIT/*.stl`: source CAD and user-created movable split parts for the MS42DC gripper.

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
- Real hardware is being aligned around ROS interfaces: AgileX `scout_ros2` controls Scout 2.0 over CAN, the local MS42DC vendor ROS2 package controls the gripper serial node, and `AuboRobot/aubo_ros2_driver` controls Aubo i5 over TCP/IP through ros2_control.

## Roadmap

1. Finalize physical calibration: tool adapter pose, sensor poses, and collision simplification for planning.
2. Add MoveIt2 configuration for the Aubo arm with interchangeable MS42DC and AG95 end-effectors.
3. Replace the Gazebo auto-pick validation planner with MoveIt2 and ros2_control controllers.
4. Upgrade object grasping from command-level validation to contact-validated or attach-aware Gazebo tasks.
5. Connect the Godot showcase to ROS2 or MuJoCo through the prepared bridge interface.
6. Validate real-hardware ROS bringup on the physical Scout, Aubo, and MS42DC after the remaining materials arrive.
7. Build the operator Web UI after the model, controllers, and launch contracts are stable.

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
  arachne_sim arachne_gripper arachne_hardware arachne_description arachne_gazebo arachne_demo \
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
GRIPPER_TYPE=ag95 GRIPPER_SIM_PROFILE=ag95 ./scripts/view_model.sh
```

## Real Hardware ROS Bringup

Arachne does not reimplement the low-level device protocols. The real-hardware path uses the available official/vendor ROS packages and keeps this repository as the integration layer:

- Scout 2.0: `scout_base` from AgileX `scout_ros2`, backed by `ugv_sdk`, with `/cmd_vel` in and `/odom` plus Scout status out over `can0`.
- MS42DC: vendor `step_motor` ROS2 package from the local MS42DC materials. Its `motor_node` owns the serial port; `ms42dc_official_bridge` maps `/arachne/gripper/command` to the vendor `motor_control` topic.
- Aubo i5: `AuboRobot/aubo_ros2_driver`, launched with `aubo_type:=aubo_i5`, `robot_ip:=...`, and `use_fake_hardware:=false`.

Prepare the vendor packages:

```bash
./scripts/prepare_real_hardware_ros.sh
```

Check the host before connecting real hardware:

```bash
./scripts/check_real_hardware_env.sh
```

The check supports both native Linux and WSL2. Aubo TCP/IP works in either environment when the robot network is reachable. MS42DC serial and Scout USB-CAN need normal Linux device nodes; on WSL2, attach the USB serial/CAN adapter with `usbipd-win` first, then verify `/dev/ttyUSB*` or `can0` inside WSL2.

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
  use_scout:=true scout_port:=can0 \
  use_ms42dc:=true ms42dc_port:=/dev/motor_serial \
  use_aubo:=false
```

When the Aubo driver and its SDK dependencies are installed:

```bash
ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=true use_ms42dc:=true use_aubo:=true \
  aubo_robot_ip:=192.168.127.128
```

## Switch Demo

Connect the Nintendo Switch Pro Controller over Bluetooth, then run the playable Gazebo showroom demo:

```bash
./scripts/switch_demo.sh
```

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

- `docs/reports/stage_0_repository_foundation.md`
- `docs/reports/stage_1_unified_robot_model.md`
- `docs/reports/stage_2_gripper_sim_control.md`
- `docs/reports/stage_3_joint_sim_control.md`
- `docs/reports/stage_4_switch_demo.md`
- `docs/reports/stage_5_godot_showcase.md`
- `docs/reports/stage_6_gazebo_autonomy.md`
- `docs/reports/stage_7_real_hardware_ros_bringup.md`
