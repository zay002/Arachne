# Control

The first simulation control layer is implemented for RViz and Gazebo demos. The base accepts `/cmd_vel` and publishes `/odom`, `odom -> base_link`, and wheel joint states for the lightweight view; Gazebo mode drives the spawned model through a diff-drive physics plugin and uses a Gazebo-specific Scout wheel orientation so forward commands produce real forward motion. The Aubo arm is controlled by `joint_state_publisher_gui` in RViz mode and by a lightweight Gazebo trajectory bridge in the Switch demo, seeded with the current user-confirmed display pose instead of the folded zero pose. Both MS42DC and AG95 expose the same two gripper states, `Open` and `Close`; the only model difference is the gripper attached under `gripper_adapter_link`.

In `display.launch.py`, default zero-state joints publish to `/arachne/default_joint_states`, GUI slider joints publish to `/arachne/gui_joint_states`, base wheel states publish to `/arachne/base/joint_states`, gripper states publish to `/arachne/gripper/joint_states`, and `joint_state_mux` is the only publisher of the unified `/joint_states` stream used by `robot_state_publisher`.

## Base Simulation

Launch the normal combined simulation:

```bash
./scripts/view_model.sh
```

The `Arachne Base` GUI provides Forward, Back, Left, Right, and Stop. It publishes `geometry_msgs/msg/Twist` on `/cmd_vel`. Terminal control uses the same topic:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25}, angular: {z: 0.0}}"
```

Reset the simulated base pose:

```bash
ros2 service call /arachne/base/reset std_srvs/srv/Trigger {}
```

`base_sim_controller` is intentionally a lightweight kinematic integrator for RViz. Full physics and collision rehearsal should be added later through a dedicated simulation backend.

## Gripper Simulation

Launch MS42DC control with the two-button gripper GUI:

```bash
ros2 launch arachne_description display.launch.py \
  gripper_type:=ms42dc \
  with_gripper_sim:=true \
  with_gripper_gui:=true \
  gripper_sim_profile:=ms42dc
```

The GUI window intentionally exposes only `Open` and `Close`. The current MS42DC close target is `0.6 rad`; override it at launch time only when retuning the physical gripper:

```bash
ros2 launch arachne_description display.launch.py \
  gripper_type:=ms42dc \
  use_gui:=true \
  with_gripper_sim:=true \
  with_gripper_gui:=true \
  gripper_sim_profile:=ms42dc \
  gripper_closed_position:=0.58
```

User-facing simulation commands:

```bash
ros2 service call /arachne/gripper/open std_srvs/srv/Trigger {}
ros2 service call /arachne/gripper/close std_srvs/srv/Trigger {}
```

Launch AG95 control with the same two-button Open/Close interface:

```bash
ros2 launch arachne_description display.launch.py \
  gripper_type:=ag95 \
  with_gripper_sim:=true \
  with_gripper_gui:=true \
  gripper_sim_profile:=ag95
```

MS42DC uses user-created split finger meshes from `third_party/MS42DC_SPLIT`. The hinge axis is CAD Z with URDF axis `0 0 -1`, the right finger mimics the left with multiplier `-1.0`, and the default close target is `0.6 rad`. Gazebo disables the URDF mimic tag because the selected physics engine does not create mimic constraints; the demo instead sends explicit mirrored commands to the left and right finger position controllers.

For manual slider inspection, launch:

```bash
ros2 launch arachne_description display.launch.py gripper_type:=ms42dc use_gui:=true
```

With the helper script, disable the simulator so the joint-state GUI controls the mimic joint directly:

```bash
WITH_GRIPPER_SIM=false WITH_GRIPPER_GUI=false ./scripts/view_model.sh
```

For arm sliders plus gripper services in one session:

```bash
ros2 launch arachne_description display.launch.py \
  gripper_type:=ms42dc \
  use_gui:=true \
  with_gripper_sim:=true \
  with_gripper_gui:=true \
  gripper_sim_profile:=ms42dc
```

Planned controllers:

- `aubo_arm_controller`: joint trajectory controller for `aubo_shoulder_joint`, `aubo_upperArm_joint`, `aubo_foreArm_joint`, `aubo_wrist1_joint`, `aubo_wrist2_joint`, and `aubo_wrist3_joint`.
- `scout_base_controller`: diff drive or native Scout driver bridge.
- `ms42dc_gripper_controller`: hardware-facing wrapper after the MS42DC communication method and command range are confirmed.

Reserved real-hardware files:

- `src/arachne_hardware/arachne_hardware/gripper_serial_driver.py`: MS42DC serial control placeholder.
- `src/arachne_hardware/arachne_hardware/base_serial_driver.py`: Scout/base serial control placeholder.
- `src/arachne_hardware/arachne_hardware/aubo_tcp_driver.py`: Aubo TCP/IP control placeholder.

The control layer should remain split by hardware device while sharing the unified robot state.

## Nintendo Switch Demo

`src/arachne_demo` adds a Nintendo Switch Pro controller-driven demo path:

- `switch_teleop.py`: maps `sensor_msgs/msg/Joy` to `/cmd_vel`, `/arachne/gui_joint_states`, `/arachne/gripper/command`, and `/arachne/demo/reset`. Body mode uses polar arcade drive: joystick radius controls instantaneous speed, while X/Y direction splits that speed into turning and forward/back motion.
- `camera_follow_controller.py`: maps the right stick to a robot-relative orbit angle, publishes `/arachne/camera_yaw`, and publishes `arachne_view_frame` for the RViz-only third-person view.
- `src/arachne_gazebo/gazebo_camera_track_bridge.cpp`: converts the ROS camera offset topic to Gazebo `/gui/track` messages, avoiding repeated `gz service` subprocess calls.
- `src/arachne_gazebo/gazebo_demo_control_bridge.cpp`: mirrors MS42DC open/close commands into two Gazebo finger controllers and forwards Aubo joint-state targets to Gazebo joint trajectory commands.
- `web_gamepad_bridge.py`: serves a small local browser bridge for WSL2 or systems without `/dev/input/js*`.
- `switch_rviz_demo.launch.py`: launches the normal RViz model with gripper/base simulation plus either `joy_node` or the web gamepad bridge.
- `switch_gazebo_demo.launch.py`: opens the Gazebo showroom without RViz, spawns the robot with Gazebo-safe MS42DC and Scout wheel settings, bridges Gazebo `/gz/odom`, and enables the Gazebo follower camera plus diff-drive physics plugins.

Run the playable Gazebo showroom demo:

```bash
./scripts/switch_demo.sh
```

On WSL2, `switch_demo.sh` exports the Mesa D3D12 settings needed by Gazebo GUI so rendering can use the Windows GPU instead of CPU `llvmpipe`. It also defaults Gazebo to the OpenGL backend and a lighter `180 Hz` physics update rate. Tune these without editing files:

```bash
GZ_UPDATE_RATE=120 ./scripts/switch_demo.sh
GZ_RENDER_BACKEND=opengl ./scripts/switch_demo.sh
MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA ./scripts/switch_demo.sh
```

Input backend selection:

```bash
INPUT_BACKEND=auto ./scripts/switch_demo.sh  # default: joy on Linux, web bridge in WSL2
INPUT_BACKEND=joy JOY_DEV=/dev/input/js1 ./scripts/switch_demo.sh
INPUT_BACKEND=web ./scripts/switch_demo.sh
```

With the web backend, open `http://127.0.0.1:8787` in the browser and press any Switch Pro button. The left stick drives the Scout in its own body frame: joystick radius controls instantaneous speed, vertical direction controls forward/back, and horizontal direction controls turning. The right stick orbits the Gazebo follower camera. `B` / `A` open and close the gripper; `ZL` + D-pad up/down moves the selected Aubo joint. `+` or the browser `RESET` button resets the base, arm, gripper, and Gazebo demo pose.

The Switch Pro web-bridge defaults use `forward_axis_multiplier=-1.0` and `lateral_axis_multiplier=1.0`. If another controller reports an axis in the opposite direction, run `FORWARD_AXIS_SIGN=1.0 ./scripts/switch_demo.sh` or `LATERAL_AXIS_SIGN=-1.0 ./scripts/switch_demo.sh`.

The default camera distance is `2.0 m`; tune it with `GAZEBO_CAMERA_DISTANCE=1.7 ./scripts/switch_demo.sh` if a closer or wider capture is needed.

Run the lightweight RViz-only view:

```bash
DEMO_MODE=rviz ./scripts/switch_demo.sh
```

The current Gazebo pass focuses on promotional driving physics and real mesh visualization in a single Gazebo window. The world uses a lighter physics step, disabled shadows, a static ramp, Gazebo DiffDrive, Gazebo `/gz/odom`, high-rate `/gui/track` camera messages, a demo Aubo trajectory bridge, and explicit MS42DC finger position controllers. Full arm and gripper physics control should be moved to ros2_control controllers later.

## Godot Showcase Frontend

`godot/arachne_showcase` is a separate Godot 4.x frontend for high-FPS third-person visualization and teleoperation feel. It loads existing Scout, Aubo i5, MS42DC, AG95, and prop meshes through generated links under `assets/vendor/`, then uses an office-style initial map, collision-aware character-body movement, proportional skid-steer controls, pushable rigid-body props, camera damping, visual suspension, and visual arm/gripper interpolation.

In WSL2, `scripts/godot_showcase.sh` forces `GALLIUM_DRIVER=d3d12` and the OpenGL compatibility renderer because the Vulkan path can fall back to CPU `llvmpipe`. Native Linux can keep Forward+ unless a different renderer is requested.

The right-stick camera reader auto-selects the strongest axis among common right-stick mappings. If a controller needs manual mapping, set `ARACHNE_CAMERA_AXIS=<axis>`.

The bridge layer is intentionally a placeholder:

- `/cmd_vel`: stored from the Godot base teleop output.
- `/joint_states`: stored from the interpolated Aubo preset positions.
- `/odom`: stored from the Godot base pose.
- `/tf`: stores the current `odom -> base_link` and static visual-frame placeholders.

The bridge defaults to standalone memory mode, and switches to UDP placeholder mode when a ROS2 environment is sourced. This keeps the showcase dependency-free while leaving a stable insertion point for a later ROS2, WebSocket, native Godot ROS2, MuJoCo, or other physics backend.

Use `scripts/test_godot_showcase.sh` to run the headless Godot self-test. It links assets, loads the scene, drives a scripted route, checks basic movement/camera/mesh/bridge health, and exits nonzero on regressions.
