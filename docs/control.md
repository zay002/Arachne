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

## Real-Hardware ROS Control

The real-hardware layer is now organized around official/vendor ROS packages rather than custom low-level protocol drivers:

- Scout 2.0 uses AgileX `scout_ros2` and `ugv_sdk`. `scout_base` subscribes to `/cmd_vel` and publishes `/odom`, `/scout_status`, and `/rc_status` over CAN.
- MS42DC uses the vendor `step_motor` ROS2 package from the local gripper materials. `motor_node` owns the serial port and accepts `step_motor/msg/Motor` on `motor_control`; `ms42dc_official_bridge` converts `/arachne/gripper/command` (`open`, `close`, `home`, `stop`) into that vendor message.
- Aubo i5 uses `AuboRobot/aubo_ros2_driver`. The official launch exposes ros2_control trajectory execution for the Aubo arm over TCP/IP.

Prepare package links:

```bash
./scripts/prepare_real_hardware_ros.sh
```

Check native Linux or WSL2 hardware visibility before motion tests:

```bash
./scripts/check_real_hardware_env.sh
```

The check reports ROS setup, vendor package links, MS42DC serial candidates, Scout SocketCAN status, and Aubo TCP reachability. On WSL2, USB serial and USB-CAN adapters must be passed through from Windows with `usbipd-win` before Linux can expose `/dev/ttyUSB*`, `/dev/ttyACM*`, or `can0`.

Build the core bringup packages:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select \
  ugv_sdk scout_msgs scout_base serial step_motor arachne_hardware \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

Launch a partial or full real-hardware session:

```bash
source install/setup.bash
ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=true scout_port:=can0 \
  use_ms42dc:=true ms42dc_port:=/dev/motor_serial \
  use_aubo:=false
```

When the Aubo SDK dependencies and network are ready:

```bash
ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=true use_ms42dc:=true use_aubo:=true \
  aubo_robot_ip:=192.168.127.128
```

The control layer should remain split by hardware device while sharing `/cmd_vel`, `/joint_states`, `/odom`, and the gripper command surface.

## Planning And Control Skeleton

The pre-hardware control skeleton is split into standard packages:

- `arachne_control`: ros2_control controller names and `mock_ros2_control.launch.py`.
- `arachne_moveit_config`: MoveIt2 groups, named arm poses, gripper open/close states, KDL IK, OMPL planning, and controller mapping.
- `arachne_nav`: Nav2 starter params for Scout using `/cmd_vel`, `/odom`, `map -> odom -> base_link`, and the lidar scan contract.
- `arachne_hardware/mock_bringup.launch.py`: simulated hardware status and state output without real devices.
- `arachne_operator`: Tk status panel for safety state, base/Aubo/gripper status, odometry, stop, and gripper Open/Close.
- `arachne_operator/sequence_executor.py`: high-level task executor for arm presets, gripper commands, demo sequences, and Nav2 goals, with status, stop, timeout, and Nav2 result handling.
- `arachne_control/prehardware_control.launch.py`: combined mock bringup for Nav2, MoveIt2, sequence execution, and optional operator panel.

Run all repository-level checks:

```bash
./scripts/check_workspace.sh
```

Launch the combined pre-hardware control stack:

```bash
ros2 launch arachne_control prehardware_control.launch.py launch_rviz:=false
```

Launch mock hardware plus the operator panel:

```bash
ros2 launch arachne_hardware mock_bringup.launch.py
ros2 launch arachne_operator operator_panel.launch.py
ros2 launch arachne_operator sequence_executor.launch.py
```

Launch ros2_control with mock hardware:

```bash
ros2 launch arachne_control mock_ros2_control.launch.py gripper_type:=ms42dc
```

Launch MoveIt2:

```bash
ros2 launch arachne_moveit_config moveit_planning.launch.py gripper_type:=ms42dc
```

Launch Nav2:

```bash
ros2 launch arachne_nav nav2_sim.launch.py
```

The default launch is self-contained for pre-hardware testing: it starts the kinematic base simulator and a mock `map -> odom` transform. Disable that transform with `with_mock_map_odom:=false` once localization or SLAM owns `map -> odom`.

High-level commands can be sent through `/arachne/sequence/command`:

```bash
ros2 topic pub --once /arachne/sequence/command std_msgs/msg/String "{data: ready}"
ros2 topic pub --once /arachne/sequence/command std_msgs/msg/String "{data: demo_pick}"
ros2 topic pub --once /arachne/sequence/command std_msgs/msg/String "{data: demo_nav_pick}"
ros2 topic pub --once /arachne/sequence/command std_msgs/msg/String "{data: 'goto 1.0 0.0 0.0'}"
```

Task progress is published on `/arachne/sequence/status`. The `stop` command cancels the current task, stops `/cmd_vel`, commands the gripper to stop, and attempts to cancel an active Nav2 goal.

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

The current Gazebo pass focuses on promotional driving physics and real mesh visualization in a single Gazebo window. The world uses a lighter physics step, disabled shadows, a flat showroom floor, Gazebo DiffDrive, Gazebo `/gz/odom`, high-rate `/gui/track` camera messages, a demo Aubo trajectory bridge, and explicit MS42DC finger position controllers. Full arm and gripper physics control should be moved to ros2_control controllers later.

## Gazebo Autonomous Pick Validation

`scripts/gazebo_autopick_demo.sh` launches a Gazebo-only autonomy check without the manual Switch teleop node. `gazebo_autopick_demo.launch.py` spawns the same Arachne robot, bridges `/cmd_vel`, `/gz/odom`, and the six direct Aubo joint-position command topics, starts the Gazebo demo arm/gripper bridge, and runs `gazebo_autopick_planner`.

The planner uses the known SDF showroom layout as a deterministic map. It inflates table, marker, crate, and pedestal obstacles by the Scout footprint, continuously refreshes 2D A* to a ground-target approach pose, smooths the route, and tracks it with a turn-then-drive pure-pursuit controller. After arrival, it aligns the chassis toward the visible `pick_bottle` near `(3.4, -2.35)`, computes pre-grasp/grasp/lift Cartesian targets in the base frame, solves Aubo position IK online with a damped least-squares Jacobian, and sends the result through both `/arachne/gui_joint_states` and direct Gazebo joint-position topics. MS42DC open/close still goes through `/arachne/gripper/command`.

This is deliberately a validation layer: it proves the launch/control interfaces, route generation, realtime base/arm coordination, and Gazebo command paths. The next implementation step is replacing the local position-only IK with MoveIt2 pose IK/path planning, then replacing demo bridges with ros2_control controllers.

## Godot Showcase Frontend

`godot/arachne_showcase` is a separate Godot 4.x frontend for high-FPS third-person visualization and teleoperation feel. It loads existing Scout, Aubo i5, MS42DC, AG95, and prop meshes through generated links under `assets/vendor/`, then uses a larger flat office-style initial map, collision-aware character-body movement, proportional skid-steer controls, pushable rigid-body props, pickable bottles/balls, camera damping, visual suspension, visual arm/gripper interpolation, and manual Aubo joint nudging.

In WSL2, `scripts/godot_showcase.sh` forces `GALLIUM_DRIVER=d3d12` and the OpenGL compatibility renderer because the Vulkan path can fall back to CPU `llvmpipe`. It also starts `scripts/godot_gamepad_bridge.py`, a browser Gamepad API bridge for controllers paired to Windows. Native Linux can keep Forward+ and use native Godot joystick input unless the web bridge is explicitly enabled with `GODOT_GAMEPAD_BRIDGE=true`.

The robot visual mesh chain is kept aligned with Gazebo by reusing the same Scout/Aubo/MS42DC asset sources and the same mount/pivot constants. Godot still uses simplified collision proxies and its own office map, so it is a showcase layer rather than the authoritative contact model.

Base driving deliberately reads only `WASD` and the left stick. D-pad up/down is reserved for the selected arm joint, so discrete arm commands cannot accidentally drive the Scout.

The right-stick camera reader auto-selects the strongest axis among common right-stick mappings. If a controller needs manual mapping, set `ARACHNE_CAMERA_AXIS=<axis>`.

Long-pressing the right-stick button, pressing `P`, or clicking the browser bridge `Auto Pick` button starts a lightweight pick demo. Godot searches the nearest pickable object, computes a nearby approach goal, drives the Scout with simple obstacle repulsion, interpolates an Aubo pick pose, closes the MS42DC, attaches the object visually, lifts, and returns the arm to `home`. This is a portfolio/research placeholder; the production path should later be replaced by MoveIt2 planning and a real ROS2 bridge.

The bridge layer is intentionally a placeholder:

- `/cmd_vel`: stored from the Godot base teleop output.
- `/joint_states`: stored from the interpolated Aubo preset positions.
- `/odom`: stored from the Godot base pose.
- `/tf`: stores the current `odom -> base_link` and static visual-frame placeholders.

The bridge defaults to standalone memory mode, and switches to UDP placeholder mode when a ROS2 environment is sourced. This keeps the showcase dependency-free while leaving a stable insertion point for a later ROS2, WebSocket, native Godot ROS2, MuJoCo, or other physics backend.

Use `scripts/fetch_godot_assets.sh` to download optional CC0 office furniture props, then `scripts/test_godot_showcase.sh` to run the headless Godot self-test. The test links assets, loads the scene, drives a scripted route, checks basic movement/camera/mesh/bridge health, verifies pickable target search and auto-pick drive/IK generation, and exits nonzero on regressions.
