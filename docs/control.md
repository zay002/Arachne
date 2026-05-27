# Control

The first simulation control layer is implemented for RViz demos. The base accepts `/cmd_vel` and publishes `/odom`, `odom -> base_link`, and wheel joint states. The Aubo arm is still controlled by `joint_state_publisher_gui`, seeded with the current user-confirmed display pose instead of the folded zero pose. Both MS42DC and AG95 expose the same two gripper states, `Open` and `Close`; the only model difference is the gripper attached under `gripper_adapter_link`.

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

MS42DC uses user-created split finger meshes from `third_party/MS42DC_SPLIT`. The hinge axis is CAD Z with URDF axis `0 0 -1`, the right finger mimics the left with multiplier `-1.0`, and the default close target is `0.6 rad`.

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
