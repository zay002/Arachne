# Control

The first simulation control layer is implemented for MS42DC and AG95 open/close demos. Both gripper variants expose the same two states, `Open` and `Close`; the only model difference is the gripper attached under `gripper_adapter_link`. In `display.launch.py`, default zero-state joints publish to `/arachne/default_joint_states`, GUI slider joints publish to `/arachne/gui_joint_states`, gripper states publish to `/arachne/gripper/joint_states`, and `joint_state_mux` is the only publisher of the unified `/joint_states` stream used by `robot_state_publisher`.

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

The control layer should remain split by hardware device while sharing the unified robot state.
