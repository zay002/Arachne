# Stage 4 Report: Switch Controller Demo

## Result

Arachne now has an interactive demo path for a Nintendo Switch Pro Controller on both native Linux and WSL2. The default Gazebo showroom opens as a single playable window with body-relative Scout driving, a smoothed robot-following third-person camera, richer terrain props, Aubo joint nudging, MS42DC open/close control, and a diff-drive physics preview. RViz mode remains available for lightweight model inspection.

The Switch Pro axes default to `FORWARD_AXIS_SIGN=-1.0` and `LATERAL_AXIS_SIGN=1.0`, matching the observed Scout front and turn direction in Gazebo. Left-stick input uses polar arcade drive: the X/Y circle radius controls instantaneous speed, and the direction splits that speed into body-frame linear and angular velocity.

## Core Files

- `src/arachne_demo/`: ROS2 Python package for demo launch and controller input.
- `arachne_demo/switch_teleop.py`: maps Switch Pro controller `Joy` messages to `/cmd_vel`, Aubo joint states, and gripper open/close commands, with body-relative base motion.
- `arachne_demo/camera_follow_controller.py`: maps the right stick to a robot-relative camera orbit and publishes camera heading/offset topics.
- `src/arachne_gazebo/`: Gazebo helper package; `gazebo_camera_track_bridge` publishes high-rate `/gui/track` messages for smooth GUI camera tracking, and `gazebo_demo_control_bridge` sends Aubo/MS42DC demo commands into Gazebo.
- `arachne_demo/web_gamepad_bridge.py`: exposes a local browser Gamepad API bridge for WSL2.
- `launch/switch_rviz_demo.launch.py`: starts RViz, base simulation, gripper simulation, the selected input backend, view control, and Switch teleop.
- `launch/switch_gazebo_demo.launch.py`: starts Gazebo without RViz, spawns Arachne with Gazebo-safe Scout wheel and MS42DC settings, bridges `/gz/odom`, and reuses the same Switch teleop path.
- `worlds/arachne_showroom.sdf`: physics-enabled demo world with floor, lighting, ramp, speed bumps, low platform, slalom markers, work table, and movable props.
- `urdf/gazebo/arachne_gazebo_plugins.xacro`: Gazebo DiffDrive, joint-state, Aubo trajectory, and MS42DC finger position plugins for the demo model.
- `scripts/switch_demo.sh`: one-command helper that defaults to Gazebo showroom mode and keeps RViz mode available through `DEMO_MODE=rviz`.

## Interfaces

- Controller input: `/joy`
- Base command: `/cmd_vel`
- Camera heading: `/arachne/camera_yaw`
- Gazebo camera offset: `/arachne/gazebo_camera/follow_offset` -> `/gui/track`
- Gazebo odometry: `/gz/odom`
- Arm demo state: `/arachne/gui_joint_states` -> `/model/arachne/joint_trajectory`
- Gripper command: `/arachne/gripper/command`
- MS42DC Gazebo fingers: `/arachne/gazebo/ms42dc_left_finger/command`, `/arachne/gazebo/ms42dc_right_finger/command`
- RViz follower frame: `arachne_view_frame`

Gazebo disables the MS42DC URDF mimic joint and drives both fingers explicitly because the selected physics engine does not create mimic constraints. The Scout wheel links keep the normal upstream orientation for RViz, while Gazebo uses a physics-specific wheel orientation so forward joystick input drives all four wheels in the same direction. The next improvement is to replace these demo bridges with full ros2_control controllers.
