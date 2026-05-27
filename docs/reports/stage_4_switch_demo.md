# Stage 4 Report: Switch Controller Demo

## Result

Arachne now has an interactive demo path for a Nintendo Switch Pro Controller on both native Linux and WSL2. The default Gazebo showroom opens as a single playable window with camera-relative base driving, a robot-following third-person camera, dynamic props, and a diff-drive physics preview. RViz mode remains available for lightweight arm pose and gripper inspection.

## Core Files

- `src/arachne_demo/`: ROS2 Python package for demo launch and controller input.
- `arachne_demo/switch_teleop.py`: maps Switch Pro controller `Joy` messages to `/cmd_vel`, Aubo joint states, and gripper open/close commands.
- `arachne_demo/camera_follow_controller.py`: maps the right stick to the Gazebo GUI follower camera and the RViz follower frame.
- `arachne_demo/web_gamepad_bridge.py`: exposes a local browser Gamepad API bridge for WSL2.
- `launch/switch_rviz_demo.launch.py`: starts RViz, base simulation, gripper simulation, the selected input backend, view control, and Switch teleop.
- `launch/switch_gazebo_demo.launch.py`: starts Gazebo without RViz, spawns Arachne with Gazebo plugins, and reuses the same Switch teleop path.
- `worlds/arachne_showroom.sdf`: physics-enabled demo world with floor, lighting, ramp, and movable props.
- `urdf/gazebo/arachne_gazebo_plugins.xacro`: Gazebo DiffDrive, joint-state, and arm hold plugins for the demo model.
- `scripts/switch_demo.sh`: one-command helper that defaults to Gazebo showroom mode and keeps RViz mode available through `DEMO_MODE=rviz`.

## Interfaces

- Controller input: `/joy`
- Base command: `/cmd_vel`
- Camera heading: `/arachne/camera_yaw`
- Arm demo state: `/arachne/gui_joint_states`
- Gripper command: `/arachne/gripper/command`
- RViz follower frame: `arachne_view_frame`

Gazebo arm physics is intentionally a preview: the arm is held at the display pose there, while RViz mode is the live arm-control view. The next improvement is to connect Aubo joint commands through ros2_control or Gazebo trajectory control.
