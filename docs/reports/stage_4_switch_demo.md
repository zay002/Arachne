# Stage 4 Report: Switch Controller Demo

## Result

Arachne now has a first interactive demo path for a Bluetooth Nintendo Switch controller. The RViz demo controls the mobile base, Aubo joint pose, and MS42DC/AG95 open-close gripper. A Gazebo showroom launch adds a lit physics world with dynamic props and a diff-drive physics preview for promotional capture.

## Core Files

- `src/arachne_demo/`: ROS2 Python package for demo launch and controller input.
- `arachne_demo/switch_teleop.py`: maps Switch controller `Joy` messages to `/cmd_vel`, Aubo joint states, and gripper open/close commands.
- `launch/switch_rviz_demo.launch.py`: starts RViz, base simulation, gripper simulation, `joy_node`, and Switch teleop.
- `launch/switch_gazebo_demo.launch.py`: starts Gazebo, spawns Arachne with Gazebo plugins, and reuses the same Switch teleop path.
- `worlds/arachne_showroom.sdf`: physics-enabled demo world with floor, lighting, ramp, and movable props.
- `urdf/gazebo/arachne_gazebo_plugins.xacro`: Gazebo DiffDrive, joint-state, and arm hold plugins for the demo model.
- `scripts/switch_demo.sh`: one-command helper for RViz or Gazebo demo mode.

## Interfaces

- Controller input: `/joy`
- Base command: `/cmd_vel`
- Arm demo state: `/arachne/gui_joint_states`
- Gripper command: `/arachne/gripper/command`

Gazebo arm physics is intentionally a preview: the arm is held at the display pose there, while RViz is the live arm-control view. The next improvement is to connect Aubo joint commands through ros2_control or Gazebo trajectory control.
