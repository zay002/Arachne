# Stage 3 Report: Combined RViz Simulation Control

## Result

Arachne now has a lightweight combined control demo in RViz. The base moves from `/cmd_vel`, the arm remains controllable through joint sliders seeded with the current user-confirmed display pose, and both MS42DC and AG95 use the same Open/Close gripper GUI. This stage is kinematic visualization, not full physics simulation.

## Core Files

- `src/arachne_sim/`: ROS2 Python package for RViz-oriented simulation controllers.
- `arachne_sim/base_sim_controller.py`: integrates `/cmd_vel`, publishes `/odom`, broadcasts `odom -> base_link`, and publishes Scout wheel joint states.
- `arachne_sim/base_teleop_gui.py`: small Forward/Back/Left/Right/Stop GUI that publishes `/cmd_vel`.
- `arachne_gripper/joint_state_mux.py`: merges default, GUI, base, and gripper joint streams into `/joint_states`.
- `launch/display.launch.py`: starts base simulation, base GUI, arm sliders, gripper simulation, gripper GUI, robot state publisher, and RViz.
- `src/arachne_hardware/`: reserved real-hardware package with empty driver files for MS42DC serial, base serial, and Aubo TCP/IP control.

## Interfaces

- Base command: `/cmd_vel`
- Base state: `/odom`, `odom -> base_link`, `/arachne/base/joint_states`
- Base reset: `/arachne/base/reset`
- Arm demo control: `joint_state_publisher_gui`
- Gripper demo control: `/arachne/gripper/open`, `/arachne/gripper/close`

The next step is MoveIt2 setup for arm planning while keeping this shared simulation launch contract stable.
