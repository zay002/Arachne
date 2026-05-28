# Stage 6: Gazebo Autonomous Pick Validation

## Summary

Arachne now has a Gazebo-side autonomy validation path. It keeps the manual Switch demo intact, but adds a separate launch that uses known showroom geometry to plan a Scout route, avoid obstacles, align near a visible ground pick target, then compute Aubo/MS42DC pick commands online.

## Core Files

- `src/arachne_demo/arachne_demo/gazebo_autopick_planner.py`: known-world planner. It runs continuously refreshed 2D A*, path smoothing, turn-then-drive pure-pursuit base tracking, target alignment, realtime Cartesian pick targets, damped least-squares Aubo position IK, direct joint command publishing, and MS42DC open/close commands.
- `src/arachne_demo/launch/gazebo_autopick_demo.launch.py`: Gazebo launch entry. It spawns Arachne, starts `/cmd_vel`, `/gz/odom`, and Aubo direct joint-command bridges, starts camera tracking, starts the Gazebo arm/gripper bridge, and starts the autonomy planner.
- `src/arachne_demo/worlds/arachne_showroom.sdf`: adds a high-visibility ground target pad plus `pick_bottle` and `pick_ball` targets in an open area farther from the robot.
- `src/arachne_description/urdf/gazebo/arachne_gazebo_plugins.xacro`: adds direct Gazebo joint-position controllers for the six Aubo joints and lowers lateral wheel friction so the four-wheel skid-steer base can rotate realistically.
- `scripts/gazebo_autopick_demo.sh`: one-command runner with the same ROS/Gazebo resource and WSL2 GPU setup used by the manual Gazebo demo.
- `src/arachne_gazebo/src/gazebo_demo_control_bridge.cpp`: reused bridge that converts ROS gripper commands and Aubo joint states into Gazebo finger/trajectory commands.

## Notes

This stage is intentionally not the final planner. It validates autonomy flow in simulation with known world state: realtime base route planning, base/arm sequencing, local position IK, and Gazebo control interfaces. The next step is to replace the local IK with MoveIt2 pose IK/path planning, then move arm/gripper execution onto ros2_control.
