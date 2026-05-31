# Stage 9: Teach And Replay Panel

## Goal

Provide a visual teach-in tool for demos and small experiments: manually control the Scout base, Aubo tool, and MS42DC gripper, record the current state by hand, and replay the recorded waypoints with one button.

## Core Files

- `src/arachne_operator/arachne_operator/teach_panel.py`: Tk UI and ROS2 node. It subscribes to `/odom`, `/joint_states`, and hardware status topics, and publishes `/cmd_vel`, Aubo trajectory/action commands, and `/arachne/gripper/command`.
- `src/arachne_operator/launch/teach_panel.launch.py`: launch entry exposing base speeds, Aubo joint names, action name, jog step, and recording directory.
- `scripts/teach_panel.sh`: repository-level launcher that sources `scripts/arachne_env.sh` before starting the launch file.
- `docs/control.zh-CN.md` / `docs/control.md`: documents the panel control contract, launch flow, and waypoint storage location.

## File Relationships

`teach_panel.py` uses the same ROS contracts as real bringup: `/cmd_vel` and `/odom` for the base, `/joint_states` and `FollowJointTrajectory` for the arm, and `/arachne/gripper/command` for the gripper. The UI saves JSON recordings; arm replay uses joint values, and base replay uses relative motion segments, so a recording can support repeatable demos and later calibration.

## Current Boundaries

Base replay still uses `/odom` to close distance and angle, but it no longer stores or replays historical absolute base coordinates. Aubo tool jog uses local position IK without full collision checking; autonomous tasks should still move to MoveIt2/Nav2.
