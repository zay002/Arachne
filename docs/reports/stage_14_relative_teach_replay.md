# Stage 14: Relative Teach Replay

## Goal

Fix teach replay so recorded base motion remains reproducible after the Scout moves, and keep arm replay tied to joint values instead of Cartesian coordinates.

## Core Files

- `src/arachne_operator/arachne_operator/teach_panel.py`: records each hold-to-drive base release as a waypoint with relative forward/backward distance or left/right angle; replay ignores historical absolute `base_pose` and closes each relative motion using current `/odom`.
- `docs/control.md` / `docs/control.zh-CN.md`: document the v2 teach recording semantics.

## Relationship

The teach panel still talks only to the existing ROS interfaces: `/cmd_vel`, `/odom`, `/joint_states`, Aubo trajectory action/topic, and `/arachne/gripper/command`. The change is in the recording layer: the base becomes an action segment, while the arm remains a joint-space waypoint.

