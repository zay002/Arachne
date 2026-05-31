# Stage 13: Teach Panel Refinement

## Goal

Make the real-hardware teach panel more useful for demo authoring: enter/exit Aubo teach mode, reuse existing waypoints, and adjust tool orientation.

## Core Files

- `src/arachne_operator/arachne_operator/teach_panel.py`: adds Aubo `Teach On/Off`, RX/RY/RZ wrist-orientation jogs, recorded manual base-motion segments, and `Duplicate` waypoint reuse.
- `src/arachne_operator/launch/teach_panel.launch.py`: exposes the teach command topic and orientation-jog parameters.
- `src/arachne_hardware/arachne_hardware/aubo_tcp_driver.py`: adds `aubo_teach_command_bridge`, which maps `/arachne/aubo/teach_command` to the Aubo 30004 JSON-RPC call `RobotManage.freedrive(true/false)`.
- `src/arachne_hardware/arachne_hardware/hardware_mock.py`: accepts the same teach command topic and reports teach on/off in mock status.
- `scripts/fetch_third_party.sh`: patches the pinned Aubo driver with a teach gate so ros2_control does not keep writing `servoJoint` hold commands during hand guiding.

## Relationships

The teach UI only publishes stable ROS commands: base motion uses `/cmd_vel`, the gripper uses `/arachne/gripper/command`, Aubo teach mode uses `/arachne/aubo/teach_command`, and arm motion still uses the trajectory action/topic. Real bringup adapts the Arachne teach command to Aubo JSON-RPC; mock bringup uses the same topic for hardware-free validation. Manual base commands between button press and release are stored as base-motion segments inside the next waypoint and are replayed with that waypoint.

## Note

RX/RY/RZ are currently conservative wrist joint increments, not full 6D Cartesian IK. They are meant for small teach-in adjustments; complex orientation planning should still go through MoveIt2 or a later full pose-IK path.
