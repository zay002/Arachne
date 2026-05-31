# Stage 15: Teach Arm Replay Recovery

## Goal

Make Aubo teach replay recover cleanly after hand-guiding, and make Scout teach motion slightly less sluggish while staying conservative.

## Core Files

- `src/arachne_hardware/arachne_hardware/aubo_tcp_driver.py`: waits for Teach Off to actually leave freedrive before clearing the local ros2_control teach gate.
- `third_party/aubo_ros2_driver/aubo_ros2_driver/src/aubo_hardware_interface.cpp`: if Aubo servo mode is briefly unavailable after teach/prestart, the hardware interface holds measured joints and retries instead of returning `ERROR` and deactivating controllers.
- `src/arachne_operator/arachne_operator/teach_panel.py`: clears stale cancel state before new arm jogs, verifies arm feedback after replay goals, and raises teach base speed defaults to `0.08 m/s`, `0.30 rad/s` manual and `0.04 m/s`, `0.14 rad/s` replay.
- `scripts/real_bringup.sh` / `scripts/real_aubo_bringup.sh`: clear stale `/tmp/arachne_aubo_teach_mode` on startup unless explicitly preserved for debugging.

## Relationship

The failure came from the Aubo controller path, not the saved waypoint format: after Teach Off, servo mode was not immediately ready, so ros2_control deactivated the hardware and `joint_trajectory_controller` rejected every later replay goal. The new path keeps the controller alive while the driver re-enters servo mode.
