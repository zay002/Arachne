# Hardware Notes

## Current Target

Arachne targets a Scout 2.0 mobile base, an Aubo i5 arm, and a Youyeetoo MS42DC soft flexible gripper.

## Known State

- Scout 2.0: modeled from the AgileX Scout v2 description in `scout_ros2`.
- Aubo i5: modeled from the official `AuboRobot/aubo_description` `aubo_i5.urdf`.
- End effector: currently modeled as a DH Robotics AG95 adaptive gripper from `dh_ag95_gripper_ros2`.

## Missing Measurements

- Exact Scout top-plate mounting holes and usable payload layout.
- Real `base_link -> arm_mount_link` transform from the physical robot.
- Aubo flange-to-MS42DC adapter dimensions.
- Final MS42DC CAD, finger geometry, maximum stroke, and communication method if the project returns to the original MS42DC hardware.

These values should be measured before real planning or collision checking is trusted.
