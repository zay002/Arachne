# Hardware Notes

## Current Target

Arachne targets a Scout 2.0 mobile base, an Aubo i5 arm, and a Yizhua Robot MS42DC two-finger flexible servo gripper.

## Known State

- Scout 2.0: modeled from the AgileX Scout v2 description in `scout_ros2`.
- Aubo i5: modeled from the official `AuboRobot/aubo_description` `aubo_i5.urdf`.
- End effector: default model is the Yizhua Robot MS42DC two-finger flexible servo gripper from local `third_party/MS42DC.step`; AG95 is retained as an optional open-source gripper variant.
- Model variants differ only by gripper. Scout, Aubo, mounts, sensors, launch flow, and Open/Close gripper interface are shared.

## Missing Measurements

- Exact Scout top-plate mounting holes and usable payload layout.
- Aubo flange-to-MS42DC adapter dimensions.
- MS42DC communication method for real hardware control.

These values should be measured before real planning or collision checking is trusted.
