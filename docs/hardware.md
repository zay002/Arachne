# Hardware Notes

## Current Target

Arachne targets a Scout 2.0 mobile base, an Aubo i5 arm, and a Youyeetoo MS42DC soft flexible gripper.

## Known State

- Scout 2.0: modeled from the AgileX Scout v2 description in `scout_ros2`.
- Aubo i5: modeled from the official `AuboRobot/aubo_description` `aubo_i5.urdf`.
- End effector: modeled from the local `third_party/MS42DC.step` CAD, converted to STL for RViz.

## Missing Measurements

- Exact Scout top-plate mounting holes and usable payload layout.
- Aubo flange-to-MS42DC adapter dimensions.
- MS42DC finger motion decomposition, maximum stroke, and communication method for real control.

These values should be measured before real planning or collision checking is trusted.
