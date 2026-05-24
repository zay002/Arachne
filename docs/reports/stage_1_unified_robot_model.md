# Stage 1 Report: Unified Robot Model

## Result

`arachne_description` defines one complete `robot_description` for Scout 2.0, Aubo i5, AG95, mounting adapters, and optional sensor frames. Scout, Aubo, and AG95 are adapted from upstream open-source hardware descriptions.

## Core Files

- `src/arachne_description/urdf/arachne.urdf.xacro`: composes the full robot.
- `urdf/scout/scout_2_vendor.xacro`: adapts AgileX Scout v2 mesh, dimensions, collisions, and wheel frames from `agilexrobotics/scout_ros2`.
- `urdf/aubo/aubo_i5_vendor.xacro`: adapts AuboRobot's `aubo_i5.urdf`, prefixes its links/joints, removes its standalone `world_joint`, and adds `tool0`.
- `urdf/gripper/ag95.urdf.xacro`: wraps `dh_ag95_description`, mounts AG95, and preserves Arachne's shared `grasp_frame`.
- `urdf/mounts/*.xacro`: fixed transforms between base, arm, adapter, and gripper.
- `launch/display.launch.py`: publishes the model, opens joint sliders, and starts RViz.
- `third_party/aubo_description`: upstream Aubo model package.
- `third_party/scout_ros2/scout_description`: upstream Scout model package exposed through `src/vendor/scout_description`.
- `third_party/dh_ag95_gripper_ros2/dh_ag95_description`: upstream AG95 model package exposed through `src/vendor/dh_ag95_description`.

## Relationships

The top-level Xacro includes each module, then connects them as one TF chain: Scout `base_link` to `arm_mount_link`, Aubo `aubo_base_link` to `tool0`, adapter to AG95, and AG95 to `grasp_frame`. Control-specific files are optional and disabled by default so the first milestone stays visualization-focused.

## Current Limitations

The Scout-to-Aubo and tool-to-gripper mounting transforms are approximate until measured on the real robot.
