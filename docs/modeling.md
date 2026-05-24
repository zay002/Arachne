# Modeling

## Principle

Arachne exposes one unified `robot_description`: Scout base, Aubo arm, MS42DC gripper, mounts, and optional sensors are one URDF tree.

## Current Files

- `src/arachne_description/urdf/arachne.urdf.xacro`: top-level model composition.
- `urdf/scout/scout_2_vendor.xacro`: Scout v2 model adapted from `agilexrobotics/scout_ros2`.
- `urdf/aubo/aubo_i5_vendor.xacro`: Aubo i5 model adapted from `AuboRobot/aubo_description`.
- `urdf/gripper/ag95.urdf.xacro`: DH Robotics AG95 wrapper and shared `grasp_frame`.
- `urdf/mounts/*`: Scout-to-arm and tool-to-gripper fixed adapters.
- `urdf/sensors/*`: optional lidar and end-effector camera placeholders.

## Frame Chain

`base_link -> arm_mount_link -> aubo_base_link -> ... -> tool0 -> gripper_adapter_link -> ag95_base_link -> grasp_link -> grasp_frame`

`map -> odom -> base_link` is intentionally not inside the URDF. It belongs to localization and odometry.

## Placeholder Policy

Scout, Aubo, and AG95 use vendor-derived meshes and kinematic parameters. The AG95 model replaces the earlier MS42DC placeholder while the project evaluates the final physical end effector.
