# Modeling

## Principle

Arachne exposes one unified `robot_description`: Scout base, Aubo arm, MS42DC gripper, mounts, and optional sensors are one URDF tree.

## Current Files

- `src/arachne_description/urdf/arachne.urdf.xacro`: top-level model composition.
- `urdf/scout/scout_2_vendor.xacro`: Scout v2 model adapted from `agilexrobotics/scout_ros2`.
- `urdf/aubo/aubo_i5_vendor.xacro`: Aubo i5 model adapted from `AuboRobot/aubo_description`.
- `urdf/gripper/ms42dc.urdf.xacro`: MS42DC mesh wrapper and shared `grasp_frame`.
- `meshes/gripper/ms42dc/MS42DC.stl`: RViz mesh converted from `third_party/MS42DC.step`.
- `urdf/mounts/*`: Scout-to-arm and tool-to-gripper fixed adapters.
- `urdf/sensors/*`: optional lidar and end-effector camera placeholders.

## Frame Chain

`base_link -> arm_mount_link -> aubo_base_link -> ... -> tool0 -> gripper_adapter_link -> ms42dc_body_link -> grasp_frame`

`map -> odom -> base_link` is intentionally not inside the URDF. It belongs to localization and odometry.

## Model Policy

Scout and Aubo use vendor-derived meshes and kinematic parameters. MS42DC uses the real source CAD as a fixed visual mesh for Stage 1; finger actuation will be modeled after the control protocol and moving subassemblies are confirmed.
