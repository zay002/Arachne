# Modeling

## Principle

Arachne exposes one unified `robot_description`: Scout base, Aubo arm, a selectable gripper, the end-effector support, and optional sensors are one URDF tree. The MS42DC and AG95 variants differ only by the gripper attached under `ee_camera_support_link`.

## Current Files

- `src/arachne_description/urdf/arachne.urdf.xacro`: top-level model composition.
- `urdf/scout/scout_2_vendor.xacro`: Scout v2 model adapted from `agilexrobotics/scout_ros2`.
- `urdf/aubo/aubo_i5_vendor.xacro`: Aubo i5 model adapted from `AuboRobot/aubo_description`.
- `urdf/gripper/ms42dc.urdf.xacro`: user-created movable split model for the Yizhua Robot MS42DC two-finger flexible servo gripper, with revolute left/right finger assemblies, mimic motion, and shared `grasp_frame`.
- `urdf/gripper/ag95.urdf.xacro`: optional DH Robotics AG95 wrapper and shared `grasp_frame`.
- `meshes/gripper/ms42dc/split/*.stl`: RViz meshes copied from `third_party/MS42DC_SPLIT`.
- `urdf/sensors/*`: optional lidar and end-effector camera placeholders.

## Frame Chain

Default: `base_link -> aubo_base_link -> ... -> aubo_wrist3_Link -> ee_camera_support_link -> ms42dc_base_link -> grasp_frame`

AG95 variant: `base_link -> aubo_base_link -> ... -> aubo_wrist3_Link -> ee_camera_support_link -> ag95_base_link -> grasp_frame`

`tool0` remains the Aubo flange frame under `aubo_wrist3_Link`; the physical camera/gripper support is modeled as a sibling branch below `aubo_wrist3_Link`.

`map -> odom -> base_link` is intentionally not inside the URDF. It belongs to localization and odometry.

## Model Policy

Scout, Aubo, and AG95 use vendor-derived meshes and kinematic parameters. MS42DC uses user-created split CAD parts with a revolute left finger and right-finger mimic. The hinge direction has been checked in RViz, and the current default close target is `0.6 rad`.
