# Stage 1 Report: Unified Robot Model

## Result

`arachne_description` defines one complete `robot_description` for Scout 2.0, Aubo i5, a selectable gripper, mounting adapters, and optional sensor frames. The MS42DC and AG95 variants differ only by gripper. Scout, Aubo, and AG95 are adapted from upstream open-source hardware descriptions; the Yizhua Robot MS42DC two-finger flexible servo gripper uses user-created split STL parts from the local CAD source.

## Core Files

- `src/arachne_description/urdf/arachne.urdf.xacro`: composes the full robot.
- `urdf/scout/scout_2_vendor.xacro`: adapts AgileX Scout v2 mesh, dimensions, collisions, and wheel frames from `agilexrobotics/scout_ros2`.
- `urdf/aubo/aubo_i5_vendor.xacro`: adapts AuboRobot's `aubo_i5.urdf`, prefixes its links/joints, removes its standalone `world_joint`, and adds `tool0`.
- `urdf/gripper/ms42dc.urdf.xacro`: loads user-created MS42DC split meshes, defines revolute left/right finger motion, and preserves Arachne's shared `grasp_frame`.
- `urdf/gripper/ag95.urdf.xacro`: wraps `dh_ag95_description` as an optional AG95 end effector.
- `src/arachne_gripper`: provides lightweight gripper simulation utilities for MS42DC and AG95 RViz demos.
- `meshes/gripper/ms42dc/split/*.stl`: RViz-ready user-created MS42DC part meshes copied from `third_party/MS42DC_SPLIT`.
- `urdf/mounts/*.xacro`: fixed transforms between base, arm, adapter, and gripper.
- `launch/display.launch.py`: publishes the model, opens joint sliders, and starts RViz.
- `third_party/aubo_description`: upstream Aubo model package.
- `third_party/scout_ros2/scout_description`: upstream Scout model package exposed through `src/vendor/scout_description`.
- `third_party/dh_ag95_gripper_ros2/dh_ag95_description`: optional AG95 model package exposed through `src/vendor/dh_ag95_description`.
- `third_party/MS42DC.step`: source CAD for the current flexible gripper model in this local workspace.

## Relationships

The top-level Xacro includes each module, then connects them as one TF chain: Scout `base_link` to `arm_mount_link`, Aubo `aubo_base_link` to `tool0`, adapter to either MS42DC or AG95, and the selected gripper to `grasp_frame`. AG95 has articulated joints from its upstream model. MS42DC now uses `ms42dc_base_link`, revolute left/right finger links, and a fixed `ms42dc_mid_link`. Both grippers are controlled in demos through the same Open/Close interface.

## Current Limitations

The Scout-to-Aubo mounting transform matches the current hardware layout. The tool-to-gripper adapter transform still needs direct measurement against the physical MS42DC mounting plate. MS42DC now uses `0.6 rad` as the default simulated closed state.
