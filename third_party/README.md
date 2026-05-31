# Third-Party Assets

Arachne uses upstream model, driver, and asset packages for the real Scout 2.0 and Aubo i5 stack. Empty directories with only `.gitkeep` mark expected local fetch/extract targets:

- `aubo_description`: cloned from `https://github.com/AuboRobot/aubo_description`
- `aubo_ros2_driver`: cloned from `https://github.com/AuboRobot/aubo_ros2_driver` and patched locally for Arachne's current Aubo bringup flow
- `scout_ros2`: cloned from `https://github.com/agilexrobotics/scout_ros2`
- `ugv_sdk`: cloned from `https://github.com/agilexrobotics/ugv_sdk`
- `MS42DC.step`: local source CAD for the current flexible gripper. The committed runtime mesh is generated from this file and stored in `src/arachne_description/meshes/gripper/ms42dc/`.
- `dh_ag95_gripper_ros2`: optional AG95 model package cloned from `https://github.com/ian-chuang/dh_ag95_gripper_ros2`
- `ms42dc_step_motor_ros2`: extracted locally from the MS42DC vendor ROS2 zip when official examples are needed
- `kenney`, `LARA_AUBOi5_AG95`, `scout_ros`: optional local asset/model sources used by the Godot showcase when available

When licensed CAD, STL, SDK, or manual files are added later, keep their source, license, version, and checksum documented here. If a vendor file cannot be redistributed, store it outside the repository and document the expected local path in `docs/references.md`.
