<p align="center">
  <img src="docs/demo/model_compare.png" alt="Arachne MS42DC and AG95 model variants" width="900">
</p>

# Arachne

Arachne is a Linux-native ROS2 framework for a Scout 2.0 + Aubo i5 mobile manipulator with selectable MS42DC and AG95 gripper models.

The first milestone is the robot model: one inspectable `robot_description`, one connected TF tree, and launch files that show the full robot in RViz. Control, MoveIt2, simulation, and Web UI will build on top of this model after the frames and dimensions are stable.

## Current Status

Implemented:

- `arachne_description` ROS2 package.
- Unified Xacro model using the AgileX Scout v2 description, the AuboRobot Aubo i5 description, selectable MS42DC or AG95 grippers, mounts, lidar, and optional end-effector camera.
- RViz display launch with `joint_state_publisher`.
- Reproducible Ubuntu setup and model-check scripts.
- Stage reports in `docs/reports/`.

Still pending:

- MS42DC is the default gripper because it matches the current hardware; AG95 is retained as an open-source alternative model.
- Mount transforms reflect the current intended hardware layout and should be rechecked if the physical mount changes.
- MS42DC motion/control integration, MoveIt2 config, simulation backend, and Web dashboard are not implemented yet.

## Repository Layout

```text
Arachne/
├── src/arachne_description/   # ROS2 description package
├── src/vendor/                 # symlinks to vendored model packages
├── docs/                      # hardware, modeling, calibration, control notes
├── docs/reports/              # short reports after each completed stage
├── scripts/                   # setup and validation helpers
├── third_party/               # downloaded upstream model packages
├── plan.md                    # development plan
└── README.md
```

## Environment

Recommended:

- Ubuntu 22.04 + ROS2 Humble
- Ubuntu 24.04 + ROS2 Jazzy

Install dependencies:

```bash
cd Arachne
./scripts/setup_ubuntu.sh
source /opt/ros/humble/setup.bash  # use /opt/ros/jazzy/setup.bash on Ubuntu 24.04
colcon build --symlink-install
source install/setup.bash
```

The setup script selects Humble on Ubuntu 22.04 and Jazzy on Ubuntu 24.04. You can override it with `ROS_DISTRO=humble` or `ROS_DISTRO=jazzy`; source the matching `/opt/ros/<distro>/setup.bash`.

The required upstream model packages are included under `third_party/` in this working tree:

- `AuboRobot/aubo_description`
- `ian-chuang/dh_ag95_gripper_ros2/dh_ag95_description`
- `agilexrobotics/scout_ros2/scout_description`

If they are missing, run:

```bash
./scripts/fetch_third_party.sh
```

## Run The Model

Build the description packages and launch the default MS42DC model in RViz:

```bash
cd Arachne
./scripts/fetch_third_party.sh

# If Conda is active, deactivate it before building ROS packages.
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash

# Recommended when switching model variants or after pulling updates.
rm -rf build/aubo_description build/scout_description build/dh_ag95_description build/arachne_description \
       install/aubo_description install/scout_description install/dh_ag95_description install/arachne_description

colcon build --base-paths src --packages-select \
  aubo_description scout_description dh_ag95_description arachne_description \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

source install/setup.bash
ros2 launch arachne_description display.launch.py gripper_type:=ms42dc
```

For Ubuntu 22.04, source `/opt/ros/humble/setup.bash` instead.

If the RViz window opens before the robot appears, wait a few seconds. The model is loaded from `/robot_description`, and the first render may lag while mesh resources are loaded.

Quick relaunch after a previous build:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch arachne_description display.launch.py
```

Useful launch arguments:

```bash
ros2 launch arachne_description display.launch.py \
  arm_mount_xyz:="0.22 0.0 0.155" \
  arm_mount_rpy:="0.0 0.0 1.57079632679" \
  gripper_type:=ms42dc \
  with_lidar:=true \
  with_ee_camera:=false
```

To visualize the AG95 variant, build the optional AG95 description package once and launch with `gripper_type:=ag95`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select \
  dh_ag95_description arachne_description \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch arachne_description display.launch.py gripper_type:=ag95
```

If Conda is active and `dh_ag95_description` fails to find `catkin_pkg`, deactivate Conda or build with the system Python as shown above.

## RViz Troubleshooting

If RViz opens but the model is blank, first close stale visualization nodes and relaunch:

```bash
pkill -x rviz2 2>/dev/null || true
pkill -x robot_state_publisher 2>/dev/null || true
pkill -x joint_state_publisher 2>/dev/null || true

source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch arachne_description display.launch.py gripper_type:=ms42dc
```

In RViz, check that `Global Status` is `Ok`, `RobotModel` is enabled, and `Fixed Frame` is `base_link`. Click `Reset` in the lower-left corner or zoom out if the camera is looking at empty space.

To confirm the model is being published:

```bash
ros2 topic echo /robot_description --once --qos-durability transient_local
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo base_link ms42dc_body_link
```

## Validate

```bash
./scripts/check_model.sh
ros2 run tf2_tools view_frames
```

`check_model.sh` writes `/tmp/arachne.urdf` and runs `check_urdf` when that tool is installed.

## Model Structure

The intended frame chain is:

```text
base_link
├── base_footprint
├── arm_mount_link
│   └── aubo_base_link
│       └── ... └── tool0
│                   └── gripper_adapter_link
│                       └── ms42dc_body_link  # or ag95_base_link when gripper_type:=ag95
│                           └── grasp_frame
├── lidar_link
├── inertial_link
└── wheel links
```

`map -> odom -> base_link` is not part of the URDF. It will come from localization and odometry later.

## Mount Pose

The current Aubo-on-Scout pose is the intended mounting pose for this hardware configuration.

Default:

```text
base_link -> arm_mount_link:
  xyz = 0.22 0.0 0.155
  rpy = 0.0 0.0 1.57079632679
```

Why this value is used:

- `base_link` is the Scout v2 root frame from `agilexrobotics/scout_ros2`.
- `z=0.155` places the Aubo mount at the configured top-deck height.
- `x=0.22` places the arm forward on the Scout top deck.
- `yaw=90 deg` sets the Aubo base orientation relative to the Scout base.

## Gripper Model

The default MS42DC source CAD is kept as `third_party/MS42DC.step` in the local workspace. RViz cannot load STEP directly, so the committed runtime mesh is:

```text
src/arachne_description/meshes/gripper/ms42dc/MS42DC.stl
```

It was generated with:

```bash
sudo apt-get install -y gmsh
./scripts/convert_ms42dc_step.sh
```

The STL uses millimeters from the original CAD and is scaled to meters in `urdf/gripper/ms42dc.urdf.xacro`.

The AG95 variant uses `ian-chuang/dh_ag95_gripper_ros2` through `src/vendor/dh_ag95_description`.

If the physical mounting plate changes, test a different pose without editing files:

```bash
ros2 launch arachne_description display.launch.py \
  arm_mount_xyz:="0.20 0.00 0.16" \
  arm_mount_rpy:="0.0 0.0 1.57079632679"
```

## Stage Reports

- `docs/reports/stage_0_repository_foundation.md`
- `docs/reports/stage_1_unified_robot_model.md`
