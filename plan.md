# Arachne Development Plan

> **Arachne** is a Linux-native ROS2 framework for mobile manipulation.  
> It treats the mobile manipulator as **one robot in description**, **multiple devices in control**, and **selectable groups in planning**.

Current target hardware:

- **Manipulator:** Aubo i5
- **Mobile base:** AgileX Scout 2.0
- **End effector:** 易爪机器人 / Youyeetoo MS42DC soft flexible gripper
- **Primary OS:** Ubuntu Linux
- **Core middleware:** ROS2

---

## 0. Core Decision

The first milestone of Arachne is **not control, not Web UI, not MuJoCo simulation**.

The first milestone is:

> Build a correct, reusable, inspectable model of the complete mobile manipulator.

This means:

```text
Scout 2.0 base
+ Aubo i5 arm
+ MS42DC gripper
+ optional camera / lidar / tool frames
= one consistent robot_description
```

The model must be valid in:

- RViz
- TF tree
- MoveIt2 Setup Assistant
- ros2_control configuration
- future MuJoCo / Gazebo / Isaac Sim integration
- Web-based digital twin visualization

---

## 1. Project Positioning

### 1.1 Name

**Arachne**

### 1.2 Full Description

Arachne is a Linux-native ROS2-based digital twin and teleoperation framework for mobile manipulation, integrating a Scout 2.0 mobile base, Aubo i5 robotic arm, MS42DC soft gripper, robot modeling, real robot control, simulation backends, end-effector vision, and web-based interaction.

### 1.3 Short Description

A Linux-native ROS2 framework for modeling, simulating, and controlling a Scout-Aubo mobile manipulator.

### 1.4 Slogan

> Weaving simulation and reality for mobile manipulation.

---

## 2. Platform Strategy

### 2.1 Primary Platform

Arachne is **Linux-native**.

Recommended configurations:

```text
Stable option:
  Ubuntu 22.04 + ROS2 Humble

Long-term/newer option:
  Ubuntu 24.04 + ROS2 Jazzy
```

Early development should prioritize **Ubuntu 22.04 + ROS2 Humble** unless a required package strongly favors Jazzy.

### 2.2 Windows Strategy

Windows is **not** an early development target.

Possible future support:

```text
Windows browser frontend only
Windows as SSH / VSCode client only
No Windows-native robot control in early versions
```

### 2.3 Rule

```text
Linux = robot system, ROS2, control, simulation, sensors
Browser = cross-platform dashboard
Windows = optional user-side access later
```

---

## 3. Hardware Configuration

### 3.1 Aubo i5

Role:

- 6-DOF manipulator
- Main manipulation device
- MoveIt2 planning group: `aubo_arm`
- ros2_control controller: `aubo_arm_controller`

Key modeling requirements:

- accurate joint names
- correct joint limits
- correct link mesh scale
- correct flange/tool frame
- collision geometry simplified for planning
- ros2_control transmissions/interfaces

Candidate open-source model sources:

1. `AuboRobot/aubo_description`
   - Contains `aubo_i5.urdf`
   - Useful as the primary visual/kinematic source for Aubo i5
   - URL: https://github.com/AuboRobot/aubo_description

2. `AuboRobot/aubo_robot`
   - Official ROS1/Noetic Aubo support package
   - Useful for legacy descriptions and naming conventions
   - URL: https://github.com/AuboRobot/aubo_robot

3. `ian-chuang/LARA_AUBOi5_AG95`
   - Contains an Aubo i5 xacro macro and an example of arm + gripper integration
   - Developed for ROS1/Gazebo/MoveIt, but useful for modeling structure
   - URL: https://github.com/ian-chuang/LARA_AUBOi5_AG95

4. `hai-h-nguyen/aubo_i5_robot` / `hai-h-nguyen/aubo-i5-full`
   - Community Aubo i5 examples with gripper/pedestal variants
   - Useful as secondary references only

Decision:

```text
Use official Aubo description as the base source.
Convert or wrap it into Arachne's own ROS2 xacro macro.
Do not directly depend on ROS1 package structure.
```

---

### 3.2 AgileX Scout 2.0

Role:

- skid-steering mobile base
- mobile platform for the Aubo i5
- Nav2-compatible base
- ros2_control or native base driver target

Candidate open-source model/control sources:

1. `agilexrobotics/scout_ros2`
   - ROS2 support package for Scout
   - Includes `scout_base`, `scout_description`, and `scout_msgs`
   - Supports Scout / Scout Mini / Scout Mini Omni
   - Uses CAN interface
   - URL: https://github.com/agilexrobotics/scout_ros2

2. `agilexrobotics/scout_ros`
   - ROS1 support package
   - Includes `scout_description` and sample xacro customization
   - Useful as a modeling reference
   - URL: https://github.com/agilexrobotics/scout_ros

3. `agilexrobotics/ugv_sdk`
   - Low-level C++ SDK for AgileX mobile platforms
   - Used by Scout ROS packages
   - URL: https://github.com/agilexrobotics/ugv_sdk

4. `AIRLab-POLIMI/scout_nav2`
   - Nav2 configuration and simulation for AgileX Scout
   - Useful reference for navigation and mobile manipulation context
   - URL: https://github.com/AIRLab-POLIMI/scout_nav2

Decision:

```text
Use scout_ros2 as the primary Scout model/control reference.
Extract and adapt scout_description into arachne_description.
Keep Scout base control separate from arm control in early phases.
```

---

### 3.3 MS42DC Soft Flexible Gripper

Role:

- end effector
- flexible/adaptive soft gripper
- mounted on Aubo i5 flange
- controlled separately from Aubo arm

Known public information:

- The Youyeetoo page describes a soft flexible gripper claw with a new version using the `MS42DC` profile.
- The device has a built-in control panel in the new version.
- Public product information mentions:
  - grasping irregular objects
  - flexible structure
  - repeatability around 0.08 mm
  - grasp object size about 10-120 mm
  - 12V-24V input
  - SDK download / C demo / schematic availability

Candidate model sources:

```text
No reliable open-source ROS/URDF model for MS42DC was found during initial search.
```

Fallback sources and references:

1. Official / distributor documentation
   - Youyeetoo soft flexible robot gripper claw page
   - URL: https://www.youyeetoo.com/blog/detail/youyeetoo-soft-flexible-robot-gripper-claw-whdpakmg0020-whdpakmg0026-206

2. Generic gripper modeling references
   - `PickNikRobotics/ros2_robotiq_gripper`
   - Useful for ROS2 gripper description + driver + controller package organization
   - URL: https://github.com/PickNikRobotics/ros2_robotiq_gripper

3. Robotiq / two-finger / soft gripper URDF examples
   - Useful for mimic joints, gripper action controller, and simplified collision geometry
   - Do not copy geometry unless license permits

Decision:

```text
Build a custom simplified MS42DC model in Arachne.
Use a conservative collision approximation first.
Use real dimensions from manual/CAD if available.
If CAD/STL is not available, create simple box/cylinder/finger geometry.
```

Initial modeling approximation:

```text
ms42dc_base_link
├── ms42dc_left_finger_base_link
│   └── ms42dc_left_finger_tip_link
├── ms42dc_right_finger_base_link
│   └── ms42dc_right_finger_tip_link
├── ms42dc_center_frame
└── grasp_frame
```

Initial control abstraction:

```text
single gripper command:
  open / close / position / force-like scalar if supported

ROS2 interface:
  GripperCommand action or custom service first
  ros2_control hardware interface later
```

---

## 4. Modeling Philosophy

### 4.1 Core Principle

> Arachne models the whole hardware setup as one robot.

There should be one unified:

```text
/robot_description
```

not separate descriptions such as:

```text
/scout/robot_description
/aubo/robot_description
/gripper/robot_description
```

### 4.2 Description Layer Rule

```text
One complete URDF/Xacro tree.
Multiple included submodules.
One coherent TF tree.
```

### 4.3 Control Layer Rule

```text
Multiple hardware devices.
Multiple controllers.
One shared robot state.
```

### 4.4 Planning Layer Rule

```text
Use selectable groups:
  aubo_arm
  gripper
  mobile_base
  whole_body
```

Do not force whole-body planning at the beginning.

---

## 5. Target TF Tree

The final model should converge toward:

```text
map
└── odom
    └── base_link                         # Scout 2.0 main body
        ├── base_footprint
        ├── left_front_wheel_link
        ├── right_front_wheel_link
        ├── left_rear_wheel_link
        ├── right_rear_wheel_link
        ├── arm_mount_link                # physical mounting plate on Scout
        │   └── aubo_base_link            # Aubo i5 base frame
        │       └── aubo_link_1
        │           └── aubo_link_2
        │               └── aubo_link_3
        │                   └── aubo_link_4
        │                       └── aubo_link_5
        │                           └── aubo_link_6
        │                               └── aubo_flange_link
        │                                   └── tool0
        │                                       └── ms42dc_base_link
        │                                           └── grasp_frame
        ├── lidar_link                    # optional
        └── camera_link                   # optional body camera
```

Important notes:

- `map -> odom` is published by localization / SLAM.
- `odom -> base_link` is published by odometry / base driver.
- `base_link -> arm_mount_link` is a fixed joint in URDF.
- `arm_mount_link -> aubo_base_link` is a fixed joint in URDF.
- `tool0 -> ms42dc_base_link` is a fixed joint in URDF.
- `grasp_frame` is a virtual frame for grasp planning.

---

## 6. Model Package Structure

Recommended first package:

```text
arachne_description/
├── package.xml
├── CMakeLists.txt
├── urdf/
│   ├── arachne.urdf.xacro
│   ├── scout/
│   │   ├── scout_2_base.urdf.xacro
│   │   ├── scout_2_wheels.urdf.xacro
│   │   └── scout_2_ros2_control.xacro
│   ├── aubo/
│   │   ├── aubo_i5.urdf.xacro
│   │   ├── aubo_i5_macro.xacro
│   │   └── aubo_i5_ros2_control.xacro
│   ├── gripper/
│   │   ├── ms42dc.urdf.xacro
│   │   ├── ms42dc_simple_collision.xacro
│   │   └── ms42dc_ros2_control.xacro
│   ├── mounts/
│   │   ├── scout_aubo_mount.xacro
│   │   └── aubo_ms42dc_adapter.xacro
│   └── sensors/
│       ├── ee_camera.xacro
│       └── lidar.xacro
├── meshes/
│   ├── scout/
│   ├── aubo/
│   ├── gripper/
│   └── mounts/
├── config/
│   ├── joint_limits.yaml
│   ├── physical_parameters.yaml
│   └── model_variants.yaml
├── launch/
│   ├── display.launch.py
│   └── view_model.launch.py
└── rviz/
    └── arachne_model.rviz
```

Top-level model file:

```text
arachne_description/urdf/arachne.urdf.xacro
```

Responsibilities:

```text
include Scout model
include Aubo i5 model
include MS42DC model
include mounts/adapters
include optional sensors
define fixed joints between modules
expose parameters for mount pose and tool pose
```

---

## 7. First Modeling Milestone

### 7.1 Goal

Create a visible, valid, and inspectable model:

```text
Scout 2.0 + Aubo i5 + MS42DC
```

### 7.2 Deliverables

```text
arachne_description
one top-level arachne.urdf.xacro
RViz display launch file
valid TF tree
initial joint state GUI support
initial simplified MS42DC gripper model
initial mounting transform parameters
```

### 7.3 Commands

```bash
ros2 launch arachne_description display.launch.py
ros2 run tf2_tools view_frames
ros2 run xacro xacro src/arachne_description/urdf/arachne.urdf.xacro > /tmp/arachne.urdf
check_urdf /tmp/arachne.urdf
```

### 7.4 Acceptance Criteria

The milestone is accepted when:

- RViz displays the whole robot correctly.
- Aubo i5 is physically mounted on the Scout base, not floating.
- MS42DC is attached to the Aubo flange/tool frame.
- Joint state publisher can move Aubo joints.
- Gripper visualization can open/close approximately, or at least has a valid grasp frame.
- TF tree has no disconnected branches.
- Link scales are correct.
- `base_link`, `aubo_base_link`, `tool0`, `ms42dc_base_link`, and `grasp_frame` are clearly defined.

---

## 8. Mounting Transform Strategy

The most important fixed transform is:

```text
base_link -> arm_mount_link -> aubo_base_link
```

This should be parameterized:

```xml
<xacro:property name="arm_mount_xyz" value="0.35 0.0 0.45"/>
<xacro:property name="arm_mount_rpy" value="0.0 0.0 1.5708"/>
```

Do not hard-code the final value too early.

Recommended workflow:

1. Start with approximate values from physical layout.
2. Visualize in RViz.
3. Compare with real robot photos.
4. Adjust mount pose.
5. Later measure with ruler / CAD / calibration.
6. Freeze the transform into `model_variants.yaml` or a xacro argument.

---

## 9. MS42DC Modeling Strategy

Because a public ROS/URDF model was not found, use a staged approach.

### Stage A: Placeholder Model

Use primitive geometry:

```text
box for base
cylinders or boxes for fingers
virtual grasp_frame
```

Purpose:

- TF correctness
- MoveIt attachment
- dashboard display
- early grasp pose planning

### Stage B: Measured Model

Use real dimensions:

- base width / height / depth
- flange hole pattern
- finger length
- maximum opening
- fingertip position
- TCP offset
- grasp center offset

Purpose:

- better collision checking
- better grasp planning
- better digital twin

### Stage C: CAD Mesh Model

Use manufacturer CAD/STL/STEP if available.

Purpose:

- high-quality visualization
- accurate Web digital twin
- presentation/demo quality

### Stage D: Control Model

Expose gripper control as:

```text
open
close
position
speed
force/current if available
```

Possible ROS interfaces:

```text
std_srvs/Trigger for earliest open/close
control_msgs/action/GripperCommand for standard gripper control
ros2_control hardware_interface for mature integration
```

---

## 10. Control Architecture

Early control should remain separated:

```text
controller_manager
├── joint_state_broadcaster
├── aubo_arm_controller
├── scout_base_controller
└── ms42dc_gripper_controller
```

Suggested controller types:

```text
Aubo i5:
  joint_trajectory_controller/JointTrajectoryController

Scout 2.0:
  diff_drive_controller/DiffDriveController
  or native scout_base cmd_vel interface first

MS42DC:
  gripper action controller later
  simple service/action wrapper first
```

Key rule:

> Do not build a monolithic whole-body controller at the beginning.

---

## 11. Planning Architecture

### 11.1 MoveIt2 Groups

Eventually define these SRDF groups:

```text
aubo_arm:
  aubo_joint_1 ... aubo_joint_6

gripper:
  ms42dc main/opening joint or virtual mimic joints

mobile_base:
  planar virtual joint or base group

whole_body:
  mobile_base + aubo_arm
```

### 11.2 Recommended Order

```text
Phase 1:
  aubo_arm only

Phase 2:
  aubo_arm + gripper

Phase 3:
  aubo_arm + gripper + Scout TF

Phase 4:
  task-level mobile manipulation

Phase 5:
  optional whole-body planning / whole-body servo
```

### 11.3 Do Not Start With Whole-Body Planning

Whole-body planning introduces:

- nonholonomic base constraints
- base-arm synchronization
- collision with environment
- multiple controller timing domains
- local minima
- unstable execution

Start with task-level coordination:

```text
Nav2 moves base near target.
MoveIt2 moves arm.
Gripper grasps.
Task manager coordinates them.
```

---

## 12. Simulation Strategy

Simulation is important, but it comes **after the model is correct**.

### 12.1 Backends

Candidate backends:

```text
MuJoCo:
  preferred for dynamics, contact, manipulation experiments

Gazebo / Ignition / Gazebo Sim:
  useful for ROS-native mobile base and Nav2 examples

Isaac Sim:
  optional future backend for synthetic data / high-end visual simulation
```

### 12.2 MuJoCo Integration

Relevant repositories:

- `ros-controls/mujoco_ros2_control`
- `dfki-ric/mujoco_ros2_control`
- `ros-controls/mujoco_ros2_simulation`

Goal:

```text
Use the same ROS2 controller interface for real and simulated robot where possible.
```

### 12.3 Model Rule

```text
URDF/Xacro is the primary source of kinematic truth.
MuJoCo/MJCF is a simulation backend representation.
Frame and joint names must remain aligned.
```

---

## 13. Web / Digital Twin Strategy

Web UI is not the first milestone.

It should start after:

```text
robot_description is stable
TF tree is stable
basic ROS2 topics exist
```

Recommended stack:

```text
Frontend:
  React
  Three.js / React Three Fiber
  TypeScript

ROS-Web bridge:
  foxglove_bridge for visualization and streaming
  rosbridge_suite for simple command/control APIs
  custom FastAPI/WebSocket node only if necessary
```

Early Web dashboard features:

```text
show robot state
show joint states
show TF/frame status
show camera stream later
send high-level commands: home, open, close, demo
```

Do not use Web UI as the low-level control loop.

---

## 14. Repository Plan

Recommended root structure:

```text
Arachne/
├── README.md
├── plan.md
├── src/
│   ├── arachne_description/
│   ├── arachne_bringup/
│   ├── arachne_control/
│   ├── arachne_moveit_config/
│   ├── arachne_gripper/
│   ├── arachne_sim_mujoco/
│   ├── arachne_nav/
│   └── arachne_web_bridge/
├── web/
│   └── arachne_dashboard/
├── third_party/
│   └── README.md
├── scripts/
│   ├── setup_ubuntu.sh
│   ├── check_model.sh
│   └── check_tf.sh
└── docs/
    ├── hardware.md
    ├── modeling.md
    ├── calibration.md
    ├── control.md
    └── references.md
```

---

## 15. Development Phases

### Phase 0: Hardware and Source Audit

Goal:

```text
Collect all available CAD, URDF, manuals, SDKs, dimensions, wiring docs.
```

Tasks:

- identify exact Aubo i5 controller version
- identify Scout 2.0 protocol/CAN setup
- identify MS42DC communication method
- download MS42DC SDK/manual if available
- check if MS42DC CAD/STL/STEP is available
- photograph physical mounting
- measure approximate mounting offsets

Deliverables:

```text
docs/hardware.md
docs/references.md
raw vendor docs stored outside repo if license-restricted
```

---

### Phase 1: Unified Robot Model

Goal:

```text
Build arachne_description.
```

Tasks:

- import/adapt Scout 2.0 model
- import/adapt Aubo i5 model
- build custom MS42DC model
- define Scout-to-Aubo fixed mount
- define Aubo-to-MS42DC fixed adapter
- define grasp frame
- define optional camera/lidar frames
- launch in RViz
- verify TF tree

Deliverables:

```text
arachne_description
RViz display launch
valid URDF output
TF tree PDF/SVG
```

---

### Phase 2: MoveIt2 Model and Arm Planning

Goal:

```text
Plan and visualize Aubo i5 motion inside the complete Scout-Aubo model.
```

Tasks:

- generate MoveIt2 config from unified model
- define `aubo_arm` group
- define end effector frame
- set collision matrix
- set joint limits
- test planning in RViz

Deliverables:

```text
arachne_moveit_config
working arm planning demo
```

---

### Phase 3: Gripper Abstraction

Goal:

```text
Make MS42DC usable as a ROS2 end effector.
```

Tasks:

- inspect SDK/C demo
- determine communication: serial/CAN/RS485/etc.
- write simple ROS2 wrapper
- expose open/close service
- expose position command if supported
- publish gripper state if readable
- later upgrade to GripperCommand action

Deliverables:

```text
arachne_gripper
open/close demo
MS42DC frame connected to tool0
```

---

### Phase 4: Real Aubo i5 Control

Goal:

```text
Move Aubo i5 through ROS2.
```

Tasks:

- test official Aubo ROS2 driver if compatible
- verify joint state names match model
- verify trajectory interface
- integrate with MoveIt2
- create safe home/demo motions

Deliverables:

```text
real_aubo.launch.py
MoveIt2 execute trajectory demo
```

Fallback:

```text
If official driver is unstable, wrap Aubo SDK into a ros2_control hardware_interface.
```

---

### Phase 5: Scout 2.0 Bringup

Goal:

```text
Control Scout 2.0 from ROS2.
```

Tasks:

- install CAN interface
- bring up scout_ros2
- verify `/cmd_vel`
- verify odometry
- verify base TF
- verify safety stop behavior

Deliverables:

```text
real_scout.launch.py
cmd_vel teleop demo
odom/base_link TF validated
```

---

### Phase 6: Task-Level Mobile Manipulation

Goal:

```text
Coordinate base + arm + gripper at task level.
```

Tasks:

- create task manager node
- sequence base movement, arm planning, gripper action
- add perception target later
- add safety checks

Deliverables:

```text
pick_demo.launch.py
simple mobile manipulation sequence
```

---

### Phase 7: Simulation Backend

Goal:

```text
Connect model to simulation.
```

Tasks:

- start with visualization-only simulation
- test Gazebo/Gazebo Sim if Scout Nav2 requires it
- test MuJoCo for manipulation/contact
- align joint names and frames
- add ros2_control simulation backend

Deliverables:

```text
arachne_sim_mujoco
sim_scout_aubo.launch.py
real/sim interface notes
```

---

### Phase 8: Web Dashboard

Goal:

```text
Build browser-based dashboard for monitoring and high-level commands.
```

Tasks:

- stream robot state
- visualize TF/URDF model
- display joint state
- display camera later
- send safe high-level commands

Deliverables:

```text
web/arachne_dashboard
arachne_web_bridge
```

---

## 16. Immediate To-Do List

Highest priority:

```text
1. Create arachne_description package.
2. Import or adapt Aubo i5 description.
3. Import or adapt Scout 2.0 description.
4. Create placeholder MS42DC xacro.
5. Define base_link -> arm_mount_link -> aubo_base_link.
6. Define tool0 -> ms42dc_base_link -> grasp_frame.
7. Display full robot in RViz.
8. Validate TF tree.
```

Before writing control code:

```text
Do not start Web UI.
Do not start MuJoCo integration.
Do not write task manager.
Do not tune Nav2.
Do not implement whole-body planning.
```

---

## 17. Modeling Acceptance Checklist

A model is acceptable only when:

- [ ] all links have consistent names
- [ ] all joints have consistent names
- [ ] no disconnected TF branches
- [ ] mesh scale is correct
- [ ] Aubo base is mounted on Scout, not world
- [ ] MS42DC is mounted on tool0/flange, not world
- [ ] grasp frame is defined
- [ ] collision geometry is simplified
- [ ] visual geometry is presentation-ready or clearly marked placeholder
- [ ] xacro arguments can adjust mount pose
- [ ] generated URDF passes validation
- [ ] RViz display works from a single launch file

---

## 18. Non-Goals for Early Development

Do not prioritize:

```text
Windows native support
perfect Web dashboard
full whole-body planning
full digital twin aesthetics
high-fidelity soft gripper physics
real-time Web low-level control
photorealistic simulation
multi-robot coordination
```

Early success is:

```text
A correct model.
A clean TF tree.
A working arm planning group.
A simple gripper abstraction.
A controllable base.
```

---

## 19. Reference Repositories

### Required / Primary

```text
AuboRobot/aubo_description
https://github.com/AuboRobot/aubo_description

AuboRobot/aubo_robot
https://github.com/AuboRobot/aubo_robot

agilexrobotics/scout_ros2
https://github.com/agilexrobotics/scout_ros2

agilexrobotics/ugv_sdk
https://github.com/agilexrobotics/ugv_sdk

ros-controls/ros2_control
https://github.com/ros-controls/ros2_control

ros-controls/ros2_controllers
https://github.com/ros-controls/ros2_controllers

moveit/moveit2
https://github.com/moveit/moveit2
```

### Useful Modeling References

```text
ian-chuang/LARA_AUBOi5_AG95
https://github.com/ian-chuang/LARA_AUBOi5_AG95

hai-h-nguyen/aubo_i5_robot
https://github.com/hai-h-nguyen/aubo_i5_robot

hai-h-nguyen/aubo-i5-full
https://github.com/hai-h-nguyen/aubo-i5-full

PickNikRobotics/ros2_robotiq_gripper
https://github.com/PickNikRobotics/ros2_robotiq_gripper
```

### Mobile Base / Navigation References

```text
AIRLab-POLIMI/scout_nav2
https://github.com/AIRLab-POLIMI/scout_nav2

agilexrobotics/scout_ros
https://github.com/agilexrobotics/scout_ros
```

### Simulation References

```text
ros-controls/mujoco_ros2_control
https://github.com/ros-controls/mujoco_ros2_control

dfki-ric/mujoco_ros2_control
https://github.com/dfki-ric/mujoco_ros2_control

ros-controls/mujoco_ros2_simulation
https://github.com/ros-controls/mujoco_ros2_simulation
```

### Web / Visualization References

```text
foxglove/ros-foxglove-bridge
https://github.com/foxglove/ros-foxglove-bridge

RobotWebTools/rosbridge_suite
https://github.com/RobotWebTools/rosbridge_suite
```

---

## 20. Final Architecture Principle

> Model first. Control second. Simulation third. Web last.

Arachne should not begin as a Web demo or a MuJoCo demo.

It should begin as a correct robot model:

```text
Aubo i5 + Scout 2.0 + MS42DC
```

Once this model is stable, every later layer becomes easier:

```text
MoveIt2
ros2_control
Nav2
MuJoCo
Web dashboard
Digital twin
Sim2real / real2sim
```
