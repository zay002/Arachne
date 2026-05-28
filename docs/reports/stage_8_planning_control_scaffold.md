# Stage 8: Planning And Control Scaffold

## Goal

Prepare the parts that can be developed before real hardware arrives: MoveIt2, ros2_control, Nav2, safety state handling, mock hardware, automated checks, and a lightweight operator panel.

## Core Files

- `src/arachne_control/`: shared controller names, `ros2_controllers.yaml`, sim/mock/real profiles, and `mock_ros2_control.launch.py`.
- `src/arachne_moveit_config/`: MoveIt2 starter SRDFs for MS42DC and AG95, named Aubo poses, KDL IK, OMPL planning, and controller mapping.
- `src/arachne_nav/`: Nav2 starter params, empty map, and `nav2_sim.launch.py` with mock base and mock map-to-odom support.
- `src/arachne_hardware/arachne_hardware/safety_state_machine.py`: manual/autonomous/disabled/estop state services.
- `src/arachne_hardware/arachne_hardware/safety_cmd_vel_gate.py`: optional gated `/cmd_vel` path.
- `src/arachne_hardware/arachne_hardware/hardware_mock.py`: no-hardware publisher for odom, joint states, and hardware status.
- `src/arachne_operator/`: Tk operator status panel plus `sequence_executor.py` for arm presets, gripper commands, demo sequences, and Nav2 goals.
- `scripts/check_workspace.sh`: one-command syntax, Xacro, SRDF, build, and launch smoke check.

## Relationships

The mock hardware publishes the same high-level state expected from the real bringup. MoveIt2 and ros2_control share the Aubo and gripper joint names from `arachne_description`. Nav2 uses the same `/cmd_vel` and `/odom` contract as RViz, Gazebo, and real Scout bringup; its sim launch adds a mock `map -> odom` transform until localization or SLAM is connected. The operator panel watches these shared status topics, and the sequence executor maps simple high-level commands onto the same low-level contracts.

## Next Work

The next pass should validate the MoveIt2 planning groups in RViz, tune ros2_control controller behavior, confirm Nav2 costmaps with real or simulated scan data, and decide which commands must be routed through the safety gate before real hardware motion is enabled.
