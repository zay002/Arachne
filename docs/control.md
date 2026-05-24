# Control

Control is deliberately not implemented in the first stage. The model includes optional mock `ros2_control` blocks so future packages can keep names aligned.

Planned controllers:

- `aubo_arm_controller`: joint trajectory controller for `aubo_shoulder_joint`, `aubo_upperArm_joint`, `aubo_foreArm_joint`, `aubo_wrist1_joint`, `aubo_wrist2_joint`, and `aubo_wrist3_joint`.
- `scout_base_controller`: diff drive or native Scout driver bridge.
- `ms42dc_gripper_controller`: gripper command action or a simple open/close wrapper first.

The control layer should remain split by hardware device while sharing the unified robot state.
