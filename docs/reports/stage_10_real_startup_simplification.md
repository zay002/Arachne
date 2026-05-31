# Stage 10: Real Startup Simplification

## Goal

Reduce daily real-hardware demos from multi-terminal, multi-argument launch commands to stable entry scripts, especially after WSL2 restarts and USB devices need to be reattached.

## Core Files

- `scripts/real_bringup.sh`: loads the ROS environment, detects Scout and MS42DC serial ports, checks Aubo state, and starts `arachne_hardware real_bringup.launch.py`.
- `scripts/real_teach_demo.sh`: starts `real_bringup.sh`, waits for `/odom`, `/joint_states`, the Aubo action, and gripper status, then opens the teach/replay panel; closing the panel stops bringup.
- `scripts/check_real_hardware_env.sh`: reuses MS42DC candidate detection so a missing `/dev/motor_serial` alias is no longer a misleading warning.
- `README.en.md` / `docs/hardware.md`: document the one-command real-hardware entries while keeping environment-variable overrides.

## File Relationships

The lower-level `real_bringup.launch.py` still exposes full parameters for debugging. The new scripts encode the lab defaults and print `hurry-porter` hints when serial devices are missing.
