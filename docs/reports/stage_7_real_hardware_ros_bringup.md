# Stage 7: Real-Hardware ROS Bringup

## Goal

Move the project toward physical hardware while keeping one ROS-facing control contract across simulation and real devices.

## Core Files

- `src/arachne_hardware/launch/real_bringup.launch.py`: integrated launch entry for Scout 2.0, MS42DC, and Aubo i5. Each subsystem can be enabled or disabled independently.
- `src/arachne_hardware/config/real_hardware.yaml`: concise hardware contract and default device parameters.
- `src/arachne_hardware/arachne_hardware/scout_waveshare_serial_driver.py`: default Scout base driver for the deployed Waveshare USB-CAN-A adapter. It maps `/cmd_vel` to Scout v2 CAN frames and publishes `/odom`.
- `src/arachne_hardware/arachne_hardware/base_serial_driver.py`: optional status bridge when using AgileX `scout_base` over SocketCAN.
- `src/arachne_hardware/arachne_hardware/gripper_serial_driver.py`: MS42DC command bridge. It maps `/arachne/gripper/command` to the vendor `step_motor/msg/Motor` topic.
- `src/arachne_hardware/arachne_hardware/aubo_tcp_driver.py`: Aubo connectivity/status probe around the official ROS2 driver.
- `scripts/real_aubo_bringup.sh`: confirmed entry for the official Aubo ROS2 driver. In prestart mode it allows controllers to activate before the arm reaches `Running`.
- `scripts/real_aubo_remote_start.sh` and `scripts/real_aubo_remote_start.py`: blocking Aubo remote startup flow. It reads measured joints, sends hold-position goals, calls `RobotManage.poweron`, then calls the full `RobotManage.startup` lifecycle and verifies steady hold after `Running`.
- `scripts/prepare_real_hardware_ros.sh`: links official/vendor ROS packages into `src/vendor`.
- `scripts/prepare_ms42dc_ros2.sh`: extracts the local MS42DC vendor ROS2 package and exposes `serial` and `step_motor`.
- `scripts/fetch_third_party.sh`: pins third-party repositories and applies the Aubo driver safety patch used for prestart controller activation.
- `scripts/check_real_hardware_env.sh`: checks native Linux or WSL2 readiness for ROS tools, vendor links, serial devices, Scout USB-CAN-A or SocketCAN, and Aubo TCP/IP.

## Package Relationships

Scout control now defaults to a direct Waveshare USB-CAN-A wrapper because the real adapter appears as a CH340 serial device in WSL2. The official AgileX `scout_ros2`/`ugv_sdk` SocketCAN path remains available as an alternate launch mode. MS42DC serial control defaults to Arachne's direct Type-C serial driver, with the vendor `step_motor` node kept as a fallback. Aubo motion control stays inside `AuboRobot/aubo_ros2_driver`, which provides ros2_control over TCP/IP.

## Notes

The Aubo remote-start root cause has been identified: calling `releaseRobotBrake` directly is not equivalent to the controller's full startup operation. Arachne now uses `RobotManage.startup` and keeps command targets synchronized to RTDE `actual_q` until the robot reports `Running`. If a tracking-precision fault or `ProtectiveStop` appears, the ROS driver should be stopped and the protective state cleared from the teach pendant/control cabinet before retrying.

The remaining work is real-machine validation: Scout command-mode testing through the Waveshare USB-CAN-A adapter, MS42DC serial alias and safe travel calibration, Aubo startup/motion retest after protective-state recovery, and motion safety checks before enabling autonomous routines. WSL2 can be used for development and network control, but USB serial and USB-CAN devices must be explicitly passed through and verified before launch.
