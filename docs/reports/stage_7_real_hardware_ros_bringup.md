# Stage 7: Real-Hardware ROS Bringup

## Goal

Move the project toward physical hardware while keeping one ROS-facing control contract across simulation and real devices.

## Core Files

- `src/arachne_hardware/launch/real_bringup.launch.py`: integrated launch entry for Scout 2.0, MS42DC, and Aubo i5. Each subsystem can be enabled or disabled independently.
- `src/arachne_hardware/config/real_hardware.yaml`: concise hardware contract and default device parameters.
- `src/arachne_hardware/arachne_hardware/base_serial_driver.py`: Scout status bridge. Real base motion is delegated to AgileX `scout_base`; Arachne observes `/odom`.
- `src/arachne_hardware/arachne_hardware/gripper_serial_driver.py`: MS42DC command bridge. It maps `/arachne/gripper/command` to the vendor `step_motor/msg/Motor` topic.
- `src/arachne_hardware/arachne_hardware/aubo_tcp_driver.py`: Aubo connectivity/status probe around the official ROS2 driver.
- `scripts/prepare_real_hardware_ros.sh`: links official/vendor ROS packages into `src/vendor`.
- `scripts/prepare_ms42dc_ros2.sh`: extracts the local MS42DC vendor ROS2 package and exposes `serial` and `step_motor`.
- `scripts/check_real_hardware_env.sh`: checks native Linux or WSL2 readiness for ROS tools, vendor links, serial devices, SocketCAN, and Aubo TCP/IP.

## Package Relationships

Scout control uses AgileX `scout_ros2` and `ugv_sdk`; Arachne sends the normal `/cmd_vel` contract and watches `/odom`. MS42DC serial control stays inside the vendor `step_motor` node; Arachne only translates Open/Close commands. Aubo motion control stays inside `AuboRobot/aubo_ros2_driver`, which provides ros2_control over TCP/IP.

## Notes

The remaining work is real-machine validation: CAN adapter setup for Scout, MS42DC serial alias and safe travel calibration, Aubo firmware/SDK compatibility, and motion safety checks before enabling autonomous routines. WSL2 can be used for development and network control, but USB serial and USB-CAN devices must be explicitly passed through and verified before launch.
