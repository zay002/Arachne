# Hardware Notes

## Target System

Arachne targets a Scout 2.0 mobile base, an Aubo i5 arm, and a Yizhua Robot MS42DC two-finger flexible servo gripper. AG95 remains as an interchangeable open-source gripper variant for demos and comparison.

## Model State

- Scout 2.0 is modeled from the AgileX Scout v2 description in `scout_ros2`.
- Aubo i5 is modeled from the official `AuboRobot/aubo_description` `aubo_i5.urdf`.
- The default end effector is the Yizhua Robot MS42DC, using local `third_party/MS42DC.step` plus user-created movable split meshes in `third_party/MS42DC_SPLIT`.
- The MS42DC and AG95 variants differ only below `gripper_adapter_link`; the base, arm, mounts, sensors, launch flow, and Open/Close gripper interface are shared.

## Real-Hardware ROS Path

Arachne uses official or vendor ROS interfaces wherever available:

- Scout 2.0: `scout_base` from AgileX `scout_ros2`, backed by `ugv_sdk`. The public ROS2 package controls Scout over CAN, normally `can0` at `500000` bitrate, with `/cmd_vel` as the velocity command input and `/odom`, `/scout_status`, and `/rc_status` as feedback.
- MS42DC: `step_motor` from the local vendor ROS2 package under the MS42DC materials. `motor_node` owns the serial device and subscribes to `motor_control`; Arachne's `ms42dc_official_bridge` maps `/arachne/gripper/command` into the vendor `step_motor/msg/Motor` message.
- Aubo i5: `AuboRobot/aubo_ros2_driver`, using TCP/IP to the robot controller and ros2_control for trajectory execution. Arachne keeps only a status probe and launch integration around the official driver.

The integrated bringup entry is:

```bash
ros2 launch arachne_hardware real_bringup.launch.py
```

Each hardware component can be disabled independently with `use_scout:=false`, `use_ms42dc:=false`, or `use_aubo:=false`.

## Native Linux And WSL2

The real-hardware ROS layer is designed to run on both native Linux and WSL2, but hardware visibility differs:

- Aubo TCP/IP is network-based and works in either environment when the controller IP is reachable.
- MS42DC serial requires a Linux serial device such as `/dev/motor_serial`, `/dev/ttyUSB*`, or `/dev/ttyACM*`. WSL2 users must pass the USB device through from Windows first.
- Scout CAN requires a SocketCAN interface such as `can0`. On native Linux this is normally a `gs_usb` or similar USB-CAN adapter. On WSL2, the adapter must be attached with `usbipd-win`, and the WSL2 kernel must include the matching USB-CAN driver.

Run the environment checker before motion tests:

```bash
./scripts/check_real_hardware_env.sh
```

## Measurements To Confirm

- Exact Scout top-plate mounting holes and usable payload layout.
- Aubo flange-to-MS42DC adapter dimensions.
- Physical MS42DC open/close travel, safe speed, device ID, serial alias, and homing behavior.
- Aubo controller firmware version and official driver compatibility before enabling motion.

These values should be confirmed on the real hardware before planning, collision checking, or autonomous execution is trusted.
