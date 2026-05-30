# Hardware Notes

## Target System

Arachne targets a Scout 2.0 mobile base, an Aubo i5 arm, and a Yizhua Robot MS42DC two-finger flexible servo gripper. AG95 remains as an interchangeable open-source gripper variant for demos and comparison.

## Model State

- Scout 2.0 is modeled from the AgileX Scout v2 description in `scout_ros2`.
- Aubo i5 is modeled from the official `AuboRobot/aubo_description` `aubo_i5.urdf`.
- The default end effector is the Yizhua Robot MS42DC, using local `third_party/MS42DC.step` plus user-created movable split meshes in `third_party/MS42DC_SPLIT`.
- The MS42DC and AG95 variants differ only below `gripper_adapter_link`; the base, arm, mounts, sensors, launch flow, and Open/Close gripper interface are shared.

## Real-Hardware ROS Path

Arachne uses official or vendor ROS interfaces wherever they are stable:

- Scout 2.0: Arachne defaults to `scout_waveshare_serial_driver`, which writes Scout v2 CAN frames through a Waveshare USB-CAN-A CH340 serial adapter. It accepts `/cmd_vel`, configures the adapter for `500000` bit/s standard CAN frames, and publishes `/odom` plus `/arachne/hardware/base_status`. The official AgileX `scout_base`/SocketCAN path is still available with `scout_driver:=official`.
- MS42DC: Arachne defaults to `ms42dc_direct_serial_driver`, which keeps the ROS topic surface `/arachne/gripper/command` and writes the documented Type-C USB serial frames directly. The local vendor `step_motor` package remains available through `ms42dc_driver:=vendor`.
- Aubo i5: `AuboRobot/aubo_ros2_driver`, using TCP/IP to the robot controller and ros2_control for trajectory execution. Arachne keeps only a status probe and launch integration around the official driver.

Current physical wiring:

- MS42DC gripper: direct USB Type-C serial connection.
- Scout 2.0: Waveshare USB-CAN-A adapter exposed as a CH340 serial device, normally `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`. Native SocketCAN `can0` is an optional alternate path.
- Aubo controller cabinet: Ethernet. The current controller MAC hint is `CC:82:7F:A3:E6:2E`; ROS control still uses the configured robot IP.

The integrated bringup entry is:

```bash
ros2 launch arachne_hardware real_bringup.launch.py
```

Each hardware component can be disabled independently with `use_scout:=false`, `use_ms42dc:=false`, or `use_aubo:=false`.

## Native Linux And WSL2

The real-hardware ROS layer is designed to run on both native Linux and WSL2, but hardware visibility differs:

- Aubo TCP/IP is network-based and works in either environment when the controller IP is reachable.
- MS42DC serial requires a Linux serial device such as `/dev/motor_serial`, `/dev/ttyACM*`, or `/dev/ttyCH343USB*`. WSL2 users must pass the CH9102 USB device through from Windows first.
- Scout defaults to Waveshare USB-CAN-A serial mode, which works in WSL2 after the CH340 device is attached with `usbipd-win`. Native Linux users may instead use a SocketCAN adapter by launching with `scout_driver:=official scout_port:=can0`.

[hurry-porter](https://github.com/zay002/hurry-porter) is recommended for WSL2/Windows device handoff and serial diagnostics. It can list Windows-side USB devices, suggest `usbipd-win` attach commands, and provides `hurry waveshare-can-a` for configuring, sending, and receiving Waveshare USB-CAN-A CAN2.0A/B frames. Arachne does not require it at runtime, but it is useful while bringing up real hardware.

Run the environment checker before motion tests:

```bash
./scripts/check_real_hardware_env.sh
```

## Real-Hardware Acceptance Test

After power, networking, serial, and CAN are stable, Arachne provides a conservative acceptance test:

1. Scout forward `0.2 m`.
2. Scout backward `0.2 m`.
3. Scout left yaw `30 deg`, then return.
4. Scout right yaw `30 deg`, then return.
5. Aubo `tool0` moves up `0.2 m` along Aubo-base Z, then returns to the starting joint pose.
6. MS42DC opens and closes `5` cycles, then leaves the gripper open.

The test node is [real_hardware_acceptance_test.py](../src/arachne_operator/arachne_operator/real_hardware_acceptance_test.py). Scout motion is closed over `/odom`; arm motion reads `/joint_states`, solves a local position-only Aubo i5 IK target, and publishes both `/aubo_arm_controller/joint_trajectory` and `/joint_trajectory_controller/joint_trajectory`; the gripper uses `/arachne/gripper/command`.

The default arm move is vertical in `aubo_base_link` coordinates. To move along the current tool Z axis instead, pass `arm_z_frame:=tool`.

First run the host check:

```bash
./scripts/check_real_hardware_env.sh --strict
```

Bring up the connected hardware in one terminal, adjusting ports/IP as needed:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch arachne_hardware real_bringup.launch.py \
  scout_driver:=waveshare \
  scout_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  ms42dc_port:=/dev/motor_serial \
  aubo_robot_ip:=192.168.127.128
```

For SocketCAN adapters on native Linux, replace the Scout arguments with `scout_driver:=official scout_port:=can0`.

For isolated MS42DC calibration, start with a small command before using the full factory stroke. The current safe default is `300` tenths of a degree (`30 deg`) at `60` tenths of rad/s (`6 rad/s`). The factory full open/close example is `18720` tenths of a degree, or `1872 deg = 5.2 turns`, and should only be used after physical travel and homing behavior are confirmed:

```bash
ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=false use_aubo:=false use_ms42dc:=true \
  ms42dc_driver:=direct \
  ms42dc_port:=/dev/motor_serial \
  ms42dc_open_angle_tenths:=300 \
  ms42dc_close_angle_tenths:=300 \
  ms42dc_speed_tenths:=60
```

Dry-run the test entry in another terminal:

```bash
./scripts/real_hardware_acceptance_test.sh
```

Run real motion only when the robot is clear and an emergency stop or power cut is within reach:

```bash
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/real_hardware_acceptance_test.sh
```

Individual subsystems can be isolated:

```bash
./scripts/real_base_test.sh
./scripts/real_arm_test.sh
./scripts/real_gripper_test.sh
```

These wrappers are also dry-run by default. To move only one subsystem:

```bash
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/real_base_test.sh
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/real_arm_test.sh
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/real_gripper_test.sh
```

## Mock Hardware

Before physical devices are attached, `arachne_hardware mock_bringup.launch.py` publishes the same high-level status and state topics used by the real bringup:

- `/odom`
- `/joint_states`
- `/arachne/hardware/base_status`
- `/arachne/hardware/aubo_status`
- `/arachne/hardware/gripper_status`
- `/arachne/safety/state`
- `/arachne/safety/enabled`

This lets MoveIt2, Nav2, the operator panel, safety services, and future Web UI work against stable ROS contracts before Scout CAN, MS42DC serial, or Aubo TCP/IP are available.

## Measurements To Confirm

- Exact Scout top-plate mounting holes and usable payload layout.
- Aubo flange-to-MS42DC adapter dimensions.
- Physical MS42DC open/close travel, safe speed, device ID, serial alias, and homing behavior.
- Aubo controller firmware version and official driver compatibility before enabling motion.

These values should be confirmed on the real hardware before planning, collision checking, or autonomous execution is trusted.
