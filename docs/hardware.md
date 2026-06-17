# Hardware Notes

## Target System

Arachne targets a Scout 2.0 mobile base, an Aubo i5 arm, and a Yizhua Robot MS42DC two-finger flexible servo gripper. AG95 remains as an interchangeable open-source gripper variant for demos and comparison.

## Model State

- Scout 2.0 is modeled from the AgileX Scout v2 description in `scout_ros2`.
- Aubo i5 is modeled from the official `AuboRobot/aubo_description` `aubo_i5.urdf`.
- The default end effector is the Yizhua Robot MS42DC, using local `third_party/MS42DC.step` plus user-created movable split meshes in `third_party/MS42DC_SPLIT`.
- The MS42DC and AG95 variants differ only below `ee_camera_support_link`; the base, arm, wrist-mounted support, sensors, launch flow, and Open/Close gripper interface are shared.

## Real-Hardware ROS Path

Arachne uses official or vendor ROS interfaces wherever they are stable:

- Scout 2.0: Arachne defaults to `scout_waveshare_serial_driver`, which writes Scout v2 CAN frames through a Waveshare USB-CAN-A CH340 serial adapter. It accepts `/cmd_vel`, configures the adapter for `500000` bit/s standard CAN frames, and publishes `/odom` plus `/arachne/hardware/base_status`. The official AgileX `scout_base`/SocketCAN path is still available with `scout_driver:=official`.
- MS42DC: Arachne defaults to `ms42dc_direct_serial_driver`, which keeps the ROS topic surface `/arachne/gripper/command` and writes the documented Type-C USB serial frames directly. This is not the CH340 device; it is the gripper controller's CH91xx/CH343-family USB serial path. Treat the current unit's CH9012 identification as this gripper path; vendor docs may also show CH9102 or `ttyCH343USB*`. The local vendor `step_motor` package remains available through `ms42dc_driver:=vendor`.
- Aubo i5: `AuboRobot/aubo_ros2_driver`, using TCP/IP to the robot controller and ros2_control for trajectory execution. Arachne keeps only a status probe and launch integration around the official driver.

Current physical wiring:

- MS42DC gripper: direct USB Type-C serial connection, preferably exposed through the stable `/dev/motor_serial` alias. This is the gripper controller's CH91xx/CH343-family path, not the base CH340.
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
- MS42DC serial requires a Linux serial device such as `/dev/motor_serial`, `/dev/ttyACM*`, or `/dev/ttyCH343USB*`. WSL2 users must pass the gripper's CH91xx/CH343-family USB serial device through from Windows first.
- Scout defaults to Waveshare USB-CAN-A serial mode, which works in WSL2 after the CH340 device is attached with `usbipd-win`. Native Linux users may instead use a SocketCAN adapter by launching with `scout_driver:=official scout_port:=can0`.

[hurry-porter](https://github.com/zay002/hurry-porter) is recommended for WSL2/Windows device handoff and serial diagnostics. It can list Windows-side USB devices, suggest `usbipd-win` attach commands, and provides `hurry waveshare-can-a` for configuring, sending, and receiving Waveshare USB-CAN-A CAN2.0A/B frames. Arachne does not require it at runtime, but it is useful while bringing up real hardware.

Run the environment checker before motion tests:

```bash
./scripts/hardware/check_real_hardware_env.sh
./scripts/hardware/real_aubo_probe.sh
```

Use fixed scripts for isolated Aubo testing. `real_aubo_bringup.sh` starts the official ROS2 driver; because this is a real-hardware control mode, it requires explicit confirmation:

```bash
./scripts/hardware/real_aubo_prepare.sh
ARACHNE_CONFIRM_AUBO_DRIVER=YES ./scripts/hardware/real_aubo_bringup.sh
```

Prefer completing connect -> power on -> start from the teach pendant/control cabinet, then use `real_aubo_prepare.sh` as a read-only state check: `SafetyMode` must be `Normal` or `ReducedMode`, and `RobotMode` must be `Running`.

If ROS-side remote startup is required, use only the blocking state-machine script. The flow does not skip steps: it waits for `joint_state_broadcaster` and `joint_trajectory_controller` to be active, reads the measured joint angles, sends a hold-position action, then runs power on -> wait for Idle/Running -> hold again -> call the Aubo `RobotManage.startup` lifecycle startup -> wait for Running -> joint steady-state check -> final hold verification. The script never calls `releaseRobotBrake` directly because releasing the brake alone is not the full startup path and may let a joint drop before servo hold is established. Missing controllers, failed actions, unsafe states, and timeouts abort the flow. `fetch_third_party.sh` applies an Arachne patch to the pinned Aubo driver so command interfaces initialize from RTDE `actual_q`, non-Running states keep the command target synchronized to the measured joints, no `servoJoint` is sent before Running, and all-zero joint commands are rejected when the measured pose is non-zero.

If the teach pendant reports a joint tracking precision fault or `ProtectiveStop`, do not keep clearing faults or retrying startup remotely. Stop the ROS driver first, physically support the arm, verify the workspace, and clear the protective state from the teach pendant/control cabinet.

Remote startup uses two terminals:

```bash
# Terminal 1
ARACHNE_CONFIRM_AUBO_DRIVER=YES ARACHNE_AUBO_ALLOW_PRESTART=YES ./scripts/hardware/real_aubo_bringup.sh

# Terminal 2
ARACHNE_CONFIRM_AUBO_REMOTE_START=YES AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/real_aubo_remote_start.sh
```

Run the small Z test in another terminal. It is dry-run by default; real motion requires confirmation:

```bash
./scripts/hardware/real_aubo_z_test.sh
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/hardware/real_aubo_z_test.sh
```

## Real-Hardware Acceptance Test

After power, networking, serial, and CAN are stable, Arachne provides a conservative acceptance test:

Real-hardware actions should be captured as executable scripts instead of relying on ad-hoc terminal snippets. Each script should define the motion goal, default parameters, safety confirmation, and observable output so debugging, demos, and repeated experiments behave consistently. Temporary commands are reserved for quick checks such as `ping`, port probing, and read-only state queries.

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
./scripts/hardware/check_real_hardware_env.sh --strict
```

Bring up the connected hardware in one terminal. For daily use, prefer the automatic entry; it selects the lab-default serial ports and checks Aubo state:

```bash
./scripts/hardware/real_bringup.sh
```

If WSL2 restarts and serial devices disappear, the script first tries to auto-attach the CH9102/CH340 devices through `hurry`; if Windows has not shared them yet, follow the printed `hurry scan` / `hurry attach <BUSID>` hint. For native SocketCAN adapters, use `SCOUT_DRIVER=official SCOUT_PORT=can0 ./scripts/hardware/real_bringup.sh`.

For teach demonstrations, use:

```bash
./scripts/hardware/real_teach_demo.sh
```

This starts bringup, waits for the core topics and Aubo action, then opens the teach/replay panel; closing the panel stops the background bringup. Teach JSON files are saved locally under `recordings/teach/` by default. Base hold-to-drive operations are stored as relative forward/backward distance or left/right turn waypoints when the button is released, and replay uses slow safety defaults.

For isolated MS42DC calibration, start with a small command before using the full factory stroke. The current small-angle test value is `300` tenths of a degree (`30 deg`) and the default demo speed is `150` tenths of rad/s (`15 rad/s`). The factory full open/close example is `18720` tenths of a degree, or `1872 deg = 5.2 turns`, and should only be used after physical travel and homing behavior are confirmed:

```bash
ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=false use_aubo:=false use_ms42dc:=true \
  ms42dc_driver:=direct \
  ms42dc_port:=/dev/motor_serial \
  ms42dc_open_angle_tenths:=300 \
  ms42dc_close_angle_tenths:=300 \
  ms42dc_speed_tenths:=150
```

Dry-run the test entry in another terminal:

```bash
./scripts/hardware/real_hardware_acceptance_test.sh
```

Run real motion only when the robot is clear and an emergency stop or power cut is within reach:

```bash
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/hardware/real_hardware_acceptance_test.sh
```

Individual subsystems can be isolated:

```bash
./scripts/hardware/real_base_test.sh
./scripts/hardware/real_arm_test.sh
./scripts/hardware/real_gripper_test.sh
```

These wrappers are also dry-run by default. To move only one subsystem:

```bash
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/hardware/real_base_test.sh
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/hardware/real_arm_test.sh
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/hardware/real_gripper_test.sh
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
