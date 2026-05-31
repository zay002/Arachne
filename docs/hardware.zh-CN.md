# 硬件说明

## 目标系统

Arachne 面向 Scout 2.0 移动底盘、Aubo i5 机械臂和易爪机器人 MS42DC 二指柔性伺服电机夹爪。AG95 保留为可切换的开源夹爪版本，用于对比和演示。

## 模型状态

- Scout 2.0 基于 AgileX `scout_ros2` 中的 Scout v2 描述建模。
- Aubo i5 基于官方 `AuboRobot/aubo_description` 中的 `aubo_i5.urdf` 建模。
- 默认末端执行器是易爪机器人 MS42DC，使用本地 `third_party/MS42DC.step` 和 `third_party/MS42DC_SPLIT` 中由项目作者手动拆分的可动 mesh。
- MS42DC 与 AG95 的差异只出现在 `gripper_adapter_link` 下方；底盘、机械臂、安装件、传感器、启动流程和 Open/Close 夹爪接口都共享。

## 真机 ROS 路线

Arachne 尽量使用稳定的官方或厂家 ROS 接口：

- Scout 2.0：Arachne 默认使用 `scout_waveshare_serial_driver`，通过 Waveshare USB-CAN-A 的 CH340 串口封装直接发送 Scout v2 CAN 帧。它接收 `/cmd_vel`，将适配器配置为 `500000` bit/s 标准 CAN 帧，并发布 `/odom` 和 `/arachne/hardware/base_status`。AgileX 官方 `scout_base`/SocketCAN 路径仍然保留，可用 `scout_driver:=official` 切换。
- MS42DC：默认使用 `ms42dc_direct_serial_driver`，保留 `/arachne/gripper/command` 这个 ROS 话题接口，并按说明书直接写入 Type-C USB 串口帧。夹具不是 CH340 设备，而是夹具控制板的 CH91xx/CH343 系列 USB 串口；当前实机可按你识别到的 CH9012 这一路处理，厂家资料里也常见 CH9102/`ttyCH343USB*` 命名。本地厂家 `step_motor` 路径仍然保留，可用 `ms42dc_driver:=vendor` 切换。
- Aubo i5：使用 `AuboRobot/aubo_ros2_driver`，通过 TCP/IP 连接机器人控制器，并使用 ros2_control 进行轨迹执行。Arachne 只在官方驱动外提供状态探针和 launch 集成。

当前实机接线：

- MS42DC 夹具：USB Type-C 直连串口，推荐稳定别名 `/dev/motor_serial`；这一路是夹具控制板的 CH91xx/CH343 系列设备，不是底盘的 CH340。
- Scout 2.0：Waveshare USB-CAN-A 适配器，进入 Linux 后通常是 CH340 串口 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`。原生 Linux 的 SocketCAN `can0` 是可选备选路径。
- Aubo 控制柜：网线连接。当前控制柜 MAC 识别提示为 `CC:82:7F:A3:E6:2E`；ROS 控制仍然使用配置的机器人 IP。

统一真机启动入口：

```bash
ros2 launch arachne_hardware real_bringup.launch.py
```

每个硬件子系统都可以独立关闭：`use_scout:=false`、`use_ms42dc:=false` 或 `use_aubo:=false`。

## 原生 Linux 与 WSL2

真机 ROS 层设计为可在原生 Linux 和 WSL2 中运行，但硬件可见性不同：

- Aubo TCP/IP 走网络，只要控制器 IP 可达，两种环境都能用。
- MS42DC 串口需要 Linux 下存在 `/dev/motor_serial`、`/dev/ttyACM*` 或 `/dev/ttyCH343USB*`。WSL2 用户需要先从 Windows 侧透传夹具的 CH91xx/CH343 系列 USB 串口设备。
- Scout 默认走 Waveshare USB-CAN-A 串口模式，WSL2 下用 `usbipd-win` 透传 CH340 设备后即可使用。原生 Linux 用户也可以用 SocketCAN 适配器，并通过 `scout_driver:=official scout_port:=can0` 切换。

推荐安装并使用 [hurry-porter](https://github.com/zay002/hurry-porter) 作为 WSL2/Windows 设备透传和串口诊断工具。它可以列出 Windows 侧 USB 设备、提示 `usbipd-win` attach 命令，并提供 `hurry waveshare-can-a` 来配置、发送和接收 Waveshare USB-CAN-A 的 CAN2.0A/B 帧。Arachne 不强依赖它，但实机排障时很方便。

运动测试前先运行环境检查：

```bash
./scripts/check_real_hardware_env.sh
./scripts/real_aubo_probe.sh
```

单独测试 Aubo 时使用固定脚本。`real_aubo_bringup.sh` 会启动官方 ROS2 driver；由于该 driver 在硬件激活阶段会进入 servo mode，必须显式确认：

```bash
ARACHNE_CONFIRM_AUBO_DRIVER=YES ./scripts/real_aubo_bringup.sh
```

另一个终端运行小幅 Z 向测试。默认只 dry-run；真实运动需要确认：

```bash
./scripts/real_aubo_z_test.sh
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/real_aubo_z_test.sh
```

## 真机验收测试

电源、网络、串口和 CAN 都稳定后，可以运行保守的真机验收测试：

真机动作应优先固化为可执行脚本，而不是依赖临时终端命令。脚本需要写清目标动作、默认参数、安全确认和可观察输出，保证后续调试、演示和复现实验时行为一致。临时命令只用于 `ping`、端口探测、只读状态读取这类快速检查。

1. Scout 前进 `0.2 m`。
2. Scout 后退 `0.2 m`。
3. Scout 左转 `30 deg`，再归位。
4. Scout 右转 `30 deg`，再归位。
5. Aubo `tool0` 沿 Aubo 基座 Z 方向上移 `0.2 m`，再回到起始关节姿态。
6. MS42DC 开合 `5` 次，最后保持打开。

测试节点是 [real_hardware_acceptance_test.py](../src/arachne_operator/arachne_operator/real_hardware_acceptance_test.py)。Scout 运动使用 `/odom` 闭环；机械臂读取 `/joint_states`，本地求 Aubo i5 的位置 IK，然后同时发布 `/aubo_arm_controller/joint_trajectory` 和 `/joint_trajectory_controller/joint_trajectory`；夹具使用 `/arachne/gripper/command`。

默认机械臂运动沿 `aubo_base_link` 坐标系的竖直 Z 方向。如果希望沿当前工具 Z 轴移动，可传入 `arm_z_frame:=tool`。

先做主机检查：

```bash
./scripts/check_real_hardware_env.sh --strict
```

一个终端启动已连接硬件，按实际端口和 IP 调整：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch arachne_hardware real_bringup.launch.py \
  scout_driver:=waveshare \
  scout_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  ms42dc_port:=/dev/motor_serial \
  aubo_robot_ip:=192.168.127.128
```

原生 Linux SocketCAN 适配器可将 Scout 参数替换为 `scout_driver:=official scout_port:=can0`。

单独标定 MS42DC 时，建议先用小角度测试，再使用厂家全行程。当前安全默认值是 `300` 个 0.1 度，也就是 `30 deg`；速度是 `60` 个 0.1 rad/s，也就是 `6 rad/s`。说明书中的全开/全闭示例是 `18720` 个 0.1 度，也就是 `1872 deg = 5.2 圈`，需要确认真实行程和回零行为后再使用：

```bash
ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=false use_aubo:=false use_ms42dc:=true \
  ms42dc_driver:=direct \
  ms42dc_port:=/dev/motor_serial \
  ms42dc_open_angle_tenths:=300 \
  ms42dc_close_angle_tenths:=300 \
  ms42dc_speed_tenths:=60
```

另一个终端先 dry-run：

```bash
./scripts/real_hardware_acceptance_test.sh
```

确认机器人周围无障碍、急停或断电手段在手边后，才运行真实运动：

```bash
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/real_hardware_acceptance_test.sh
```

也可以单独测试某个子系统：

```bash
./scripts/real_base_test.sh
./scripts/real_arm_test.sh
./scripts/real_gripper_test.sh
```

这些入口默认也只是 dry-run。只让某个子系统真实运动时：

```bash
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/real_base_test.sh
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/real_arm_test.sh
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/real_gripper_test.sh
```

## Mock 硬件

在物理设备接入前，`arachne_hardware mock_bringup.launch.py` 会发布与真机 bringup 相同的高层状态和状态话题：

- `/odom`
- `/joint_states`
- `/arachne/hardware/base_status`
- `/arachne/hardware/aubo_status`
- `/arachne/hardware/gripper_status`
- `/arachne/safety/state`
- `/arachne/safety/enabled`

这样 MoveIt2、Nav2、operator 面板、安全服务和未来 Web UI 都可以在没有 Scout CAN、MS42DC 串口或 Aubo TCP/IP 的情况下，先围绕稳定 ROS 契约联调。

## 待确认测量

- Scout 顶板安装孔位和可用载荷布局。
- Aubo 法兰到 MS42DC 转接板尺寸。
- MS42DC 真实开闭行程、安全速度、设备 ID、串口别名和回零行为。
- Aubo 控制器固件版本与官方驱动兼容性。

这些值需要在真实硬件上确认后，才能信任规划、碰撞检查或自治执行。
