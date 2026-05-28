# 硬件说明

## 目标系统

Arachne 面向 Scout 2.0 移动底盘、Aubo i5 机械臂和易爪机器人 MS42DC 二指柔性伺服电机夹爪。AG95 保留为可切换的开源夹爪版本，用于对比和演示。

## 模型状态

- Scout 2.0 基于 AgileX `scout_ros2` 中的 Scout v2 描述建模。
- Aubo i5 基于官方 `AuboRobot/aubo_description` 中的 `aubo_i5.urdf` 建模。
- 默认末端执行器是易爪机器人 MS42DC，使用本地 `third_party/MS42DC.step` 和 `third_party/MS42DC_SPLIT` 中由项目作者手动拆分的可动 mesh。
- MS42DC 与 AG95 的差异只出现在 `gripper_adapter_link` 下方；底盘、机械臂、安装件、传感器、启动流程和 Open/Close 夹爪接口都共享。

## 真机 ROS 路线

Arachne 尽量使用官方或厂家 ROS 接口：

- Scout 2.0：使用 AgileX `scout_ros2` 中的 `scout_base`，底层依赖 `ugv_sdk`。公开 ROS2 包通过 CAN 控制 Scout，通常为 `can0`、`500000` 波特率；`/cmd_vel` 为速度命令输入，`/odom`、`/scout_status` 和 `/rc_status` 为反馈。
- MS42DC：使用本地 MS42DC 厂家资料中的 `step_motor` ROS2 包。`motor_node` 独占串口并订阅 `motor_control`；Arachne 的 `ms42dc_official_bridge` 将 `/arachne/gripper/command` 映射为厂家 `step_motor/msg/Motor` 消息。
- Aubo i5：使用 `AuboRobot/aubo_ros2_driver`，通过 TCP/IP 连接机器人控制器，并使用 ros2_control 进行轨迹执行。Arachne 只在官方驱动外提供状态探针和 launch 集成。

统一真机启动入口：

```bash
ros2 launch arachne_hardware real_bringup.launch.py
```

每个硬件子系统都可以独立关闭：`use_scout:=false`、`use_ms42dc:=false` 或 `use_aubo:=false`。

## 原生 Linux 与 WSL2

真机 ROS 层设计为可在原生 Linux 和 WSL2 中运行，但硬件可见性不同：

- Aubo TCP/IP 走网络，只要控制器 IP 可达，两种环境都能用。
- MS42DC 串口需要 Linux 下存在 `/dev/motor_serial`、`/dev/ttyUSB*` 或 `/dev/ttyACM*`。WSL2 用户需要先从 Windows 侧透传 USB 设备。
- Scout CAN 需要 SocketCAN 接口，例如 `can0`。原生 Linux 通常使用 `gs_usb` 或类似 USB-CAN 适配器。WSL2 下需要用 `usbipd-win` 挂载适配器，并且 WSL2 内核需要包含对应 USB-CAN 驱动。

运动测试前先运行环境检查：

```bash
./scripts/check_real_hardware_env.sh
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
