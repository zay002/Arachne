# Stage 7：真机 ROS Bringup

## 目标

让项目向真实硬件靠近，同时保持仿真和真机共享同一个面向 ROS 的控制契约。

## 核心文件

- `src/arachne_hardware/launch/real_bringup.launch.py`：Scout 2.0、MS42DC 和 Aubo i5 的统一 launch 入口。每个子系统都可以独立启用或关闭。
- `src/arachne_hardware/config/real_hardware.yaml`：简明硬件契约和默认设备参数。
- `src/arachne_hardware/arachne_hardware/scout_waveshare_serial_driver.py`：当前 Waveshare USB-CAN-A 适配器的默认 Scout 底盘驱动。它把 `/cmd_vel` 映射为 Scout v2 CAN 帧并发布 `/odom`。
- `src/arachne_hardware/arachne_hardware/base_serial_driver.py`：使用 AgileX `scout_base`/SocketCAN 时的可选状态桥。
- `src/arachne_hardware/arachne_hardware/gripper_serial_driver.py`：MS42DC 命令桥。它把 `/arachne/gripper/command` 映射到厂家 `step_motor/msg/Motor` 话题。
- `src/arachne_hardware/arachne_hardware/aubo_tcp_driver.py`：围绕官方 ROS2 驱动的 Aubo 连通性/状态探针。
- `scripts/real_aubo_bringup.sh`：官方 Aubo ROS2 driver 的确认入口。prestart 模式允许 controller 在机械臂进入 `Running` 前先激活。
- `scripts/real_aubo_remote_start.sh` 与 `scripts/real_aubo_remote_start.py`：阻塞式 Aubo 远程启动流程。它读取实测关节角、发送 hold-position、调用 `RobotManage.poweron`，再调用完整 `RobotManage.startup` 生命周期，并在 `Running` 后验证稳定保持。
- `scripts/prepare_real_hardware_ros.sh`：把官方/厂家 ROS 包链接到 `src/vendor`。
- `scripts/prepare_ms42dc_ros2.sh`：解压本地 MS42DC 厂家 ROS2 包，并暴露 `serial` 和 `step_motor`。
- `scripts/fetch_third_party.sh`：固定第三方仓库版本，并给 Aubo driver 应用 prestart controller 激活所需的安全补丁。
- `scripts/check_real_hardware_env.sh`：检查原生 Linux 或 WSL2 对 ROS 工具、vendor 链接、串口设备、Scout USB-CAN-A 或 SocketCAN 和 Aubo TCP/IP 的准备情况。

## 包关系

由于实机 Waveshare USB-CAN-A 在 WSL2 中表现为 CH340 串口，Scout 控制现在默认使用直接串口 wrapper；AgileX `scout_ros2`/`ugv_sdk` 的 SocketCAN 路径仍作为备选 launch 模式保留。MS42DC 串口控制默认使用 Arachne 的 Type-C 直连驱动，厂家 `step_motor` 节点作为 fallback 保留。Aubo 运动控制留在 `AuboRobot/aubo_ros2_driver` 内，该驱动通过 TCP/IP 提供 ros2_control。

## 说明

Aubo 远程启动的根因已经定位：直接调用 `releaseRobotBrake` 不等价于控制器完整启动操作。Arachne 现在改用 `RobotManage.startup`，并在机器人报告 `Running` 前持续把命令目标同步到 RTDE `actual_q`。如果出现关节跟踪精度错误或 `ProtectiveStop`，应先停止 ROS driver，再从示教器/控制柜侧清除保护状态后重试。

剩余工作是真机验证：通过 Waveshare USB-CAN-A 测试 Scout 指令控制模式、MS42DC 串口别名和安全行程标定、Aubo 保护状态恢复后的启动/小幅运动复测，以及启用自治例程前的运动安全检查。WSL2 可用于开发和网络控制，但 USB 串口和 USB-CAN 设备必须显式透传并验证后才能启动。
