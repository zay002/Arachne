# Stage 7：真机 ROS Bringup

## 目标

让项目向真实硬件靠近，同时保持仿真和真机共享同一个面向 ROS 的控制契约。

## 核心文件

- `src/arachne_hardware/launch/real_bringup.launch.py`：Scout 2.0、MS42DC 和 Aubo i5 的统一 launch 入口。每个子系统都可以独立启用或关闭。
- `src/arachne_hardware/config/real_hardware.yaml`：简明硬件契约和默认设备参数。
- `src/arachne_hardware/arachne_hardware/base_serial_driver.py`：Scout 状态桥。真实底盘运动交给 AgileX `scout_base`；Arachne 只观察 `/odom`。
- `src/arachne_hardware/arachne_hardware/gripper_serial_driver.py`：MS42DC 命令桥。它把 `/arachne/gripper/command` 映射到厂家 `step_motor/msg/Motor` 话题。
- `src/arachne_hardware/arachne_hardware/aubo_tcp_driver.py`：围绕官方 ROS2 驱动的 Aubo 连通性/状态探针。
- `scripts/prepare_real_hardware_ros.sh`：把官方/厂家 ROS 包链接到 `src/vendor`。
- `scripts/prepare_ms42dc_ros2.sh`：解压本地 MS42DC 厂家 ROS2 包，并暴露 `serial` 和 `step_motor`。
- `scripts/check_real_hardware_env.sh`：检查原生 Linux 或 WSL2 对 ROS 工具、vendor 链接、串口设备、SocketCAN 和 Aubo TCP/IP 的准备情况。

## 包关系

Scout 控制使用 AgileX `scout_ros2` 和 `ugv_sdk`；Arachne 发送标准 `/cmd_vel` 契约并监听 `/odom`。MS42DC 串口控制留在厂家 `step_motor` 节点内部；Arachne 只转换 Open/Close 命令。Aubo 运动控制留在 `AuboRobot/aubo_ros2_driver` 内，该驱动通过 TCP/IP 提供 ros2_control。

## 说明

剩余工作是真机验证：Scout CAN 适配器设置、MS42DC 串口别名和安全行程标定、Aubo 固件/SDK 兼容性，以及启用自治例程前的运动安全检查。WSL2 可用于开发和网络控制，但 USB 串口和 USB-CAN 设备必须显式透传并验证后才能启动。
