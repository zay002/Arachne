<p align="center">
  <img src="docs/demo/arachne.png" alt="Arachne robot system showcase" width="900">
</p>

# Arachne

[English](README.en.md) · [快速启动](#快速启动) · [真机接口](#真机接口) · [文档](#文档)

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20%7C%20Humble-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04%20%7C%2022.04-orange)
![License](https://img.shields.io/badge/license-MIT-green)

Arachne 是一个面向移动操作机器人的 ROS 2 workspace。默认机器人由 Scout 2.0 移动底盘、Aubo i5 机械臂和易爪机器人 MS42DC 二指柔性伺服电机夹爪组成；AG95 作为可切换夹爪模型保留。

项目目标是提供一套可复现的研发基线：统一机器人模型、RViz/Gazebo/Godot 演示、MoveIt2/Nav2/ros2_control 起步配置，以及面向真机的 ROS 接口。

<table>
  <tr>
    <td width="50%" align="center"><img src="docs/demo/realbot_1.jpg" alt="Arachne physical robot front view" width="100%"></td>
    <td width="50%" align="center"><img src="docs/demo/realbot_2.jpg" alt="Arachne physical robot side view" width="100%"></td>
  </tr>
</table>

## 特性

- 统一 URDF/Xacro：Scout 2.0、Aubo i5、MS42DC、AG95、传感器和安装坐标系位于同一 TF 树。
- 可切换夹爪：MS42DC 使用项目作者手动拆分的可动 CAD 部件；AG95 保留为开源对照模型。
- 仿真与展示：RViz 模型检查、Gazebo 手柄 demo、自主拾取验证，以及 Godot 高帧率展示前端。
- 控制骨架：MoveIt2、Nav2、ros2_control、sequence executor 和 VLA/WAM action chunk translator。
- 真机接口：Scout 2.0、MS42DC、Aubo i5 均通过 ROS 话题、launch 和安全检查脚本接入。

## 快速启动

支持 Ubuntu 24.04 + ROS 2 Jazzy，兼容 Ubuntu 22.04 + ROS 2 Humble。

```bash
git clone https://github.com/zay002/Arachne.git
cd Arachne

./scripts/setup_ubuntu.sh
./scripts/fetch_third_party.sh

conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash  # Ubuntu 22.04 使用 /opt/ros/humble/setup.bash

colcon build --base-paths src --packages-select \
  aubo_description scout_description dh_ag95_description \
  arachne_sim arachne_gripper arachne_hardware arachne_control arachne_moveit_config \
  arachne_nav arachne_operator arachne_description arachne_gazebo arachne_demo \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

source install/setup.bash
./scripts/view_model.sh
```

`view_model.sh` 会启动默认 MS42DC 模型、底盘遥控 GUI、Aubo 关节滑条和夹爪 Open/Close 控制窗。

## 常用入口

| 目标 | 命令 |
| --- | --- |
| 查看默认 MS42DC 模型 | `./scripts/view_model.sh` |
| 查看 AG95 模型 | `./scripts/use_gripper.sh ag95 view` |
| 检查 URDF 和基础接口 | `./scripts/check_workspace.sh` |
| Gazebo 手柄 demo | `./scripts/switch_demo.sh` |
| Gazebo 自主拾取验证 | `./scripts/gazebo_autopick_demo.sh` |
| Godot 展示前端 | `./scripts/godot_showcase.sh` |
| 真机环境检查 | `./scripts/check_real_hardware_env.sh` |
| Aubo 只读连通探测 | `./scripts/real_aubo_probe.sh` |
| Aubo 启动状态确认 | `./scripts/real_aubo_prepare.sh` |
| Aubo 真机 driver 启动 | `ARACHNE_CONFIRM_AUBO_DRIVER=YES ./scripts/real_aubo_bringup.sh` |
| Aubo 阻塞远程启动 | `ARACHNE_CONFIRM_AUBO_REMOTE_START=YES ./scripts/real_aubo_remote_start.sh` |
| Aubo 小幅 Z 向测试 | `ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/real_aubo_z_test.sh` |

<p align="center">
  <img src="docs/demo/gazebo.png" alt="Arachne Gazebo demo" width="48%">
  <img src="docs/demo/godot.png" alt="Arachne Godot showcase" width="48%">
</p>

## 真机接口

Arachne 的真机层尽量复用官方或厂家 ROS 路线，并在本仓库内维护当前硬件需要的集成节点。

| 设备 | 默认接口 | 说明 |
| --- | --- | --- |
| Scout 2.0 | `scout_waveshare_serial_driver` | `/cmd_vel` 到 Scout v2 CAN 帧；Waveshare USB-CAN-A，CH340 串口，默认 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` |
| MS42DC | `ms42dc_direct_serial_driver` | `/arachne/gripper/command` 到 Type-C 串口帧；夹具控制板为 CH91xx/CH343 系列，当前实机按 CH9012 路线处理，推荐别名 `/dev/motor_serial` |
| Aubo i5 | `AuboRobot/aubo_ros2_driver` | TCP/IP + ros2_control，按机器人 IP 启动 |

准备真机相关 ROS 包：

```bash
./scripts/prepare_real_hardware_ros.sh
./scripts/real_aubo_probe.sh
./scripts/real_aubo_prepare.sh
```

Aubo 推荐优先在示教器/控制柜上完成“连接 -> 上电 -> 启动”。如果需要从 ROS 侧远程启动，只使用阻塞式脚本：它会先确认 controller active、读取当前关节角并发送 hold-position，再按“上电 -> servo mode 确认 -> 抱闸释放 -> Running 后保持”的顺序执行，任何一步超时或状态异常都会退出。

远程启动需要两个终端：

```bash
# 终端 1：启动 driver，并允许上电前激活 controller
ARACHNE_CONFIRM_AUBO_DRIVER=YES ARACHNE_AUBO_ALLOW_PRESTART=YES ./scripts/real_aubo_bringup.sh

# 终端 2：执行阻塞式远程启动状态机
ARACHNE_CONFIRM_AUBO_REMOTE_START=YES AUBO_ROBOT_IP=192.168.127.128 ./scripts/real_aubo_remote_start.sh
```

按已连接设备启动：

```bash
ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=true \
  scout_driver:=waveshare \
  scout_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  use_ms42dc:=true \
  ms42dc_port:=/dev/motor_serial \
  use_aubo:=false
```

WSL2 用户推荐使用 [hurry-porter](https://github.com/zay002/hurry-porter) 辅助 USB 透传、串口扫描和 Waveshare USB-CAN-A 诊断。

```bash
hurry scan
hurry waveshare-can-a recv \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --can-bitrate 500000 \
  --frame-type standard \
  --duration 2
```

真机运动测试默认 dry-run。确认电源、急停和空间安全后再显式允许运动：

```bash
./scripts/real_hardware_acceptance_test.sh
./scripts/real_aubo_z_test.sh
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/real_hardware_acceptance_test.sh
```

## 项目结构

| 路径 | 内容 |
| --- | --- |
| `src/arachne_description` | 统一机器人模型、RViz 配置、夹爪变体和传感器坐标系 |
| `src/arachne_demo` | Switch Pro 手柄、Gazebo 展厅、自主拾取验证 |
| `src/arachne_hardware` | 真机 bringup、Scout/MS42DC wrapper、安全状态和命令门控 |
| `src/arachne_control` | ros2_control 控制器命名、mock 控制器和硬件 profile |
| `src/arachne_moveit_config` | Aubo i5 + MS42DC/AG95 的 MoveIt2 起步配置 |
| `src/arachne_nav` | Scout Nav2 起步配置 |
| `src/arachne_operator` | 操作员面板、sequence executor、VLA/WAM action chunk translator |
| `godot/arachne_showcase` | Godot 4.x 第三人称展示前端 |
| `docs` | 建模、控制、硬件、标定、阶段报告和参考资料 |

## 文档

- [建模](docs/modeling.zh-CN.md)
- [控制](docs/control.zh-CN.md)
- [硬件](docs/hardware.zh-CN.md)
- [标定](docs/calibration.zh-CN.md)
- [参考资料](docs/references.zh-CN.md)
- [阶段报告](docs/reports)

英文版本位于同名文档，例如 [docs/hardware.md](docs/hardware.md)。

## Roadmap

- 完成 Aubo i5 真机 TCP/IP 与 ros2_control 验证。
- 将 Scout、Aubo、MS42DC 的联合 bringup 固化为安全流程。
- 用 MoveIt2 和 ros2_control 替换 Gazebo 自主拾取验证中的轻量规划器。
- 基于真实里程计、定位和传感器继续完善 Nav2。
- 将 Godot 前端通过 ROS 2 bridge 连接到真实或仿真后端。

## License

本仓库代码使用 [MIT License](LICENSE)。第三方模型、CAD、SDK 和说明书遵循各自来源许可证；来源记录见 [third_party/README.md](third_party/README.md) 和 [docs/references.zh-CN.md](docs/references.zh-CN.md)。
