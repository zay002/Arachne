<p align="center">
  <img src="docs/demo/arachne.png" alt="Arachne robot system showcase" width="900">
</p>

# Arachne

[English](README.en.md) · [快速启动](#快速启动) · [真机入口](#真机入口) · [文档](#文档)

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)
![License](https://img.shields.io/badge/license-MIT-green)

Arachne 是一个 ROS 2 移动操作机器人 workspace，当前主线服务于 Scout 2.0 底盘、Aubo i5 机械臂、MS42DC 夹具、Gemini335 RGB-D 相机和 C16 雷达的真机上位机开发。

项目目标是把底盘、机械臂、夹具、视觉和雷达统一到一个可示教、可验证、可继续接上层任务和学习策略的机器人平台。当前成熟链路包括 Aubo SDK action、Teach Panel、demo orchestrator、grasp task、road cleanup 和 offline regression。

<p align="center">
  <img src="docs/demo/realbot.PNG" alt="Arachne 真机硬件介绍图" width="900">
</p>

## 特性

- 统一模型：Scout、Aubo i5、MS42DC、AG95、Gemini335、C16、车头吊篮和后置架在同一 TF/URDF 树中。
- 真机控制：Scout `/cmd_vel`、MS42DC 串口、Aubo ROS2 driver 状态、Aubo SDK action 执行。
- 示教闭环：Teach Panel 支持底盘、机械臂、夹具、记录和回放。
- 视觉任务：Gemini335 + YOLO/点云 ROI，服务于 grasp task 和 road cleanup。
- 开发闭环：offline regression、dry-run action smoke、demo orchestrator offline smoke 已固化。

## 快速启动

```bash
source scripts/env/arachne_env.sh
./scripts/build/build_workspace.sh
./scripts/model/view_model.sh
```

模型检查优先走 `scripts/model/view_model.sh`。手动运行 ROS 命令时先加载：

```bash
source scripts/env/arachne_env.sh
source install/setup.bash
```

## 推荐工作流

1. 无硬件：`./scripts/build/check_offline_regression.sh`
2. 看模型：`./scripts/model/view_model.sh`
3. 只读硬件：`AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_readonly.sh`
4. 真机 bringup：`./scripts/hardware/real_bringup.sh`
5. 示教器：`./scripts/operator/teach_panel.sh`
6. 任务服务：`./scripts/vision/grasp_task_server.sh` 或 `./scripts/vision/road_cleanup_task_server.sh`

## 当前状态

- Aubo 可进入 `Running / Normal`。
- `/joint_states` 正常。
- `/arachne/aubo/move_joint` action 已接入 SDK `moveJoint`，fallback 保留。
- Teach Panel 可控制 Aubo、底盘和夹具。
- `grasp_task_server`、`road_cleanup_task_server` 和 `demo_orchestrator` 保留为上层任务入口。
- dry-run、offline regression、demo orchestrator smoke test 均通过。

## 主要入口

| 目标 | 命令 |
| --- | --- |
| 查看模型 | `./scripts/model/view_model.sh` |
| 离线回归 | `./scripts/build/check_offline_regression.sh` |
| Aubo action dry-run smoke | `./scripts/test/smoke_aubo_move_joint_dry_run.sh` |
| Demo orchestrator offline smoke | `./scripts/test/smoke_demo_orchestrator_offline.sh` |
| Aubo 只读检查 | `./scripts/hardware/check_aubo_readonly.sh` |
| Aubo Running 只读检查 | `./scripts/hardware/check_aubo_running_readonly.sh` |
| 真机底层启动 | `./scripts/hardware/real_bringup.sh` |
| 真机示教器 | `./scripts/operator/teach_panel.sh` |
| 抓取任务服务器 | `./scripts/vision/grasp_task_server.sh` |
| 道路巡检任务服务器 | `./scripts/vision/road_cleanup_task_server.sh` |
| Gemini335 相机 | `./scripts/vision/gemini335_bringup.sh` |
| Gemini335 YOLO 实时预览 | `./scripts/vision/gemini_yolo_live.sh` |
| AprilTag 导航初始化 | `./scripts/vision/apriltag_nav_initialize.sh` |
| lidar/Nav2 | `./scripts/hardware/real_lidar_nav.sh` |
| 停止真机栈 | `./scripts/hardware/stop_real_stack.sh` |

<p align="center">
  <img src="docs/demo/gazebo.png" alt="Arachne Gazebo demo" width="48%">
  <img src="docs/demo/godot.png" alt="Arachne Godot showcase" width="48%">
</p>

## 真机入口

日常真机启动：

```bash
./scripts/hardware/real_bringup.sh
./scripts/operator/teach_panel.sh
```

Aubo 只读检查：

```bash
AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_readonly.sh
AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_running_readonly.sh
```

设备默认接口：

| 设备 | 默认接口 |
| --- | --- |
| Scout 2.0 | `/cmd_vel`，Waveshare USB-CAN-A |
| MS42DC | `/arachne/gripper/command`，`/arachne/hardware/gripper_status` |
| Aubo i5 | TCP `30004`，`/joint_states`，`/arachne/aubo/move_joint` |
| Gemini335 | 末端 RGB-D 相机 |
| C16 | 雷达点云 |

真实运动需要现场确认急停、空间、线缆和人员安全。dry-run、只读检查和 action server 存在不等于真实抓取安全。

## 项目结构

| 路径 | 内容 |
| --- | --- |
| `src/arachne_description` | 统一机器人模型、RViz 配置、夹爪和传感器坐标系 |
| `src/arachne_hardware` | 真机 bringup、Scout/MS42DC driver、Aubo action/status bridge |
| `src/arachne_operator` | Teach Panel、demo orchestrator、grasp task、road cleanup |
| `src/arachne_sensors` | Gemini335 相机节点 |
| `src/arachne_nav` | Scout Nav2 起步配置 |
| `scripts/` | 主入口和兼容 wrapper，详见 `scripts/README.md` |
| `yolo_workspace` | YOLO 权重、engine、数据集和校准目录 |
| `third_party` | 当前构建需要的第三方驱动和模型 |
| `docs` | 长期文档；阶段记录归档在 `docs/archive/2026-06-refactor/` |

## 文档

- [架构](docs/architecture.zh-CN.md)
- [硬件](docs/hardware.zh-CN.md)
- [Aubo 控制](docs/aubo_control.zh-CN.md)
- [任务服务](docs/tasks.zh-CN.md)
- [开发流程](docs/development.zh-CN.md)
- [标定与导航](docs/calibration_nav.zh-CN.md)
- [排障](docs/troubleshooting.zh-CN.md)
- [脚本入口](scripts/README.md)

## Roadmap

- 稳定真机 bringup、Teach Panel 和 Aubo SDK action。
- 提升 grasp task / road cleanup 的实时性和稳定性。
- 完善 Gemini335 标定、点云 ROI 和可达性判定。
- 沉淀示教数据，逐步接入更高层的策略学习和移动操作任务。

## License

本仓库代码使用 [MIT License](LICENSE)。第三方模型、CAD、SDK 和说明书遵循各自来源许可证。
