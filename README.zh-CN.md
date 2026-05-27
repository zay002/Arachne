<p align="center">
  <img src="docs/demo/model_compare.png" alt="Arachne MS42DC and AG95 model variants" width="900">
</p>

# Arachne 中文说明

Arachne 是一个面向 Scout 2.0 移动底盘、Aubo i5 机械臂和可切换夹爪的 ROS2 workspace。当前默认硬件模型是 Scout 2.0 + Aubo i5 + 易爪机器人二指柔性伺服电机夹爪（MS42DC）；AG95 作为开源夹爪模型保留，用于对比和演示。

两套模型的底盘、机械臂、安装位姿、传感器占位、启动流程和夹爪控制接口都相同，唯一差异是 `gripper_adapter_link` 后面的夹爪模型。MS42DC 和 AG95 在演示界面里都只提供 `Open` / `Close` 两个状态。

## 我们提供了什么

- `src/arachne_description`：统一的 Xacro/URDF 机器人模型、RViz 配置、模型变体、安装框架和传感器框架。
- `src/arachne_gripper`：夹爪仿真控制器、joint-state mux，以及只有 `Open` / `Close` 的小型 GUI。
- `scripts`：环境安装、第三方模型下载、可视化启动、URDF 检查和夹爪仿真测试脚本。
- `docs`：硬件、建模、控制、标定说明，以及阶段报告。
- `docs/demo/model_compare.png`：MS42DC 与 AG95 两套夹爪模型展示图。
- `third_party/MS42DC.step`：MS42DC 原始 CAD。
- `third_party/MS42DC_SPLIT/*.stl`：由项目作者手动拆分制作的 MS42DC 可动部件模型，用于真实开合可视化。

上游大仓库不会上传到 git，包括 Aubo、Scout、AG95 的完整第三方仓库；它们由 `scripts/fetch_third_party.sh` 按固定 commit 下载。`build/`、`install/`、`log/` 和本地开发计划 `plan.md` 也不会上传。

## 当前状态

- 已完成 Scout 2.0 + Aubo i5 + MS42DC/AG95 的统一 `robot_description`。
- Aubo 安装在当前硬件确认的 Scout 顶部位置。
- MS42DC 使用作者手动拆分的真实 CAD 部件，左右夹指可以绕真实铰点开合。
- MS42DC 默认闭合角为 `0.6 rad`。
- RViz 通过 `scripts/view_model.sh` 启动，会自动清理旧的可视化节点，并打开机械臂关节滑条、夹爪仿真和 Open/Close 控制窗。

## Roadmap

1. 完成物理标定：末端转接板、传感器位姿和用于规划的简化碰撞模型。
2. 为 Aubo + MS42DC/AG95 两种末端配置 MoveIt2。
3. 添加 `ros2_control` 控制器，以及 Aubo、Scout、MS42DC 的硬件接口。
4. 添加仿真后端，用于运动规划和任务预演。
5. 在模型、控制器和 launch 接口稳定后，再构建 Web 操作界面。

## 快速启动

推荐环境：

- Ubuntu 24.04 + ROS2 Jazzy
- Ubuntu 22.04 + ROS2 Humble

```bash
cd Arachne
./scripts/setup_ubuntu.sh
./scripts/fetch_third_party.sh

conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash  # Ubuntu 22.04 使用 /opt/ros/humble/setup.bash

colcon build --base-paths src --packages-select \
  aubo_description scout_description dh_ag95_description arachne_gripper arachne_description \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

source install/setup.bash
./scripts/view_model.sh
```

查看 AG95 版本：

```bash
GRIPPER_TYPE=ag95 GRIPPER_SIM_PROFILE=ag95 ./scripts/view_model.sh
```

手动调 MS42DC 闭合角：

```bash
WITH_GRIPPER_SIM=false WITH_GRIPPER_GUI=false ./scripts/view_model.sh
```

拖动 `ms42dc_left_finger_joint`，右指会通过 mimic 反向跟随。默认值已经是 `0.6 rad`，临时覆盖可以这样启动：

```bash
GRIPPER_CLOSED_POSITION=0.58 ./scripts/view_model.sh
```

## 常用检查

```bash
./scripts/check_model.sh
./scripts/test_gripper_sim.sh
```

如果 RViz 只看到网格，优先用 `./scripts/view_model.sh` 重新启动；这个脚本会清理旧的 RViz、robot_state_publisher 和 joint_state_publisher 节点。模型加载可能需要等待几秒。
