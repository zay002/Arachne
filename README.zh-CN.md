<p align="center">
  <img src="docs/demo/model_compare.png" alt="Arachne MS42DC and AG95 model variants" width="900">
</p>

# Arachne 中文说明

Arachne 是一个面向 Scout 2.0 移动底盘、Aubo i5 机械臂和可切换夹爪的 ROS2 workspace。当前默认硬件模型是 Scout 2.0 + Aubo i5 + 易爪机器人二指柔性伺服电机夹爪（MS42DC）；AG95 作为开源夹爪模型保留，用于对比和演示。

两套模型的底盘、机械臂、安装位姿、传感器占位、启动流程和夹爪控制接口都相同，唯一差异是 `gripper_adapter_link` 后面的夹爪模型。MS42DC 和 AG95 在演示界面里都只提供 `Open` / `Close` 两个状态。

## 我们提供了什么

- `src/arachne_description`：统一的 Xacro/URDF 机器人模型、RViz 配置、模型变体、安装框架和传感器框架。
- `src/arachne_sim`：面向 RViz 的底盘仿真，负责 `/cmd_vel` 积分、里程计 TF、轮子 joint state 和底盘遥控 GUI。
- `src/arachne_gripper`：夹爪仿真控制器、joint-state mux，以及只有 `Open` / `Close` 的小型 GUI。
- `src/arachne_demo`：Nintendo Switch 手柄遥控、RViz demo 启动和 Gazebo 展示世界。
- `src/arachne_hardware`：预留真机驱动包，包含空的夹具串口、底盘串口、Aubo TCP/IP 驱动文件。
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
- RViz 通过 `scripts/view_model.sh` 启动，会自动清理旧的可视化节点，并打开底盘遥控、机械臂关节滑条、夹爪仿真和 Open/Close 控制窗。
- 机械臂滑条 GUI 默认从当前用户确认的展示姿态启动；点击 `Center` 会回到这个姿态。
- `scripts/switch_demo.sh` 可以用 Nintendo Switch 手柄控制底盘、Aubo 关节和夹爪。

## Roadmap

1. 完成物理标定：末端转接板、传感器位姿和用于规划的简化碰撞模型。
2. 为 Aubo + MS42DC/AG95 两种末端配置 MoveIt2。
3. 在需要物理碰撞和任务预演时，将当前 RViz 轻量底盘积分器替换为完整仿真后端。
4. 真机材料到齐后，实现 Aubo TCP/IP、Scout 串口、MS42DC 串口硬件接口。
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
  aubo_description scout_description dh_ag95_description \
  arachne_sim arachne_gripper arachne_hardware arachne_description arachne_demo \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

source install/setup.bash
./scripts/view_model.sh
```

底盘遥控 GUI 会发布 `/cmd_vel`，底盘仿真节点会发布 `/odom`、`odom -> base_link` 和轮子 joint state。

如果看到 Aubo 折叠到 Scout 车体里，先重新构建并用 helper 脚本启动，确保安装目录里的 launch 文件是最新的：

```bash
colcon build --base-paths src --packages-select arachne_description
source install/setup.bash
./scripts/view_model.sh
```

也可以直接用命令控制底盘：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25}, angular: {z: 0.0}}"
```

查看 AG95 版本：

```bash
GRIPPER_TYPE=ag95 GRIPPER_SIM_PROFILE=ag95 ./scripts/view_model.sh
```

## Switch 手柄 Demo

先通过蓝牙连接 Nintendo Switch 手柄，然后运行：

```bash
./scripts/switch_demo.sh
```

如果手柄不是 `/dev/input/js0`，可以指定设备，例如 `JOY_DEV=/dev/input/js1 ./scripts/switch_demo.sh`。

默认按键：

- 左摇杆：底盘前进、后退和转向。
- 按住 `ZL` + 右摇杆上下：移动当前选中的 Aubo 关节。
- `L` / `R`：切换上一个/下一个 Aubo 关节。
- `B`：打开夹爪。`A`：闭合夹爪。
- `+`：机械臂回到展示姿态。`-`：底盘停止。

打开带物理物体的 Gazebo 展示世界：

```bash
DEMO_MODE=gazebo ./scripts/switch_demo.sh
```

Gazebo 版本用于宣传和物理预览：它加载真实机器人 mesh、灯光展厅、可碰撞物体和 diff-drive 物理插件。机械臂实时关节运动目前仍以 RViz 为主，完整 Gazebo 机械臂控制会在 ros2_control/Gazebo 栈完成后补上。

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

重置底盘仿真位姿：

```bash
ros2 service call /arachne/base/reset std_srvs/srv/Trigger {}
```

如果 RViz 只看到网格，优先用 `./scripts/view_model.sh` 重新启动；这个脚本会清理旧的 RViz、robot_state_publisher 和 joint_state_publisher 节点。模型加载可能需要等待几秒。
