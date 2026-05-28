<p align="center">
  <img src="docs/demo/arachne.png" alt="Arachne 机器人系统宣传图" width="900">
</p>

# Arachne 中文说明

[English README](README.md)

Arachne 是一个面向 Scout 2.0 移动底盘、Aubo i5 机械臂和可切换夹爪的 ROS2 workspace。当前默认硬件模型是 Scout 2.0 + Aubo i5 + 易爪机器人二指柔性伺服电机夹爪（MS42DC）；AG95 作为开源夹爪模型保留，用于对比和演示。

两套模型的底盘、机械臂、安装位姿、传感器占位、启动流程和夹爪控制接口都相同，唯一差异是 `gripper_adapter_link` 后面的夹爪模型。MS42DC 和 AG95 在演示界面里都只提供 `Open` / `Close` 两个状态。

## 阅读地图

- [我们提供了什么](#我们提供了什么)：按 package 理解项目结构。
- [当前状态](#当前状态)：现在已经跑通的能力。
- [Roadmap](#roadmap)：近期开发方向。
- [快速启动](#快速启动)：构建并打开默认 RViz 模型。
- [规划与控制骨架](#规划与控制骨架)：MoveIt2、Nav2、ros2_control、sequence executor 和 VLA/WAM action chunk。
- [真机 ROS Bringup](#真机-ros-bringup)：Scout、MS42DC、Aubo 的厂家/官方 ROS 接入。
- [Switch 手柄 Demo](#switch-手柄-demo)、[Gazebo 自主拾取验证](#gazebo-自主拾取验证)、[Godot 展示前端](#godot-展示前端)：交互式 demo。
- [常用检查](#常用检查)、[关键坐标系](#关键坐标系)、[阶段报告](#阶段报告)：日常维护和回顾入口。

相关文档：[建模](docs/modeling.zh-CN.md)、[控制](docs/control.zh-CN.md)、[硬件](docs/hardware.zh-CN.md)、[标定](docs/calibration.zh-CN.md)、[参考资料](docs/references.zh-CN.md)。

## 我们提供了什么

- [src/arachne_description](src/arachne_description)：统一的 Xacro/URDF 机器人模型、RViz 配置、模型变体、安装框架和传感器框架。
- [src/arachne_sim](src/arachne_sim)：面向 RViz 的底盘仿真，负责 `/cmd_vel` 积分、里程计 TF、轮子 joint state 和底盘遥控 GUI。
- [src/arachne_gripper](src/arachne_gripper)：夹爪仿真控制器、joint-state mux，以及只有 `Open` / `Close` 的小型 GUI。
- [src/arachne_demo](src/arachne_demo)：Nintendo Switch Pro 手柄遥控、RViz demo 启动、Gazebo 展示世界和 Gazebo 自主拾取验证。
- [src/arachne_gazebo](src/arachne_gazebo)：Gazebo 专用辅助节点，用于更流畅的 GUI 相机跟随，以及 demo 中的机械臂/夹爪控制桥。
- [src/arachne_hardware](src/arachne_hardware)：真机 ROS bringup 集成包。底层控制交给 Scout 2.0、Aubo i5 和 MS42DC 对应的官方/厂家 ROS 包，Arachne 负责统一状态与命令桥接。
- [src/arachne_control](src/arachne_control)：统一的 ros2_control 控制器命名、mock 控制器 launch，以及 sim/mock/real 硬件 profile。
- [src/arachne_moveit_config](src/arachne_moveit_config)：Aubo i5 + MS42DC/AG95 两种末端的 MoveIt2 起步配置。
- [src/arachne_nav](src/arachne_nav)：基于 `/cmd_vel` 和 `/odom` 契约的 Scout Nav2 起步配置。
- [src/arachne_operator](src/arachne_operator)：轻量 Tk 操作员状态面板、sequence executor 和 VLA/WAM action chunk translator，用于查看 safety、硬件状态、里程计，提供底盘停止、夹爪开闭按钮，以及外部策略接入入口。
- [godot/arachne_showcase](godot/arachne_showcase)：Godot 4.x 高帧率展示前端，包含视觉 teleop、跟随相机、机械臂预设姿态、可拾取物体 demo 逻辑和 ROS2 bridge 占位接口。
- [scripts](scripts)：环境安装、第三方模型下载、夹具切换、可视化启动、URDF 检查和夹爪仿真测试脚本。
- [docs](docs)：硬件、建模、控制、标定说明，以及阶段报告；维护文档均提供同名 `*.zh-CN.md` 中文版。
- [docs/demo/arachne.png](docs/demo/arachne.png)：项目首页宣传图。
- [docs/demo/model_compare.png](docs/demo/model_compare.png)：MS42DC 与 AG95 两套夹爪模型展示图。
- [third_party/MS42DC.step](third_party/MS42DC.step)：MS42DC 原始 CAD。
- [third_party/MS42DC_SPLIT](third_party/MS42DC_SPLIT)：由项目作者手动拆分制作的 MS42DC 可动部件模型，用于真实开合可视化。

外部依赖由 `scripts/fetch_third_party.sh` 按固定版本恢复，保证新环境可以复现。`build/`、`install/` 和 `log/` 是 colcon 在本地构建时生成的标准输出目录。

## 当前状态

- 已完成 Scout 2.0 + Aubo i5 + MS42DC/AG95 的统一 `robot_description`。
- Aubo 安装在当前硬件确认的 Scout 顶部位置。
- MS42DC 使用作者手动拆分的真实 CAD 部件，左右夹指可以绕真实铰点开合。
- MS42DC 默认闭合角为 `0.6 rad`。
- RViz 通过 `scripts/view_model.sh` 启动，会自动清理旧的可视化节点，并打开底盘遥控、机械臂关节滑条、夹爪仿真和 Open/Close 控制窗。
- 机械臂滑条 GUI 默认从当前用户确认的展示姿态启动；点击 `Center` 会回到这个姿态。
- `scripts/switch_demo.sh` 默认启动 Gazebo 展厅 demo，可以用 Nintendo Switch Pro 手柄控制底盘、平滑第三人称视角、Aubo 关节和夹爪。Gazebo 会使用专门的 Scout 轮子物理姿态，确保前进输入时四个轮子同向驱动。
- `scripts/gazebo_autopick_demo.sh` 会启动 Gazebo 自主拾取验证：Scout 根据已知展厅障碍物规划路线，靠近可见地面目标，并实时计算 Aubo/MS42DC 拾取控制。
- `scripts/godot_showcase.sh` 可启动单独的 Godot 4.x 第三人称展示前端，包含可碰撞底盘运动、涂装材质、视觉悬挂、平滑跟随相机、机械臂手动微调、夹爪开闭、可拾取水瓶/小球和 ROS2/UDP bridge 占位接口。
- `scripts/use_gripper.sh` 是推荐的夹具切换入口，可在可视化、demo、MoveIt2、ros2_control、Nav2 和未接真机联合启动中统一选择 MS42DC 或 AG95。
- 真机控制路径已统一为 ROS 接口：Scout 2.0 使用 AgileX `scout_ros2` 通过 CAN 控制，MS42DC 使用本地厂家资料包自带 ROS2 串口节点，Aubo i5 使用 `AuboRobot/aubo_ros2_driver` 通过 TCP/IP 和 ros2_control 控制。
- 没接真机时，也可以用 mock 节点、安全状态机、ros2_control 控制器命名、MoveIt2 起步配置和 Nav2 起步配置继续联调。

## Roadmap

1. 完成物理标定：末端转接板、传感器位姿和用于规划的简化碰撞模型。
2. 在 RViz/Gazebo 中验证新的 MoveIt2 和 ros2_control 起步配置。
3. 用 MoveIt2 和 ros2_control 替换当前 Gazebo 自主拾取验证中的轻量规划器。
4. 先用仿真里程计跑通 Nav2，再在真机阶段替换真实定位和里程计。
5. 将物体抓取从命令级验证升级为 Gazebo 接触验证或 attach-aware 任务。
6. 通过已预留的 bridge 接口，把 Godot 展示前端连接到 ROS2 或 MuJoCo。
7. 真机材料到齐后，在物理 Scout、Aubo 和 MS42DC 上验证官方/厂家 ROS bringup。
8. 在模型、控制器和 launch 接口稳定后，再构建完整 Web 操作界面。

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
  arachne_sim arachne_gripper arachne_hardware arachne_control arachne_moveit_config \
  arachne_nav arachne_operator arachne_description arachne_gazebo arachne_demo \
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
./scripts/use_gripper.sh ag95 view
```

同一个入口也可以切换其他栈里的夹具：

```bash
./scripts/use_gripper.sh ms42dc prehardware launch_rviz:=false
./scripts/use_gripper.sh ag95 moveit launch_rviz:=true
./scripts/use_gripper.sh ms42dc gazebo
```

## 规划与控制骨架

不接真机也可以先检查规划/控制接口：

```bash
./scripts/check_workspace.sh
```

更多细节见 [docs/control.zh-CN.md](docs/control.zh-CN.md)。主要代码入口是 [prehardware_control.launch.py](src/arachne_control/launch/prehardware_control.launch.py)、[sequence_executor.py](src/arachne_operator/arachne_operator/sequence_executor.py) 和 [action_chunk_translator.py](src/arachne_operator/arachne_operator/action_chunk_translator.py)。

一条命令启动未接真机时的联合控制环境：

```bash
ros2 launch arachne_control prehardware_control.launch.py launch_rviz:=false
```

启动硬件 mock：

```bash
ros2 launch arachne_hardware mock_bringup.launch.py
ros2 launch arachne_operator operator_panel.launch.py
ros2 launch arachne_operator sequence_executor.launch.py
```

启动 ros2_control mock 控制器：

```bash
ros2 launch arachne_control mock_ros2_control.launch.py gripper_type:=ms42dc
```

启动 MoveIt2 起步配置：

```bash
ros2 launch arachne_moveit_config moveit_planning.launch.py gripper_type:=ms42dc
```

启动 Nav2 起步配置：

```bash
ros2 launch arachne_nav nav2_sim.launch.py
```

默认会同时启动轻量底盘仿真和 mock `map -> odom` 变换，因此在 lidar/定位硬件尚未接入时 Nav2 也能进入 active。后续如果由真实定位或 SLAM 提供 `map -> odom`，启动时加 `with_mock_map_odom:=false`。

`sequence_executor` 是一个轻量高层命令入口。它通过 `/arachne/sequence/command` 执行带状态、超时、停止处理和 Nav2 结果检查的任务 step，例如 `ready`、`open`、`demo_pick`、`demo_nav_pick` 或 `goto 1.0 0.0 0.0`。

外部 VLA/WAM 策略可以接入 action chunk translator。它从 `/arachne/vla/action_chunk` 接收 JSON，并把每个 step 转成 `/cmd_vel`、Aubo 关节轨迹和 `/arachne/gripper/command`：

```bash
ros2 launch arachne_operator action_chunk_translator.launch.py
ros2 topic pub --once /arachne/vla/action_chunk std_msgs/msg/String \
  "{data: '{\"action\":[0.15,0.0,0,0,0,0,0,0,1],\"duration\":0.3}'}"
```

这些入口目前用于接口验证。下一轮需要在 RViz/Gazebo 中继续调 planning group、控制器行为、Nav2 costmap 和安全门控。

## 真机 ROS Bringup

Arachne 不重复实现底层硬件协议，而是复用已有官方/厂家 ROS 包：

- Scout 2.0：使用 AgileX `scout_ros2` 中的 `scout_base`，底层依赖 `ugv_sdk`，通过 `can0` 接收 `/cmd_vel` 并发布 `/odom` 与 Scout 状态。
- MS42DC：使用本地 MS42DC 厂家资料包中的 `step_motor` ROS2 包。`motor_node` 独占串口，`ms42dc_official_bridge` 将 `/arachne/gripper/command` 映射为厂家 `motor_control` 话题。
- Aubo i5：使用 `AuboRobot/aubo_ros2_driver`，以 `aubo_type:=aubo_i5`、`robot_ip:=...`、`use_fake_hardware:=false` 启动真机控制。

接线和真机 bringup 时主要看 [docs/hardware.zh-CN.md](docs/hardware.zh-CN.md)、[real_bringup.launch.py](src/arachne_hardware/launch/real_bringup.launch.py) 和 [real_hardware.yaml](src/arachne_hardware/config/real_hardware.yaml)。

准备真机相关 ROS 包：

```bash
./scripts/prepare_real_hardware_ros.sh
```

接真机前先检查主机环境：

```bash
./scripts/check_real_hardware_env.sh
```

这个检查脚本同时支持正常 Linux 和 WSL2。Aubo 走 TCP/IP，只要机器人网络可达，两边都可以用。MS42DC 串口和 Scout USB-CAN 需要 Linux 里能看到真实设备节点；在 WSL2 下，需要先用 `usbipd-win` 把 USB 串口或 USB-CAN 设备透传进 WSL2，再确认 `/dev/ttyUSB*` 或 `can0` 存在。

构建核心真机 bringup 包：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select \
  ugv_sdk scout_msgs scout_base serial step_motor arachne_hardware \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

可以按当前已连接硬件选择启动子系统：

```bash
source install/setup.bash
ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=true scout_port:=can0 \
  use_ms42dc:=true ms42dc_port:=/dev/motor_serial \
  use_aubo:=false
```

Aubo 官方驱动和 SDK 依赖安装完成后，再启用机械臂：

```bash
ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=true use_ms42dc:=true use_aubo:=true \
  aubo_robot_ip:=192.168.127.128
```

## Switch 手柄 Demo

先通过蓝牙连接 Nintendo Switch Pro 手柄，然后运行可玩的 Gazebo 展厅 demo：

```bash
./scripts/switch_demo.sh
```

相关文件：[scripts/switch_demo.sh](scripts/switch_demo.sh)、[switch_gazebo_demo.launch.py](src/arachne_demo/launch/switch_gazebo_demo.launch.py)、[switch_teleop.py](src/arachne_demo/arachne_demo/switch_teleop.py)、[arachne_showroom.sdf](src/arachne_demo/worlds/arachne_showroom.sdf)。

在正常 Linux 中，`switch_demo.sh` 会优先使用 `/dev/input/js0`。在 WSL2 中，或者系统里没有 joystick 设备时，它会自动启动浏览器桥接：

```bash
./scripts/switch_demo.sh
# 然后在 Windows 或 Linux 浏览器打开 http://127.0.0.1:8787
```

也可以手动指定输入方式：

```bash
INPUT_BACKEND=joy JOY_DEV=/dev/input/js1 ./scripts/switch_demo.sh
INPUT_BACKEND=web ./scripts/switch_demo.sh
```

对于 Switch Pro 手柄，WSL2 下通常优先推荐浏览器桥接，因为手柄保持连接在 Windows 蓝牙侧，再由浏览器把标准 Gamepad 状态转发到 ROS2。

<p align="center">
  <img src="docs/demo/Bridge.png" alt="Arachne 浏览器手柄桥接页面" width="720">
</p>

如果只想打开轻量 RViz 控制视图：

```bash
DEMO_MODE=rviz ./scripts/switch_demo.sh
```

默认按键：

- 左摇杆：按小车自身坐标连续控制 Scout；摇杆半径决定瞬时速度，纵向分量控制前进/后退，横向分量控制转向。
- 右摇杆：围绕机器人旋转平滑的 Gazebo 跟随相机；在 RViz 模式下旋转 RViz 跟随视角。
- 按住 `ZL` + 十字键上下：移动当前选中的 Aubo 关节。
- `L` / `R`：切换上一个/下一个 Aubo 关节。
- `B`：打开夹爪。`A`：闭合夹爪。
- `+` 或浏览器 `RESET` 按钮：重置底盘、机械臂、夹爪和 Gazebo demo 位姿。`-`：底盘停止。

默认 Gazebo 版本只打开 Gazebo 展厅窗口，不启动 RViz：它加载真实机器人 mesh、轻量化物理展厅、可碰撞物体、diff-drive 物理插件、Gazebo `/gz/odom`、手柄控制的第三人称相机，以及 Aubo 关节微调和 MS42DC 开闭控制桥。完整 ros2_control/Gazebo 控制栈会在后续继续补齐。

## Gazebo 自主拾取验证

运行已知世界信息下的自主拾取验证：

```bash
./scripts/gazebo_autopick_demo.sh
```

相关文件：[scripts/gazebo_autopick_demo.sh](scripts/gazebo_autopick_demo.sh)、[gazebo_autopick_demo.launch.py](src/arachne_demo/launch/gazebo_autopick_demo.launch.py)、[gazebo_autopick_planner.py](src/arachne_demo/arachne_demo/gazebo_autopick_planner.py)。

这个入口不会启动手柄 teleop，避免和自治节点抢 `/cmd_vel`。当前规划器会读取硬编码的 Gazebo 展厅障碍物图，持续刷新 2D A* 路线，用“先转向再前进”的 pure-pursuit 控制 Scout 停到位于约 `(3.4, -2.35)` 的地面 `pick_bottle` 前方约 `0.78 m`，然后在每个控制 tick 根据当前底盘到目标物的相对位姿，用阻尼最小二乘位置 IK 实时计算 Aubo 关节目标。机械臂命令会同时走 `/arachne/gui_joint_states` 和 `ros_gz_bridge` 直连的 Gazebo 单关节位置话题；MS42DC 开闭仍由 Gazebo demo bridge 控制。它是通往 MoveIt2/ros2_control 的仿真验证层，不是真机最终规划器。

<p align="center">
  <img src="docs/demo/gazebo.png" alt="Arachne Gazebo 展厅 demo" width="900">
</p>

相机距离可以不重新构建直接微调：

```bash
GAZEBO_CAMERA_DISTANCE=1.7 ./scripts/switch_demo.sh
```

如果其他手柄上报的左摇杆 Y 轴刚好相反，可以不重新构建直接切换：

```bash
FORWARD_AXIS_SIGN=1.0 ./scripts/switch_demo.sh
```

手动调 MS42DC 闭合角：

```bash
WITH_GRIPPER_SIM=false WITH_GRIPPER_GUI=false ./scripts/view_model.sh
```

拖动 `ms42dc_left_finger_joint`，右指会通过 mimic 反向跟随。默认值已经是 `0.6 rad`，临时覆盖可以这样启动：

```bash
GRIPPER_CLOSED_POSITION=0.58 ./scripts/view_model.sh
```

## Godot 展示前端

Godot 前端用于高帧率第三人称演示和宣传视频，不替代 Gazebo 物理仿真。它通过本地链接复用现有 Scout 2.0、Aubo i5、MS42DC、AG95 和场景物件 mesh，并提供平地办公室地图、键盘/手柄比例控制、可碰撞 Scout 运动、可推动物件、视觉悬挂、平滑跟随相机、MS42DC 开闭动画、Aubo 预设姿态插值和手动关节微调。Aubo 在 Godot 中使用橘色机身和黑色关节涂装，场景会以固定随机种子撒布可拾取水瓶和小球。

相关文件：[godot/arachne_showcase](godot/arachne_showcase)、[scripts/godot_showcase.sh](scripts/godot_showcase.sh)、[scripts/fetch_godot_assets.sh](scripts/fetch_godot_assets.sh)、[scripts/test_godot_showcase.sh](scripts/test_godot_showcase.sh)。

<p align="center">
  <img src="docs/demo/godot.png" alt="Arachne Godot 展示前端" width="900">
</p>

```bash
./scripts/install_godot4.sh   # 如果已经安装 godot4，可以跳过
./scripts/fetch_third_party.sh
./scripts/fetch_godot_assets.sh   # 可选：下载 CC0 办公室道具
./scripts/godot_showcase.sh
```

如果 Godot 不在 `PATH` 中，可以设置 `GODOT_BIN=/path/to/godot4`。启动脚本会先准备本地 mesh 链接和生成的 GLB 缓存文件，再打开 Godot；如果当前 shell 已 source ROS2 环境，会自动使用 UDP bridge 占位模式。机器人视觉 mesh 和安装尺寸来自 URDF/Gazebo 使用的同一批 Scout/Aubo/MS42DC 资源；Godot 为了流畅展示使用简化碰撞代理，并使用独立的办公室展示地图。

在 WSL2 中，启动脚本会自动选择 Mesa D3D12 OpenGL 渲染，让窗口走 Windows GPU，而不是 CPU `llvmpipe`，并启动一个浏览器 Gamepad API 桥接页面，用于识别连接在 Windows 蓝牙侧的 Switch Pro 手柄。打开终端打印的 `http://127.0.0.1:8790` 页面并按一下手柄按钮即可。如果想优先使用独显：

```bash
MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA ./scripts/godot_showcase.sh
```

控制方式：左摇杆或 `WASD` 控制底盘，右摇杆或 `Q/E` 环绕第三人称相机，`A/B` 或 `C/O` 闭合/打开夹爪，`1..5` 选择机械臂预设姿态，`LB/RB` 或 `H/K` 选择关节，D-pad 上/下或 `U/J` 微调当前 Aubo 关节。长按右摇杆按键、按 `P`，或点击浏览器桥接页的 `Auto Pick`，会运行最近物体拾取 demo：寻找目标、自动靠近、用轻量 IK/插值移动 Aubo、闭合夹爪、抬起并回到初始位。D-pad 不参与底盘运动。

如果原生 Godot 手柄路径的相机轴映射异常，可以手动指定相机轴：

```bash
ARACHNE_CAMERA_AXIS=2 ./scripts/godot_showcase.sh
```

无窗口自测：

```bash
./scripts/test_godot_showcase.sh
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

## 关键坐标系

```text
base_link
└── arm_mount_link
    └── aubo_base_link
        └── ... └── tool0
            └── gripper_adapter_link
                └── ms42dc_body_link  # 或 ag95_base_link
                    ├── ms42dc_base_link
                    ├── ms42dc_left_finger_link
                    ├── ms42dc_right_finger_link
                    └── grasp_frame
```

`map -> odom -> base_link` 不属于 URDF 本体，后续由定位和里程计提供。

## 阶段报告

- [stage 0：仓库基础](docs/reports/stage_0_repository_foundation.zh-CN.md)
- [stage 1：统一机器人模型](docs/reports/stage_1_unified_robot_model.zh-CN.md)
- [stage 2：夹爪仿真控制](docs/reports/stage_2_gripper_sim_control.zh-CN.md)
- [stage 3：关节仿真控制](docs/reports/stage_3_joint_sim_control.zh-CN.md)
- [stage 4：Switch 手柄 demo](docs/reports/stage_4_switch_demo.zh-CN.md)
- [stage 5：Godot 展示前端](docs/reports/stage_5_godot_showcase.zh-CN.md)
- [stage 6：Gazebo 自主验证](docs/reports/stage_6_gazebo_autonomy.zh-CN.md)
- [stage 7：真机 ROS bringup](docs/reports/stage_7_real_hardware_ros_bringup.zh-CN.md)
- [stage 8：规划与控制骨架](docs/reports/stage_8_planning_control_scaffold.zh-CN.md)

英文阶段报告与中文版本放在同一目录：[docs/reports](docs/reports)。
