# 控制

第一层仿真控制已经覆盖 RViz 和 Gazebo demo。轻量视图中，底盘接收 `/cmd_vel`，发布 `/odom`、`odom -> base_link` 和轮子 joint state；Gazebo 模式通过 diff-drive 物理插件驱动模型，并使用 Gazebo 专用 Scout 轮子朝向，使前进命令能产生真实前进。Aubo 在 RViz 模式下由 `joint_state_publisher_gui` 控制，在 Switch demo 中由轻量 Gazebo 轨迹桥控制，默认姿态使用用户确认过的展示姿态，而不是折叠的零位。MS42DC 和 AG95 都只暴露 `Open` / `Close` 两个夹爪状态；模型唯一差异是 `gripper_adapter_link` 下方的夹爪。

在 `display.launch.py` 中，默认零位 joint state 发布到 `/arachne/default_joint_states`，GUI 滑条发布到 `/arachne/gui_joint_states`，底盘轮子状态发布到 `/arachne/base/joint_states`，夹爪状态发布到 `/arachne/gripper/joint_states`，`joint_state_mux` 是统一 `/joint_states` 的唯一发布者，供 `robot_state_publisher` 使用。

## 底盘仿真

启动常规组合仿真：

```bash
./scripts/view_model.sh
```

`Arachne Base` GUI 提供 Forward、Back、Left、Right 和 Stop，并向 `/cmd_vel` 发布 `geometry_msgs/msg/Twist`。终端也使用同一个话题控制：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25}, angular: {z: 0.0}}"
```

重置仿真底盘位姿：

```bash
ros2 service call /arachne/base/reset std_srvs/srv/Trigger {}
```

`base_sim_controller` 是面向 RViz 的轻量运动学积分器。完整物理和碰撞演练应放到专门仿真后端中。

## 夹爪仿真

启动带双按钮 GUI 的 MS42DC 控制：

```bash
ros2 launch arachne_description display.launch.py \
  gripper_type:=ms42dc \
  with_gripper_sim:=true \
  with_gripper_gui:=true \
  gripper_sim_profile:=ms42dc
```

GUI 只暴露 `Open` 和 `Close`。当前 MS42DC 闭合目标为 `0.6 rad`；只有重新调真实夹爪时才建议通过 launch 参数覆盖：

```bash
ros2 launch arachne_description display.launch.py \
  gripper_type:=ms42dc \
  use_gui:=true \
  with_gripper_sim:=true \
  with_gripper_gui:=true \
  gripper_sim_profile:=ms42dc \
  gripper_closed_position:=0.58
```

用户侧仿真服务：

```bash
ros2 service call /arachne/gripper/open std_srvs/srv/Trigger {}
ros2 service call /arachne/gripper/close std_srvs/srv/Trigger {}
```

AG95 使用相同的 Open/Close 接口：

```bash
ros2 launch arachne_description display.launch.py \
  gripper_type:=ag95 \
  with_gripper_sim:=true \
  with_gripper_gui:=true \
  gripper_sim_profile:=ag95
```

MS42DC 使用 `third_party/MS42DC_SPLIT` 中由项目作者手动拆分的真实夹指 mesh。铰链轴为 CAD Z 轴，URDF 轴为 `0 0 -1`，右指以 multiplier `-1.0` mimic 左指，默认闭合角为 `0.6 rad`。Gazebo 会禁用 URDF mimic 标签，因为当前物理引擎不创建 mimic 约束；demo 会显式向左右夹指位置控制器发送镜像命令。

手动滑条检查：

```bash
ros2 launch arachne_description display.launch.py gripper_type:=ms42dc use_gui:=true
```

通过 helper 脚本禁用夹爪 simulator，让 joint-state GUI 直接控制 mimic 关节：

```bash
WITH_GRIPPER_SIM=false WITH_GRIPPER_GUI=false ./scripts/view_model.sh
```

机械臂滑条和夹爪服务一起启动：

```bash
ros2 launch arachne_description display.launch.py \
  gripper_type:=ms42dc \
  use_gui:=true \
  with_gripper_sim:=true \
  with_gripper_gui:=true \
  gripper_sim_profile:=ms42dc
```

## 真机 ROS 控制

真机层现在围绕官方/厂家 ROS 包组织，而不是自写底层协议驱动：

- Scout 2.0 使用 AgileX `scout_ros2` 和 `ugv_sdk`。`scout_base` 订阅 `/cmd_vel`，通过 CAN 发布 `/odom`、`/scout_status` 和 `/rc_status`。
- MS42DC 使用本地夹爪资料中的厂家 `step_motor` ROS2 包。`motor_node` 独占串口并在 `motor_control` 上接收 `step_motor/msg/Motor`；`ms42dc_official_bridge` 将 `/arachne/gripper/command` 中的 `open`、`close`、`home`、`stop` 转成厂家消息。
- Aubo i5 使用 `AuboRobot/aubo_ros2_driver`。官方 launch 通过 TCP/IP 暴露 Aubo 的 ros2_control 轨迹执行。

准备包链接：

```bash
./scripts/prepare_real_hardware_ros.sh
```

运动测试前检查原生 Linux 或 WSL2 的硬件可见性：

```bash
./scripts/check_real_hardware_env.sh
```

检查内容包括 ROS 环境、vendor 包链接、MS42DC 串口候选、Scout SocketCAN 状态和 Aubo TCP 可达性。在 WSL2 下，USB 串口和 USB-CAN 适配器必须先通过 `usbipd-win` 从 Windows 透传进来，Linux 内才会出现 `/dev/ttyUSB*`、`/dev/ttyACM*` 或 `can0`。

构建核心 bringup 包：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select \
  ugv_sdk scout_msgs scout_base serial step_motor arachne_hardware \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

启动部分或完整真机会话：

```bash
source install/setup.bash
ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=true scout_port:=can0 \
  use_ms42dc:=true ms42dc_port:=/dev/motor_serial \
  use_aubo:=false
```

Aubo SDK 依赖和网络准备好后：

```bash
ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=true use_ms42dc:=true use_aubo:=true \
  aubo_robot_ip:=192.168.127.128
```

控制层应继续按设备拆分，但共享 `/cmd_vel`、`/joint_states`、`/odom` 和夹爪命令接口。

## 规划与控制骨架

未接真机前的控制骨架拆成几个标准包：

- `arachne_control`：ros2_control 控制器命名和 `mock_ros2_control.launch.py`。
- `arachne_moveit_config`：MoveIt2 group、机械臂命名姿态、夹爪 open/close 状态、KDL IK、OMPL 规划和控制器映射。
- `arachne_nav`：Scout 的 Nav2 起步参数，使用 `/cmd_vel`、`/odom`、`map -> odom -> base_link` 和 lidar scan 契约。
- `arachne_hardware/mock_bringup.launch.py`：无真实设备时发布仿真硬件状态。
- `arachne_operator`：Tk 状态面板，用于查看 safety、底盘/Aubo/夹爪状态、里程计，并提供停止和夹爪 Open/Close。

运行仓库级检查：

```bash
./scripts/check_workspace.sh
```

启动 mock 硬件和 operator 面板：

```bash
ros2 launch arachne_hardware mock_bringup.launch.py
ros2 launch arachne_operator operator_panel.launch.py
```

启动 ros2_control mock 硬件：

```bash
ros2 launch arachne_control mock_ros2_control.launch.py gripper_type:=ms42dc
```

启动 MoveIt2：

```bash
ros2 launch arachne_moveit_config moveit_planning.launch.py gripper_type:=ms42dc
```

启动 Nav2：

```bash
ros2 launch arachne_nav nav2_sim.launch.py
```

## Nintendo Switch Demo

`src/arachne_demo` 提供 Nintendo Switch Pro 手柄 demo 路径：

- `switch_teleop.py`：将 `sensor_msgs/msg/Joy` 映射到 `/cmd_vel`、`/arachne/gui_joint_states`、`/arachne/gripper/command` 和 `/arachne/demo/reset`。Body 模式使用极坐标 arcade drive：摇杆半径控制瞬时速度，X/Y 方向分解成车体坐标下的线速度和角速度。
- `camera_follow_controller.py`：将右摇杆映射成机器人相对环绕相机角度，并发布相机朝向/偏移话题。
- `src/arachne_gazebo/gazebo_camera_track_bridge.cpp`：把 ROS 相机偏移话题转成 Gazebo `/gui/track` 消息，避免反复调用 `gz service` 子进程。
- `src/arachne_gazebo/gazebo_demo_control_bridge.cpp`：把 MS42DC 开闭命令镜像到 Gazebo 左右夹指控制器，并把 Aubo joint-state 目标转成 Gazebo 关节轨迹命令。
- `web_gamepad_bridge.py`：为 WSL2 或没有 `/dev/input/js*` 的系统提供本地浏览器 Gamepad API 桥。
- `switch_rviz_demo.launch.py`：启动 RViz、底盘仿真、夹爪仿真、所选输入后端、视角控制和 Switch teleop。
- `switch_gazebo_demo.launch.py`：只打开 Gazebo，不开 RViz；生成 Gazebo 安全的 Scout 轮子和 MS42DC 设置，桥接 `/gz/odom`，并复用同一套 Switch teleop。

运行可玩的 Gazebo 展厅 demo：

```bash
./scripts/switch_demo.sh
```

WSL2 中，`switch_demo.sh` 会导出 Gazebo GUI 所需的 Mesa D3D12 设置，让渲染使用 Windows GPU 而不是 CPU `llvmpipe`。默认也会使用 OpenGL 后端和较轻的 `180 Hz` 物理更新率。可用环境变量调参：

```bash
GZ_UPDATE_RATE=120 ./scripts/switch_demo.sh
GZ_RENDER_BACKEND=opengl ./scripts/switch_demo.sh
MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA ./scripts/switch_demo.sh
```

输入后端选择：

```bash
INPUT_BACKEND=auto ./scripts/switch_demo.sh
INPUT_BACKEND=joy JOY_DEV=/dev/input/js1 ./scripts/switch_demo.sh
INPUT_BACKEND=web ./scripts/switch_demo.sh
```

Web 后端下，在浏览器打开 `http://127.0.0.1:8787` 并按一下 Switch Pro 按键。左摇杆按小车自身坐标控制 Scout：摇杆半径控制瞬时速度，纵向控制前进/后退，横向控制转向。右摇杆环绕 Gazebo 跟随相机。`B` / `A` 开闭夹爪；`ZL` + D-pad 上下移动当前选中的 Aubo 关节。`+` 或浏览器 `RESET` 按钮会重置底盘、机械臂、夹爪和 Gazebo demo 位姿。

Switch Pro Web 桥默认使用 `forward_axis_multiplier=-1.0` 和 `lateral_axis_multiplier=1.0`。如果其他手柄轴方向相反，可运行：

```bash
FORWARD_AXIS_SIGN=1.0 ./scripts/switch_demo.sh
LATERAL_AXIS_SIGN=-1.0 ./scripts/switch_demo.sh
```

默认相机距离为 `2.0 m`，可调整：

```bash
GAZEBO_CAMERA_DISTANCE=1.7 ./scripts/switch_demo.sh
```

轻量 RViz-only 视图：

```bash
DEMO_MODE=rviz ./scripts/switch_demo.sh
```

当前 Gazebo 版本重点是单窗口宣传驾驶物理和真实 mesh 可视化。它使用较轻的物理步长、关闭阴影、平地展厅、Gazebo DiffDrive、Gazebo `/gz/odom`、高频 `/gui/track` 相机消息、demo Aubo 轨迹桥，以及显式 MS42DC 夹指位置控制器。完整机械臂和夹爪物理控制后续应迁移到 ros2_control 控制器。

## Gazebo 自主拾取验证

`scripts/gazebo_autopick_demo.sh` 启动一个不含手动 Switch teleop 的 Gazebo-only 自治检查。`gazebo_autopick_demo.launch.py` 会生成同一个 Arachne 机器人，桥接 `/cmd_vel`、`/gz/odom` 和六个 Aubo 直连关节位置命令话题，启动 Gazebo demo 机械臂/夹爪桥，并运行 `gazebo_autopick_planner`。

规划器使用已知 SDF 展厅布局作为确定性地图。它按 Scout footprint 膨胀桌子、标记物、箱子和台座障碍，持续刷新到地面目标靠近位姿的 2D A* 路线，平滑路径，并用“先转向再前进”的 pure-pursuit 控制底盘。到达后，它把车体朝向位于约 `(3.4, -2.35)` 的 `pick_bottle`，在 base frame 中计算 pre-grasp/grasp/lift 笛卡尔目标，用阻尼最小二乘 Jacobian 在线求解 Aubo 位置 IK，并同时发送到 `/arachne/gui_joint_states` 和 Gazebo 直连关节位置话题。MS42DC 开闭仍通过 `/arachne/gripper/command`。

这只是验证层：它证明 launch/control 接口、路线生成、实时底盘/机械臂协同和 Gazebo 命令路径可用。下一步是用 MoveIt2 pose IK/路径规划替换本地位置 IK，并把 demo bridge 替换成 ros2_control 控制器。

## Godot 展示前端

`godot/arachne_showcase` 是单独的 Godot 4.x 前端，用于高帧率第三人称展示和遥控手感。它通过 `assets/vendor/` 下的生成链接加载现有 Scout、Aubo i5、MS42DC、AG95 和道具 mesh，并使用更大的平地办公室式初始地图、可碰撞 character-body 运动、比例 skid-steer 控制、可推动刚体道具、可拾取水瓶/小球、相机阻尼、视觉悬挂、机械臂/夹爪视觉插值，以及手动 Aubo 关节微调。

WSL2 中，`scripts/godot_showcase.sh` 会强制 `GALLIUM_DRIVER=d3d12` 和 OpenGL compatibility renderer，因为 Vulkan 可能退回 CPU `llvmpipe`。脚本也会启动 `scripts/godot_gamepad_bridge.py`，为连接在 Windows 侧的手柄提供浏览器 Gamepad API 桥。原生 Linux 默认使用 Forward+ 和 Godot 原生 joystick 输入，除非显式设置 `GODOT_GAMEPAD_BRIDGE=true`。

Godot 的机器人视觉 mesh 链尽量复用 Gazebo/URDF 使用的同一批 Scout/Aubo/MS42DC 资源和相同安装/铰点常量。为了流畅性，Godot 仍使用简化碰撞代理和自己的办公室地图，因此它是展示层，不是权威接触模型。

底盘驾驶只读取 `WASD` 和左摇杆。D-pad 上下保留给当前机械臂关节，避免离散机械臂命令误触底盘运动。

右摇杆相机读取器会在常见右摇杆轴映射中自动选择最强轴。如果需要手动映射，设置 `ARACHNE_CAMERA_AXIS=<axis>`。

长按右摇杆按键、按 `P`，或点击浏览器桥的 `Auto Pick` 会启动轻量拾取 demo。Godot 会寻找最近可拾取物体，计算靠近目标，使用简单避障斥力驱动 Scout，插值到 Aubo 拾取姿态，闭合 MS42DC，视觉吸附物体，抬起，并回到 `home`。这是作品集/研究占位逻辑；生产路径后续应由 MoveIt2 规划和真实 ROS2 bridge 替换。

Bridge 层目前是占位：

- `/cmd_vel`：存储 Godot 底盘 teleop 输出。
- `/joint_states`：存储 Aubo 插值预设姿态。
- `/odom`：存储 Godot 底盘位姿。
- `/tf`：存储当前 `odom -> base_link` 和静态视觉 frame 占位。

默认 bridge 使用内存模式；当检测到 ROS2 环境时切换到 UDP 占位模式。这样展示前端可以无依赖运行，同时为后续 ROS2、WebSocket、Godot 原生 ROS2、MuJoCo 或其它物理后端保留稳定插入点。

使用 `scripts/fetch_godot_assets.sh` 下载可选 CC0 办公室家具道具，然后用 `scripts/test_godot_showcase.sh` 运行 Godot headless 自测。测试会链接资产、加载场景、执行脚本路线、检查运动/相机/mesh/bridge 状态、验证可拾取目标搜索和 auto-pick drive/IK 生成，并在回归时返回非零。
