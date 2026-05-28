# Stage 4 报告：Switch 手柄 Demo

## 结果

Arachne 现在有一条可在原生 Linux 和 WSL2 上使用 Nintendo Switch Pro Controller 的交互式 demo 路径。默认 Gazebo 展厅作为单个可玩窗口打开，提供基于车体坐标的 Scout 平地驾驶、平滑跟随机器人的第三人称相机、Aubo 关节微调、MS42DC 开闭控制，以及 diff-drive 物理预览。RViz 模式仍可用于轻量模型检查。

Switch Pro 轴默认使用 `FORWARD_AXIS_SIGN=-1.0` 和 `LATERAL_AXIS_SIGN=1.0`，匹配 Gazebo 中观察到的 Scout 车头和转向方向。左摇杆输入使用极坐标 arcade drive：X/Y 圆半径控制瞬时速度，方向分解为车体坐标下的线速度和角速度。

## 核心文件

- `src/arachne_demo/`：demo launch 和控制器输入 ROS2 Python 包。
- `arachne_demo/switch_teleop.py`：把 Switch Pro `Joy` 消息映射到 `/cmd_vel`、Aubo joint state 和夹爪 open/close 命令，底盘运动基于车体坐标。
- `arachne_demo/camera_follow_controller.py`：把右摇杆映射为机器人相对相机环绕角，并发布相机 heading/offset 话题。
- `src/arachne_gazebo/`：Gazebo helper 包；`gazebo_camera_track_bridge` 发布高频 `/gui/track` 消息实现平滑 GUI 相机跟随，`gazebo_demo_control_bridge` 将 Aubo/MS42DC demo 命令送入 Gazebo。
- `arachne_demo/web_gamepad_bridge.py`：为 WSL2 提供本地浏览器 Gamepad API 桥。
- `launch/switch_rviz_demo.launch.py`：启动 RViz、底盘仿真、夹爪仿真、所选输入后端、视角控制和 Switch teleop。
- `launch/switch_gazebo_demo.launch.py`：启动无 RViz 的 Gazebo，生成带 Gazebo 安全 Scout 轮子与 MS42DC 设置的 Arachne，桥接 `/gz/odom`，并复用同一套 Switch teleop。
- `worlds/arachne_showroom.sdf`：带物理的 demo 世界，包含平地、灯光、绕桩标记、工作台和可移动道具。
- `urdf/gazebo/arachne_gazebo_plugins.xacro`：demo 模型的 Gazebo DiffDrive、joint-state、Aubo trajectory 和 MS42DC 夹指位置插件。
- `scripts/switch_demo.sh`：一键启动脚本，默认 Gazebo 展厅模式，并通过 `DEMO_MODE=rviz` 保留 RViz 模式。

## 接口

- 控制器输入：`/joy`
- 底盘命令：`/cmd_vel`
- 相机 heading：`/arachne/camera_yaw`
- Gazebo 相机偏移：`/arachne/gazebo_camera/follow_offset` -> `/gui/track`
- Gazebo 里程计：`/gz/odom`
- 机械臂 demo 状态：`/arachne/gui_joint_states` -> `/model/arachne/joint_trajectory`
- 夹爪命令：`/arachne/gripper/command`
- MS42DC Gazebo 夹指：`/arachne/gazebo/ms42dc_left_finger/command`、`/arachne/gazebo/ms42dc_right_finger/command`
- RViz 跟随 frame：`arachne_view_frame`

Gazebo 会禁用 MS42DC URDF mimic 关节，并显式驱动两个夹指，因为当前物理引擎不会创建 mimic 约束。Scout 轮子 link 在 RViz 中保留正常上游朝向，而 Gazebo 使用物理专用轮子朝向，使前进摇杆输入让四个轮子同向驱动。下一步改进是用完整 ros2_control 控制器替换这些 demo bridge。
