# Stage 3 报告：组合 RViz 仿真控制

## 结果

Arachne 现在有一个轻量组合 RViz 控制 demo。底盘通过 `/cmd_vel` 移动，机械臂通过带有用户确认展示姿态的关节滑条控制，MS42DC 和 AG95 使用相同的 Open/Close 夹爪 GUI。本阶段是运动学可视化，不是完整物理仿真。

## 核心文件

- `src/arachne_sim/`：面向 RViz 的仿真控制器 ROS2 Python 包。
- `arachne_sim/base_sim_controller.py`：积分 `/cmd_vel`，发布 `/odom`，广播 `odom -> base_link`，并发布 Scout 轮子 joint state。
- `arachne_sim/base_teleop_gui.py`：发布 `/cmd_vel` 的 Forward/Back/Left/Right/Stop 小 GUI。
- `arachne_gripper/joint_state_mux.py`：把 default、GUI、base 和 gripper joint stream 合并为 `/joint_states`。
- `launch/display.launch.py`：启动底盘仿真、底盘 GUI、机械臂滑条、夹爪仿真、夹爪 GUI、robot state publisher 和 RViz。
- `src/arachne_hardware/`：真机集成包，后续填入官方/厂家 ROS bringup wrapper。

## 接口

- 底盘命令：`/cmd_vel`
- 底盘状态：`/odom`、`odom -> base_link`、`/arachne/base/joint_states`
- 底盘重置：`/arachne/base/reset`
- 机械臂 demo 控制：`joint_state_publisher_gui`
- 夹爪 demo 控制：`/arachne/gripper/open`、`/arachne/gripper/close`

下一步是在保持共享仿真 launch 契约稳定的同时，为机械臂规划加入 MoveIt2。
