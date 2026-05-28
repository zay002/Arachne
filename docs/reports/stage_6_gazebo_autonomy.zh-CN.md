# Stage 6：Gazebo 自主拾取验证

## 总结

Arachne 现在有一条 Gazebo 侧自治验证路径。它保留手动 Switch demo，同时新增一个独立 launch，使用已知展厅几何规划 Scout 路线、避障、对准远处可见地面拾取目标，并在线计算 Aubo/MS42DC 拾取命令。

## 核心文件

- `src/arachne_demo/arachne_demo/gazebo_autopick_planner.py`：已知世界规划器。它持续刷新 2D A*、平滑路径、用“先转向再前进”的 pure-pursuit 跟踪底盘、对准目标、生成实时笛卡尔拾取目标、用阻尼最小二乘 Aubo 位置 IK 求解、发布直连关节命令，并控制 MS42DC 开闭。
- `src/arachne_demo/launch/gazebo_autopick_demo.launch.py`：Gazebo launch 入口。它生成 Arachne，启动 `/cmd_vel`、`/gz/odom` 和 Aubo 直连关节命令桥，启动相机跟踪、Gazebo 机械臂/夹爪桥和自治规划器。
- `src/arachne_demo/worlds/arachne_showroom.sdf`：增加高可见度地面目标垫，以及离机器人更远、位于开放区域的 `pick_bottle` 和 `pick_ball`。
- `src/arachne_description/urdf/gazebo/arachne_gazebo_plugins.xacro`：为六个 Aubo 关节加入 Gazebo 直连位置控制器，并降低横向轮胎摩擦，使四轮 skid-steer 底盘能更真实地转向。
- `scripts/gazebo_autopick_demo.sh`：一键运行脚本，复用手动 Gazebo demo 的 ROS/Gazebo 资源和 WSL2 GPU 设置。
- `src/arachne_gazebo/src/gazebo_demo_control_bridge.cpp`：复用桥接器，将 ROS 夹爪命令和 Aubo joint state 转换为 Gazebo 夹指/轨迹命令。

## 说明

本阶段不是最终规划器。它验证已知世界仿真中的自治流程：实时底盘路径规划、底盘/机械臂时序、本地位置 IK 和 Gazebo 控制接口。下一步是用 MoveIt2 pose IK/路径规划替换本地 IK，再把机械臂/夹爪执行迁移到 ros2_control。
