# 建模

## 原则

Arachne 暴露一个统一的 `robot_description`：Scout 底盘、Aubo 机械臂、可切换夹爪、安装件和可选传感器都在同一棵 URDF 树里。MS42DC 和 AG95 两个版本唯一的区别，是 `gripper_adapter_link` 下方连接的夹爪不同。

## 当前文件

- `src/arachne_description/urdf/arachne.urdf.xacro`：顶层模型组合入口。
- `urdf/scout/scout_2_vendor.xacro`：基于 `agilexrobotics/scout_ros2` 改写的 Scout v2 模型。
- `urdf/aubo/aubo_i5_vendor.xacro`：基于 `AuboRobot/aubo_description` 改写的 Aubo i5 模型。
- `urdf/gripper/ms42dc.urdf.xacro`：易爪机器人 MS42DC 二指柔性伺服电机夹爪的可动拆分模型，包含左右旋转夹指、mimic 运动和统一的 `grasp_frame`。
- `urdf/gripper/ag95.urdf.xacro`：可选 DH Robotics AG95 包装模型，并接入统一 `grasp_frame`。
- `meshes/gripper/ms42dc/split/*.stl`：从 `third_party/MS42DC_SPLIT` 复制来的 RViz 可用 MS42DC 拆分 mesh。
- `urdf/mounts/*`：底盘到机械臂、末端到夹爪的固定转接。
- `urdf/sensors/*`：可选 lidar 和末端相机占位。

## 坐标链

默认 MS42DC 版本：

```text
base_link -> arm_mount_link -> aubo_base_link -> ... -> tool0 -> gripper_adapter_link -> ms42dc_body_link -> grasp_frame
```

AG95 版本：

```text
base_link -> arm_mount_link -> aubo_base_link -> ... -> tool0 -> gripper_adapter_link -> ag95_base_link -> grasp_frame
```

`map -> odom -> base_link` 不放在 URDF 内；它属于定位和里程计系统。

## 模型策略

Scout、Aubo 和 AG95 使用来自上游或厂家模型的 mesh 与运动学参数。MS42DC 使用项目作者基于本地 CAD 手动拆分的可动零件，左夹指为主动旋转关节，右夹指通过 mimic 反向跟随。铰链方向已经在 RViz 中检查，当前默认闭合角是 `0.6 rad`。
