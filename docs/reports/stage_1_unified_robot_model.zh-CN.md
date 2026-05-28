# Stage 1 报告：统一机器人模型

## 结果

`arachne_description` 定义了一个完整 `robot_description`，包含 Scout 2.0、Aubo i5、可切换夹爪、安装转接和可选传感器 frame。MS42DC 与 AG95 两个版本只在夹爪处不同。Scout、Aubo 和 AG95 来自上游开源硬件描述；易爪机器人 MS42DC 二指柔性伺服电机夹爪使用本地 CAD 来源中由项目作者手动拆分的 STL 零件。

## 核心文件

- `src/arachne_description/urdf/arachne.urdf.xacro`：组合完整机器人。
- `urdf/scout/scout_2_vendor.xacro`：从 `agilexrobotics/scout_ros2` 适配 AgileX Scout v2 mesh、尺寸、碰撞和轮子 frame。
- `urdf/aubo/aubo_i5_vendor.xacro`：适配 AuboRobot 的 `aubo_i5.urdf`，给 link/joint 加前缀，移除独立 `world_joint`，并添加 `tool0`。
- `urdf/gripper/ms42dc.urdf.xacro`：加载项目作者手动拆分的 MS42DC mesh，定义左右旋转夹指运动，并保留 Arachne 统一 `grasp_frame`。
- `urdf/gripper/ag95.urdf.xacro`：把 `dh_ag95_description` 包装成可选 AG95 末端。
- `src/arachne_gripper`：为 MS42DC 和 AG95 RViz demo 提供轻量夹爪仿真工具。
- `meshes/gripper/ms42dc/split/*.stl`：从 `third_party/MS42DC_SPLIT` 复制来的 RViz 可用 MS42DC 零件 mesh。
- `urdf/mounts/*.xacro`：底盘、机械臂、转接件和夹爪之间的固定变换。
- `launch/display.launch.py`：发布模型，打开关节滑条并启动 RViz。
- `third_party/aubo_description`：上游 Aubo 模型包。
- `third_party/scout_ros2/scout_description`：通过 `src/vendor/scout_description` 暴露的上游 Scout 模型包。
- `third_party/dh_ag95_gripper_ros2/dh_ag95_description`：通过 `src/vendor/dh_ag95_description` 暴露的可选 AG95 模型包。
- `third_party/MS42DC.step`：当前柔性夹爪模型的本地 CAD 来源。

## 文件关系

顶层 Xacro 包含每个模块，并把它们连接成一条 TF 链：Scout `base_link` 到 `arm_mount_link`，Aubo `aubo_base_link` 到 `tool0`，转接件连接到 MS42DC 或 AG95，所选夹爪连接到 `grasp_frame`。AG95 使用上游模型自带的关节。MS42DC 使用 `ms42dc_base_link`、左右旋转夹指 link，以及固定 `ms42dc_mid_link`。两个夹爪在 demo 中都通过同一个 Open/Close 接口控制。

## 当前限制

Scout 到 Aubo 的安装变换匹配当前硬件布局。末端到夹爪的转接变换仍需要根据真实 MS42DC 安装板直接测量。MS42DC 当前默认仿真闭合状态为 `0.6 rad`。
