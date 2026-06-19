# Arachne 仓库结构审计

审计日期：2026-06-19。范围：`scripts/`、`src/*/launch`、ROS2 `console_scripts`/C++ executable、主要 ROS topic/service/action、真机/仿真入口、兼容 wrapper、疑似重复代码。本文只记录现状，不建议删除文件，也不改变现有 demo 行为。

## 结论摘要

- 真机主入口集中在 `scripts/hardware/real_bringup.sh`、`scripts/operator/teach_panel.sh`、`scripts/hardware/real_teach_demo.sh`、`scripts/hardware/real_full_acceptance.sh`，以及对应 ROS launch `arachne_hardware real_bringup.launch.py`。`scripts/hardware/real_grasp_console.sh` 已在 Phase 1 中降级为 deprecated compatibility wrapper。
- 路面清理真实流程入口是 `scripts/vision/grasp_task_server.sh` + `scripts/vision/road_cleanup_task_server.sh`，操作员入口由 `scripts/operator/teach_panel.sh` 管理。流程是相机话题 -> YOLO/segmentation 检测事件 -> 点云/MoveIt/抓取预览管线 -> 底盘停止/恢复 -> 真机执行 -> basket drop-off，没有发现专门绕过真实流程的 sim-only 捷径。
- 仿真入口集中在 `scripts/sim/switch_demo.sh`、`scripts/sim/gazebo_autopick_demo.sh`、`scripts/sim/moveit_grasp_planning_demo.sh`、`scripts/sim/urban_trash_sorting_demo.sh`。
- `scripts/` 里大多是可执行 shell 入口；ROS executable 主要由各包 `setup.py` 的 `console_scripts` 安装，另有 `arachne_gazebo` 两个 C++ 节点。
- 明确的 deprecated wrapper 很少。主要兼容层是 legacy topic/action、`.sh` 包装 `.py` 工具、`view_sensor_model.sh` 别名、官方/直连双驱动 bridge。

## Scripts 标签

标签含义：

- `primary`：推荐用户或操作员直接运行的主入口。
- `helper`：被主入口调用，或用于环境、构建、诊断、准备。
- `deprecated`：兼容旧路径/旧接口的 wrapper，保留以免破坏现有用法。
- `experimental`：验证、展示、远端规划、mock、资产生成等非标准真机主链路。

| 标签 | 脚本 | 说明 |
| --- | --- | --- |
| helper | `scripts/env/arachne_env.sh` | workspace 环境加载 |
| helper | `scripts/env/arachne_real_defaults.sh` | 真机默认参数 |
| helper | `scripts/env/load_local_env.sh` | 本地 `.env` 加载 |
| helper | `scripts/env/ros_env.sh` | ROS distro/workspace 通用环境 |
| helper | `scripts/build/build_workspace.sh` | colcon 全量构建 |
| helper | `scripts/build/build_selected.sh` | 选择性构建 |
| helper | `scripts/build/check_workspace.sh` | workspace 检查 |
| helper | `scripts/build/setup_ubuntu.sh` | Ubuntu 依赖准备 |
| helper | `scripts/build/setup_jetson_humble.sh` | Jetson/Humble 依赖准备 |
| deprecated | `scripts/hardware/real_grasp_console.sh` | 旧真机操作台 compatibility wrapper；新入口使用 `scripts/operator/teach_panel.sh` |
| primary | `scripts/hardware/real_full_teach.sh` | 完整真机示教/回放入口 |
| primary | `scripts/hardware/real_full_acceptance.sh` | 完整真机验收入口 |
| primary | `scripts/hardware/real_bringup.sh` | Scout/MS42DC/Aubo 真机 bringup |
| primary | `scripts/hardware/real_teach_demo.sh` | 一键真机示教 demo，仍被 README 推荐 |
| helper | `scripts/hardware/real_grasp_console_remote.sh` | 远端/SSH 场景下启动旧真机操作台的 compatibility/helper wrapper |
| helper | `scripts/hardware/check_real_hardware_env.sh` | 真机严格环境检查 |
| helper | `scripts/hardware/stop_real_stack.sh` | 停止真机后台进程 |
| helper | `scripts/hardware/fetch_third_party.sh` | 第三方驱动获取/补丁 |
| helper | `scripts/hardware/prepare_real_hardware_ros.sh` | 真机 ROS 依赖准备 |
| helper | `scripts/hardware/prepare_ms42dc_ros2.sh` | MS42DC vendor ROS2 包准备 |
| helper | `scripts/hardware/find_aubo_by_mac.py` | Aubo IP/MAC 探测 |
| helper | `scripts/hardware/real_aubo_prepare.py` | Aubo Running/SafetyMode 准备检查 |
| helper | `scripts/hardware/real_aubo_prepare.sh` | `real_aubo_prepare.py` shell 包装 |
| helper | `scripts/hardware/real_aubo_probe.py` | Aubo RPC/状态探测 |
| helper | `scripts/hardware/real_aubo_probe.sh` | `real_aubo_probe.py` shell 包装 |
| helper | `scripts/hardware/real_aubo_remote_start.py` | 阻塞式 Aubo 远程上电/启动状态机 |
| helper | `scripts/hardware/real_aubo_remote_start.sh` | `real_aubo_remote_start.py` shell 包装 |
| helper | `scripts/hardware/real_aubo_bringup.sh` | Aubo driver 单独 bringup |
| helper | `scripts/hardware/real_aubo_payload.py` | Aubo payload 设置 |
| helper | `scripts/hardware/real_arm_test.sh` | 真机机械臂测试 wrapper |
| helper | `scripts/hardware/real_aubo_z_test.sh` | Aubo Z 方向验收 wrapper |
| helper | `scripts/hardware/real_base_test.sh` | 真机底盘测试 |
| helper | `scripts/hardware/real_gripper_test.sh` | 真机夹爪测试 |
| helper | `scripts/hardware/real_hardware_acceptance_test.sh` | ROS acceptance launch wrapper |
| helper | `scripts/hardware/real_grasp_status.sh` | 抓取链路 topic/service 状态快照 |
| primary | `scripts/hardware/real_lidar_nav.sh` | 真机 lidar/Nav2 定位导航 |
| helper | `scripts/hardware/real_lidar_save_map.sh` | 真机建图保存 |
| primary | `scripts/operator/teach_panel.sh` | 真机示教器主入口 |
| primary | `scripts/vision/gemini335_bringup.sh` | Gemini335 相机 bringup |
| primary | `scripts/vision/gemini_yolo_live.sh` | Gemini + YOLO live 检测 |
| primary | `scripts/vision/grasp_preview.sh` | 相机/YOLO/点云/MoveIt 抓取预览管线 |
| primary | `scripts/vision/grasp_preview_real_sync.sh` | 真机姿态同步，可选 guarded real execution |
| primary | `scripts/vision/grasp_task_server.sh` | 抓取任务 server launch wrapper |
| primary | `scripts/vision/road_cleanup_task_server.sh` | 路面清理任务 server launch wrapper |
| helper | `scripts/vision/setup_yolo_env.sh` | YOLO venv/依赖准备 |
| helper | `scripts/vision/download_yolo_weights.sh` | YOLO 权重下载 |
| helper | `scripts/vision/export_yolo_engine.sh` | TensorRT engine 导出 |
| helper | `scripts/vision/gemini_yolo_detect.py` | YOLO 检测实现 |
| helper | `scripts/vision/grasp_preview_pipeline.py` | 抓取预览/规划/执行核心脚本 |
| helper | `scripts/vision/raw_image_viewer.py` | 原始 image topic 查看器 |
| helper | `scripts/vision/stop_gemini_yolo_live.sh` | 停止 live 检测 |
| helper | `scripts/vision/apriltag_hand_eye_calibration.sh` | 手眼标定入口 |
| helper | `scripts/vision/apriltag_nav_initialize.sh` | AprilTag 导航初始化 |
| helper | `scripts/vision/apriltag_nav_start_mapping.sh` | AprilTag 导航建图启动 |
| experimental | `scripts/vision/gemini_yolo_test.sh` | YOLO 离线/单机测试 |
| experimental | `scripts/vision/mock_road_cleanup_task_test.py` | road cleanup mock smoke test |
| primary | `scripts/sim/switch_demo.sh` | Switch/Gazebo/RViz 可玩 demo |
| primary | `scripts/sim/gazebo_autopick_demo.sh` | Gazebo 自主拾取验证 |
| primary | `scripts/sim/moveit_grasp_planning_demo.sh` | MoveIt 抓取规划仿真 |
| primary | `scripts/sim/urban_trash_sorting_demo.sh` | 道路垃圾语义流程 RViz 仿真 |
| helper | `scripts/sim/test_gripper_sim.sh` | 夹爪仿真回归 |
| primary | `scripts/model/view_model.sh` | 模型/RViz 查看 |
| helper | `scripts/model/check_model.sh` | URDF/xacro 检查 |
| helper | `scripts/model/check_tf.sh` | TF frame 检查 |
| helper | `scripts/model/use_gripper.sh` | 按夹爪类型分发 view/demo/prehardware |
| helper | `scripts/model/convert_ms42dc_step.sh` | STEP 到 STL 转换 |
| deprecated | `scripts/model/view_sensor_model.sh` | 旧的传感器模型查看别名，转到 `view_model.sh` |
| experimental | `scripts/godot/godot_showcase.sh` | Godot showcase |
| experimental | `scripts/godot/godot_gamepad_bridge.py` | Godot/web gamepad bridge |
| helper | `scripts/godot/install_godot4.sh` | Godot 安装 |
| helper | `scripts/godot/fetch_godot_assets.sh` | Godot 资产获取 |
| helper | `scripts/godot/test_godot_showcase.sh` | Godot showcase 测试 |
| primary | `scripts/agent/agent_bridge.sh` | safe Agent Bridge launch |
| experimental | `scripts/remote/remote_moveit_planner_stack.sh` | 远端 MoveIt planner 栈 |
| experimental | `scripts/remote/remote_moveit_planner_server.py` | 远端 MoveIt planner server |
| experimental | `scripts/remote/remote_planner_server.py` | 轻量/HTTP remote planner server |
| experimental | `scripts/remote/remote_planner_client.py` | remote planner client |
| helper | `scripts/remote/deploy_remote_planner.sh` | remote planner 部署 |
| helper | `scripts/remote/sync_remote_planner.sh` | remote planner 同步 |
| experimental | `scripts/calibration/generate_apriltag_floor_board.py` | AprilTag 地面标定板资产生成 |

## Launch 文件

| Launch | 角色 |
| --- | --- |
| `src/arachne_hardware/launch/real_bringup.launch.py` | 真机 Scout、MS42DC、Aubo、可选相机、可选可视化、teach panel 参数总入口 |
| `src/arachne_hardware/launch/mock_bringup.launch.py` | mock 硬件状态、安全状态机、可选 cmd_vel gate |
| `src/arachne_operator/launch/teach_panel.launch.py` | 真机示教/回放 GUI |
| `src/arachne_operator/launch/grasp_task_server.launch.py` | 抓取任务 server |
| `src/arachne_operator/launch/road_cleanup_task_server.launch.py` | 路面清理任务 server |
| `src/arachne_operator/launch/operator_panel.launch.py` | 简化 operator panel |
| `src/arachne_operator/launch/real_hardware_acceptance_test.launch.py` | 真机验收测试 |
| `src/arachne_operator/launch/sequence_executor.launch.py` | 高层 sequence executor |
| `src/arachne_operator/launch/action_chunk_translator.launch.py` | VLA/WAM action chunk translator |
| `src/arachne_operator/launch/teach_visualization.launch.py` | 真机 joint state 可视化 |
| `src/arachne_sensors/launch/gemini335.launch.py` | Gemini335 V4L2、TF、可选 image_view |
| `src/arachne_description/launch/display.launch.py` | URDF/RViz/底盘仿真/夹爪仿真展示 |
| `src/arachne_description/launch/view_model.launch.py` | `display.launch.py` 包装 |
| `src/arachne_moveit_config/launch/moveit_planning.launch.py` | MoveIt planning/move_group |
| `src/arachne_nav/launch/nav2_lidar.launch.py` | lidar/Nav2 真机导航 |
| `src/arachne_nav/launch/nav2_sim.launch.py` | Nav2 仿真 |
| `src/arachne_control/launch/mock_ros2_control.launch.py` | mock ros2_control controllers |
| `src/arachne_control/launch/prehardware_control.launch.py` | mock bringup + Nav2 + MoveIt + sequence/operator 组合 |
| `src/arachne_demo/launch/switch_rviz_demo.launch.py` | RViz Switch demo |
| `src/arachne_demo/launch/switch_gazebo_demo.launch.py` | Gazebo Switch demo |
| `src/arachne_demo/launch/gazebo_autopick_demo.launch.py` | Gazebo autonomous pick demo |
| `src/arachne_sim/launch/moveit_grasp_planning_demo.launch.py` | MoveIt grasp planning demo |
| `src/arachne_sim/launch/urban_trash_sorting_demo.launch.py` | Urban trash sorting RViz demo |
| `src/arachne_agent_bridge/launch/agent_bridge.launch.py` | Agent Bridge |

## ROS Executable

### Python console_scripts

- `arachne_hardware`：`scout_official_status_bridge`、`scout_waveshare_serial_driver`、`ms42dc_official_bridge`、`ms42dc_direct_serial_driver`、`aubo_official_status_probe`、`aubo_teach_command_bridge`、`aubo_sdk_velocity_bridge`、`safety_state_machine`、`safety_cmd_vel_gate`、`hardware_mock`。
- `arachne_operator`：`action_chunk_translator`、`apriltag_hand_eye_calibrator`、`apriltag_nav_initializer`、`grasp_task_server`、`operator_panel`、`real_hardware_acceptance_test`、`road_cleanup_task_server`、`sequence_executor`、`teach_panel`、`teach_visualization_joint_states`。
- `arachne_sensors`：`gemini335_v4l2_node`。
- `arachne_gripper`：`gripper_sim_controller`、`gripper_state_gui`、`joint_state_mux`。
- `arachne_sim`：`base_sim_controller`、`base_teleop_gui`、`end_effector_direction_markers`、`moveit_grasp_planning_demo`、`urban_trash_sorting_demo`。
- `arachne_demo`：`camera_follow_controller`、`gazebo_autopick_planner`、`switch_teleop`、`web_gamepad_bridge`。
- `arachne_agent_bridge`：`agent_bridge`。

### C++ executable

- `arachne_gazebo`：`gazebo_camera_track_bridge`、`gazebo_demo_control_bridge`。

## 主要 Topic / Service / Action

### 真机/安全/基础运动

- `/cmd_vel`：底盘速度命令；真机 Scout、mock hardware、sim base、teach panel、agent bridge、demo teleop 都围绕它工作。
- `/odom`：底盘里程计；真机 Scout、mock/sim base、teach/road cleanup 依赖。
- `/joint_states`：主关节状态；Aubo、joint_state_mux、teach panel、MoveIt 依赖。
- `/joint_trajectory_controller/follow_joint_trajectory`：Aubo 官方 trajectory action，真机示教/验收等待它可用。
- `/joint_trajectory_controller/joint_trajectory`：legacy arm trajectory topic。
- `/aubo_arm_controller/joint_trajectory`：当前 arm trajectory topic。
- `/arachne/aubo/joint_velocity_command`：Aubo SDK 速度 jog 命令。
- `/arachne/aubo/teach_command`：Aubo teach/freedrive 命令。
- `/arachne/gripper/command`、`/arachne/gripper/angle_degrees`、`/arachne/hardware/gripper_status`：MS42DC/夹爪命令与状态。
- `/arachne/hardware/base_status`、`/arachne/hardware/aubo_status`、`/arachne/hardware/gripper_status`：硬件状态。
- `/arachne/safety/state`、`/arachne/safety/enabled`：安全状态。
- `/arachne/safety/enable`、`/disable`、`/estop`、`/recover`、`/set_manual`、`/set_autonomous`：安全服务。
- `/arachne/cmd_vel_raw` -> `/cmd_vel`：可选 `safety_cmd_vel_gate` 门控链路。

### 相机/感知/抓取

- `/camera/color/image_raw`、`/camera/color/camera_info`、`/camera/depth/image_raw`、`/camera/depth/image_color`、`/camera/depth/camera_info`、`/camera/points`：Gemini335 输出。
- `/arachne/perception/taco_instances`：YOLO/segmentation 检测事件，road cleanup 订阅。
- `/arachne/grasp_preview/markers`、`/roi_cloud`、`/path`、`/annotated_image`、`/joint_states`、`/restart_search`：抓取预览/规划可视化与重新搜索。
- `/plan_kinematic_path`：MoveIt planning service。
- `/arachne/grasp_task/start`、`/cancel`、`/stop`、`/restore`、`/status`、`/preflight`、`/base_stop`、`/base_status`：抓取任务与底盘恢复服务。
- `/arachne/grasp_task/state`、`/event`、`/base_state`、`/base_command`：抓取任务状态、事件、底盘命令。
- `/arachne/road_cleanup/start`、`/stop`、`/status`、`/preflight`：路面清理任务服务。
- `/arachne/road_cleanup/state`、`/event`：路面清理任务状态与事件。

### 操作员/自动化/演示

- `/arachne/teach/status`：teach panel 状态。
- `/arachne/sequence/command`、`/arachne/sequence/status`、`/arachne/sequence/stop`：sequence executor。
- `/arachne/vla/action_chunk`、`/arachne/vla/translator/status`、`/arachne/vla/translator/stop`：VLA/WAM action chunk translator。
- `/arachne/agent/command`、`/status`、`/event`、`/tools`、`/arachne/agent/safe_stop`：Agent Bridge。
- `/joy`：手柄输入。
- `/arachne/gui_joint_states`、`/arachne/default_joint_states`、`/arachne/base/joint_states`、`/arachne/gripper/joint_states`：展示/仿真 joint state 输入。
- `/arachne/demo/reset`、`/arachne/base/reset`：demo/base reset。
- `/arachne/camera_yaw`、`/arachne/gazebo_camera/follow_offset`、`/gui/track`：demo 相机跟随。
- `/gz/odom`、`/model/arachne/joint_trajectory`、`/arachne/gazebo/<joint>/command`：Gazebo demo 桥接。
- `/arachne/urban_trash/markers`、`/arachne/urban_trash/roi_cloud`、`/arachne/urban_trash/base_path`、`/map`、`/arachne/urban_trash/return_home`：urban trash sorting demo。

## 真机入口

- `./scripts/operator/teach_panel.sh`：推荐的真机示教器主入口。相关底层 bringup 和任务服务按需单独启动。
- `./scripts/hardware/real_full_teach.sh --yes`：完整示教流程，包含环境检查、Aubo 预启动/远程启动、Scout/MS42DC、teach panel。
- `./scripts/hardware/real_full_acceptance.sh --yes`：完整验收流程，包含环境检查、Aubo 启动、Scout/MS42DC、acceptance test。
- `./scripts/hardware/real_bringup.sh`：底层真机 bringup，可按 `--no-scout`、`--no-ms42dc`、`--no-aubo` 分设备启动。
- `./scripts/operator/teach_panel.sh` 或 `ros2 launch arachne_operator teach_panel.launch.py`：面板直启，要求相关底层服务已就绪。
- `./scripts/vision/gemini335_bringup.sh`、`./scripts/vision/grasp_preview_real_sync.sh`、`./scripts/vision/grasp_task_server.sh`、`./scripts/vision/road_cleanup_task_server.sh`：相机/抓取/清理链路分项入口。
- `./scripts/hardware/real_lidar_nav.sh`：真机 lidar/Nav2。

## 仿真入口

- `./scripts/sim/switch_demo.sh`：Gazebo/RViz 手柄 demo。
- `./scripts/sim/gazebo_autopick_demo.sh`：Gazebo autonomous pick 验证。
- `./scripts/sim/moveit_grasp_planning_demo.sh`：MoveIt grasp planning 仿真。
- `./scripts/sim/urban_trash_sorting_demo.sh`：道路垃圾语义流程仿真。源码注释说明它按语义层镜像真实流程：patrol/scan -> detection -> ROI cloud -> grasp/drop。
- `ros2 launch arachne_hardware mock_bringup.launch.py`：硬件接口 mock。
- `ros2 launch arachne_control prehardware_control.launch.py`：mock bringup + Nav2 + MoveIt + sequence/operator 组合。

## Deprecated / 兼容 Wrapper

- `scripts/model/view_sensor_model.sh`：旧模型查看入口，当前转发到 `scripts/model/view_model.sh`。
- `scripts/hardware/real_grasp_console.sh`：旧真机 console wrapper，当前保留并在运行时提示 `deprecated: use scripts/operator/teach_panel.sh instead`。
- `scripts/hardware/real_grasp_console_remote.sh`：旧 remote planner + console helper，当前保留并提示 deprecated。
- `legacy_arm_trajectory_topic`：`teach_panel`、`action_chunk_translator`、`real_hardware_acceptance_test` 保留 `/joint_trajectory_controller/joint_trajectory` 兼容旧 controller topic。
- `scripts/hardware/real_aubo_prepare.sh`、`real_aubo_probe.sh`、`real_aubo_remote_start.sh`：shell wrapper 包装同名 Python 实现，方便现有命令行用法。
- `ms42dc_official_bridge` 与 `ms42dc_direct_serial_driver`、`scout_official_status_bridge` 与 `scout_waveshare_serial_driver`：不是 deprecated，但属于 vendor/official 与 Arachne direct driver 的并存兼容层。
- 未发现旧的 `scripts/` 根目录顶层入口；README 已说明使用分类路径。

## 疑似重复代码

- `scripts/hardware/real_full_teach.sh` 与 `scripts/hardware/real_full_acceptance.sh`：确认、日志目录、后台进程启动/清理、Aubo payload/remote start 流程相似；目前差异是最终运行 teach panel 还是 acceptance test。
- `scripts/hardware/real_aubo_*.sh` 与同名 `.py`：shell wrapper + Python 实现组合，属于有意保留的命令行兼容。
- `scripts/remote/remote_moveit_planner_server.py` 与 `scripts/remote/remote_planner_server.py`：两个 remote planner server 入口命名接近，应在后续确认职责边界。
- `scripts/sim/moveit_grasp_planning_demo.sh`、`scripts/sim/urban_trash_sorting_demo.sh`、`src/arachne_sim/*demo.py`：MoveIt planning、fallback samples、RViz playback 模式相似；目前一个是单物体抓取规划，一个是巡检/垃圾语义流程。
- `src/arachne_demo/launch/switch_gazebo_demo.launch.py` 与 `src/arachne_demo/launch/gazebo_autopick_demo.launch.py`：Gazebo spawn/bridge/camera/demo_control_bridge 有重复，差异是手动 Switch teleop vs autonomous planner。
- `src/arachne_demo/arachne_demo/camera_follow_controller.py` 与 `src/arachne_gazebo/src/gazebo_camera_track_bridge.cpp`：共同承担 demo 相机跟随链路的 ROS/Gazebo 两端。
- `src/arachne_hardware/arachne_hardware/gripper_serial_driver.py` 与 `ms42dc_direct_serial_driver.py`：同一 MS42DC 命令面向不同底层驱动路径。
- `src/arachne_hardware/arachne_hardware/base_serial_driver.py` 与 `scout_waveshare_serial_driver.py`：同一 Scout 状态/odom 面向 official 与 Waveshare serial 路径。

## 审计边界

- 未运行会启动硬件、相机、Gazebo 或 ROS graph 的命令。
- 未删除文件，未改源码、launch、配置或 demo 行为。
- 只更新文档索引；脚本标签是基于当前 README、launch、setup.py 和源码 topic/service 声明的静态审计判断。
