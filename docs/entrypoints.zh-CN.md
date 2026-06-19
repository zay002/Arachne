# Arachne 主入口与安全标注

本文定义 Phase 1 后推荐使用的稳定入口。旧命令不删除，但不再作为主入口宣传。

## 推荐主入口

| 目标 | 主入口 | Profile | Safe by default | 说明 |
| --- | --- | --- | --- | --- |
| 模型检查 | `./scripts/model/view_model.sh` | mock | yes | 检查 URDF、TF、mesh、传感器和吊篮模型。 |
| 道路垃圾语义仿真 | `./scripts/sim/urban_trash_sorting_demo.sh` | sim | yes | RViz 中验证巡检、识别、ROI cloud、抓取、投篮语义流程。 |
| Gazebo 自主拾取 | `./scripts/sim/gazebo_autopick_demo.sh` | sim | yes | Gazebo 自主拾取验证。 |
| MoveIt 抓取规划 demo | `./scripts/sim/moveit_grasp_planning_demo.sh` | sim | yes | MoveIt 规划和 RViz 回放验证。 |
| Switch demo | `./scripts/sim/switch_demo.sh` | sim | yes | Switch/RViz/Gazebo 可玩 demo。 |
| 真机底层启动 | `./scripts/hardware/real_bringup.sh` | real-dry-run | yes | 启动 Scout/MS42DC/Aubo driver 和状态桥，不主动执行任务运动。 |
| 真机示教器 | `./scripts/operator/teach_panel.sh` | real-execute | no | 主示教/回放入口；面板可控制真机。 |
| Demo 编排器 | `ros2 launch arachne_operator demo_orchestrator.launch.py autostart:=false` | mixed | yes | Phase 3B 编排层，提供 `/arachne/demo/*` 服务；不直接执行底层机械臂运动。 |
| 真机一键示教 demo | `./scripts/hardware/real_teach_demo.sh` | real-execute | no | 启动 bringup、等待接口、打开 teach panel、退出后清理。 |
| 真机完整验收 | `./scripts/hardware/real_full_acceptance.sh --yes` | real-execute | no | 完整验收流程，会运动 Scout/Aubo/MS42DC。 |
| 视觉抓取任务 | `./scripts/vision/grasp_task_server.sh` | mixed | yes | 默认不执行真机；真机执行需要 launch 参数确认。 |
| 真机姿态同步抓取预览 | `./scripts/vision/grasp_preview_real_sync.sh` | mixed | yes | 默认只同步/预览；`--execute-real` 才执行。 |
| 道路垃圾清理任务 | `./scripts/vision/road_cleanup_task_server.sh` | mixed | yes | 任务层入口；实际运动取决于底层 grasp/base 服务。 |
| Gemini335 相机 | `./scripts/vision/gemini335_bringup.sh` | real-dry-run | yes | 启动 RGB-D 相机。 |
| Gemini335 + YOLO live | `./scripts/vision/gemini_yolo_live.sh` | real-dry-run | yes | 实时图像检测/标注。 |
| Nav2/lidar | `./scripts/hardware/real_lidar_nav.sh` | real-dry-run | yes | 启动 C16/Nav2 相关栈。 |
| 保存地图 | `./scripts/hardware/real_lidar_save_map.sh` | real-dry-run | yes | 保存 SLAM/map 输出。 |
| AprilTag 导航初始化 | `./scripts/vision/apriltag_nav_initialize.sh` | real-dry-run | yes | 由相机/AprilTag 初始化导航参数。 |
| AprilTag 建图 | `./scripts/vision/apriltag_nav_start_mapping.sh` | real-dry-run | yes | AprilTag 辅助建图启动。 |
| Agent Bridge | `./scripts/agent/agent_bridge.sh` | mixed | yes | 默认 motion disabled。 |
| 停止真机栈 | `./scripts/hardware/stop_real_stack.sh` | mixed | yes | 停止已知 Arachne 真机进程。 |

## Deprecated Wrapper

| Wrapper | 替代入口 | 说明 |
| --- | --- | --- |
| `./scripts/hardware/real_grasp_console.sh` | `./scripts/operator/teach_panel.sh` | 旧真机 console 兼容入口，运行时提示 deprecated；保留以免破坏旧命令。 |
| `./scripts/hardware/real_grasp_console_remote.sh` | `./scripts/operator/teach_panel.sh` | 远端 planner + 旧 console helper，保留兼容。 |
| `./scripts/model/view_sensor_model.sh` | `./scripts/model/view_model.sh` | 旧传感器模型查看 wrapper。 |

## 真机执行安全变量

| 变量/参数 | 适用入口 | 含义 |
| --- | --- | --- |
| `--yes` | `real_full_teach.sh`、`real_full_acceptance.sh`、旧 `real_grasp_console.sh` | 明确允许对应真机流程启动。 |
| `ARACHNE_CONFIRM_REAL_MOTION=YES` | acceptance、Aubo Z 测试等 | 允许运动测试。 |
| `ARACHNE_CONFIRM_REAL_TEACH=YES` | `real_full_teach.sh` | 允许完整示教栈启动。 |
| `ARACHNE_CONFIRM_AUBO_DRIVER=YES` | `real_aubo_bringup.sh` | 允许启动 Aubo driver。 |
| `ARACHNE_CONFIRM_AUBO_REMOTE_START=YES` | `real_aubo_remote_start.sh` | 允许 ROS 侧远程上电/启动 Aubo。 |
| `ARACHNE_CONFIRM_GRASP_EXECUTE_REAL=YES` + `--execute-real` | `grasp_preview_real_sync.sh` | 允许预览管线执行真实抓取。 |
| `execute_real:=true confirm_execute_real:=true` | `grasp_task_server.sh` | 允许抓取任务 server 调用真实执行路径。 |

## Profile 说明

- `mock`：只做静态检查、文档、模型、生成资产或 mock test，不连接真机。
- `sim`：启动 RViz/Gazebo/Godot 等仿真或展示，不驱动真实硬件。
- `real-dry-run`：连接或观察真机、启动 driver、发布状态，默认不执行任务运动。
- `real-execute`：可以导致 Scout/Aubo/MS42DC 运动或改变真实控制状态。
- `mixed`：默认安全，但可通过参数升级为 real-execute，或依赖外部已启动服务。

## 日常开发命令

```bash
source scripts/env/arachne_env.sh
./scripts/build/build_workspace.sh
./scripts/model/view_model.sh
./scripts/sim/urban_trash_sorting_demo.sh
```

真机联调建议顺序：

```bash
./scripts/hardware/check_real_hardware_env.sh --strict
./scripts/hardware/real_bringup.sh
./scripts/operator/teach_panel.sh
./scripts/vision/grasp_task_server.sh
./scripts/vision/road_cleanup_task_server.sh
```

`teach_panel.launch.py` 默认 `with_demo_orchestrator:=true`，面板优先调用
`/arachne/demo/start_visual_grasp`、`/arachne/demo/start_road_cleanup` 和
`/arachne/demo/stop`；orchestrator 不可用时回退到面板内置编排逻辑。

旧入口 `./scripts/hardware/real_grasp_console.sh` 只作为兼容保留。
