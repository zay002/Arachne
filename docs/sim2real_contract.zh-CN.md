# Sim2Real 契约

仿真是对真实机器人工作流的排练，不应绕过真实操作序列。

## 基本原则

- 仿真不能为了展示效果绕过真实流程。
- mock/sim/real-dry-run/real-execute 应尽量复用同一套 topic/service/action。
- 当仿真与真机行为分歧时，优先调整仿真贴近真机，除非明确要求做 visualization-only mock。
- 道路清理任务的仿真语义应保持：operator/teach entrypoint、camera-first observation、YOLO/segmentation-style target acquisition、point-cloud/grasp planning、base stop/recovery、arm execution、basket drop-off。
- Phase 3B 起，demo orchestrator 是 mock/sim/real-dry-run/real-execute 的共同编排层；各 profile 应尽量通过相同 `/arachne/demo/*` service 触发相同顺序。

## Profile 合约

| Profile | 说明 |
| --- | --- |
| `mock` | 静态检查、模型、mock test 或假硬件状态，不要求真实设备。 |
| `sim` | RViz/Gazebo/Godot 运行，验证语义和接口，不驱动真实设备。 |
| `real-dry-run` | 可以连接/观察真机或启动 driver，但默认不执行任务运动。 |
| `real-execute` | 可以让真实 Scout/Aubo/MS42DC 动作，必须有显式确认或操作员监督。 |

## 关键接口

这些接口应在 mock、sim、real-dry-run、real-execute 之间尽量保持一致：

- `/cmd_vel`
- `/odom`
- `/joint_states`
- `/arachne/gripper/command`
- `/arachne/aubo/teach_command`
- `/arachne/aubo/joint_velocity_command`
- `/arachne/aubo/move_joint`
- `/arachne/demo/state`
- `/arachne/demo/start_camera`
- `/arachne/demo/start_visual_grasp`
- `/arachne/demo/start_road_cleanup`
- `/arachne/demo/stop`
- `/arachne/demo/preflight`
- `/arachne/demo/status`
- `/arachne/grasp_task/start`
- `/arachne/grasp_task/stop`
- `/arachne/grasp_task/status`
- `/arachne/grasp_task/base_command`
- `/arachne/road_cleanup/start`
- `/arachne/road_cleanup/stop`
- `/arachne/road_cleanup/status`

## 当前状态

- `urban_trash_sorting_demo` 已按语义层模拟巡检、检测、ROI cloud、抓取和投篮。
- `road_cleanup_task_server` 真机路径不直接做抓取，而是调用 `grasp_task_server`，保持任务层和抓取层分离。
- `mock_bringup.launch.py` 发布与真机 bringup 相同的高层状态 topic，用于无设备联调。
- `demo_orchestrator` 只编排 camera/viewer/grasp_server/cleanup_server 和 task services；Aubo motion 仍通过 `/arachne/aubo/move_joint` action 或现有 task server 受控路径。

## 后续

Phase 3C 建议逐步让 grasp task 和 road cleanup 中的 Aubo joint execution 统一接入 `/arachne/aubo/move_joint` action，同时继续保留 fallback。
