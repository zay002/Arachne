# Arachne 架构

Arachne 分成四层：

1. `arachne_hardware`：Scout、MS42DC、Aubo driver/action/status bridge。
2. `arachne_operator`：Teach Panel、demo orchestrator、grasp task、road cleanup task。
3. `arachne_description` / `arachne_sensors`：URDF/TF、Gemini335、C16。
4. `scripts/`：薄入口，负责加载环境和调用 ROS/CLI。

## Aubo 控制链路

- ROS2 driver 负责状态和 `/joint_states`。
- SDK action server 暴露 `/arachne/aubo/move_joint`。
- 真实执行优先走 action；fallback SDK 路径保留。
- Teach jog 走受 gate/owner 保护的 velocity bridge。

## 任务链路

- `demo_orchestrator` 提供 `/arachne/demo/*` Trigger 服务。
- `grasp_task_server` 负责感知、点云 ROI、抓取规划和执行调度。
- `road_cleanup_task_server` 负责道路巡检、停车、调用抓取任务和恢复行进。

任务层不直接绕过硬件安全入口。
