# Demo Orchestrator

Phase 3B 新增 `demo_orchestrator`，用于把 Visual Grasp、Road Cleanup、camera/viewer/task server 启停和 preflight 从 teach panel 中逐步迁出。

## 边界

- orchestrator 只做编排，不做底层运动控制。
- Aubo 关节运动仍通过 `/arachne/aubo/move_joint` action 或现有 task server 受控路径。
- `grasp_task_server.py` 和 `road_cleanup_task_server.py` 核心任务逻辑不变。
- teach panel 是 GUI 客户端；orchestrator 不可用时，teach panel 回退到原内部编排逻辑。

## 启动

```bash
ros2 launch arachne_operator demo_orchestrator.launch.py autostart:=false
```

`teach_panel.launch.py` 默认 `with_demo_orchestrator:=true`，会随 teach panel 一起启动 orchestrator。用户入口脚本不变。

## Services

- `/arachne/demo/start_camera`
- `/arachne/demo/stop_camera`
- `/arachne/demo/start_visual_grasp`
- `/arachne/demo/start_road_cleanup`
- `/arachne/demo/pause_road_cleanup`
- `/arachne/demo/return_home`
- `/arachne/demo/stop`
- `/arachne/demo/preflight`
- `/arachne/demo/status`

所有服务当前使用 `std_srvs/Trigger`，便于 Phase 3B 保持接口轻量。

## State

`/arachne/demo/state` 使用 `std_msgs/String`，内容为 JSON：

```json
{
  "state": "ready",
  "camera": "running pid=1234",
  "viewer": "stopped",
  "grasp_server": "running pid=1235",
  "cleanup_server": "stopped",
  "last_error": ""
}
```

## 编排流程

`start_visual_grasp`：

1. stop base。
2. 发布 Aubo `teach_off`。
3. start camera。
4. start viewer。
5. start grasp server。
6. 等待 `/arachne/grasp_task/preflight`。
7. 调用 `/arachne/grasp_task/start`。

`start_road_cleanup`：

1. stop base。
2. 发布 Aubo `teach_off`。
3. start camera。
4. start grasp server。
5. start cleanup server。
6. 等待 `/arachne/road_cleanup/preflight`。
7. 调用 `/arachne/road_cleanup/start`。

`stop`：

1. best-effort 调用 `/arachne/road_cleanup/stop`。
2. best-effort 调用 `/arachne/grasp_task/stop`。
3. stop base。
4. 停止 managed 子进程。

## Fallback

teach panel 参数：

- `demo_orchestrator_enabled:=true`
- `demo_orchestrator_fallback_internal:=true`

当 orchestrator service 不可用且 fallback enabled 时，teach panel 使用原有 `_visual_grasp_start_worker()`、`_call_cleanup_task_worker()` 和 managed-process 逻辑，保持旧 demo 行为可用。
