# Grasp Task Server

`grasp_task_server` wraps the already working `grasp_preview_real_sync.sh` real-grasp flow as a repeatable ROS 2 task service. It does not reimplement detection, MoveIt planning, or Aubo SDK execution. Instead, it adds preflight checks, a task state machine, process control, and structured logs around the existing pipeline.

## Role

The current `grasp_preview` is a single-run demo: start the script, lock a target, plan, execute, and exit. `grasp_task_server` turns that path into a stable primitive so a future VLA/WAM, web UI, or chat-driven agent can call a task interface instead of touching low-level hardware commands.

State machine:

```text
idle -> preflight -> running -> succeeded
                         |-> failed
                         |-> canceled
```

## Launch

Start real bringup first and verify that Aubo, Scout, MS42DC, and Gemini335 are available:

```bash
./scripts/hardware/real_bringup.sh
```

Start the task server in another terminal. By default it does not arm real motion:

```bash
./scripts/vision/grasp_task_server.sh
```

Only after the workspace is clear, emergency stop is reachable, and the robot state is normal, enable real execution:

```bash
./scripts/vision/grasp_task_server.sh \
  execute_real:=true \
  confirm_execute_real:=true \
  with_rviz:=false
```

## API

Run preflight:

```bash
ros2 service call /arachne/grasp_task/preflight std_srvs/srv/Trigger {}
```

Start one grasp task:

```bash
ros2 service call /arachne/grasp_task/start std_srvs/srv/Trigger {}
```

Query state:

```bash
ros2 service call /arachne/grasp_task/status std_srvs/srv/Trigger {}
```

Cancel the active task:

```bash
ros2 service call /arachne/grasp_task/cancel std_srvs/srv/Trigger {}
```

The server also publishes JSON strings on:

- `/arachne/grasp_task/state`
- `/arachne/grasp_task/event`

## Autonomous Base Motion Primitive

The task server now reserves a Scout base motion interface for future mobile
manipulation. The intended production path is internal automatic triggering:
detection/planning logic inside the task server decides when the base should
move. The implementation follows the teach panel pattern: publish `/cmd_vel`
and use `/odom` to stop relative translation and relative yaw motions.

Base state:

```bash
ros2 service call /arachne/grasp_task/base_status std_srvs/srv/Trigger {}
```

Stop base motion:

```bash
ros2 service call /arachne/grasp_task/base_stop std_srvs/srv/Trigger {}
```

For debugging or external bridges, JSON commands can be sent to
`/arachne/grasp_task/base_command`. This topic is not the final human-control
path; later task logic can call the same internal primitive directly.

Drive relative:

```bash
ros2 topic pub --once /arachne/grasp_task/base_command std_msgs/msg/String \
  "{data: '{\"command\":\"drive_relative\",\"distance_m\":0.3}'}"
```

Turn relative:

```bash
ros2 topic pub --once /arachne/grasp_task/base_command std_msgs/msg/String \
  "{data: '{\"command\":\"turn_relative\",\"angle_deg\":30.0}'}"
```

Replay base segments recorded by the teach panel:

```json
{
  "command": "replay_segments",
  "segments": [
    {"type": "linear", "signed_distance_m": 0.5},
    {"type": "angular", "signed_angle_rad": 0.52}
  ]
}
```

Base state is also published as JSON on `/arachne/grasp_task/base_state` and
embedded in the `base` field of `/arachne/grasp_task/status`.

## Logs

Each task creates one run directory:

```text
log/grasp_tasks/YYYYMMDD_HHMMSS_xxxxxxxx/
├── task_request.json
├── preflight.json
├── runner.json
├── process.log
├── events.jsonl
└── summary.json
```

`process.log` captures the full `grasp_preview` stdout/stderr stream; `events.jsonl` stores state transitions and key events; `summary.json` stores the final result. The nested `grasp_preview` log directory is copied into the `grasp_preview_log_dir` field when the runner prints it.

## Safety

Default preflight checks:

- Workspace and runner script exist.
- `install/setup.bash` exists.
- Real execution requires `confirm_execute_real:=true`.
- Aubo status topic is recent.
- `/joint_states` contains all six Aubo joints.
- MS42DC gripper status topic is recent.
- If the safety state machine is available, the server can switch to autonomous before execution and back to manual after completion.

Strict mode:

```bash
./scripts/vision/grasp_task_server.sh \
  execute_real:=true \
  confirm_execute_real:=true \
  require_safety_state_machine:=true \
  require_odom:=true \
  require_camera_topics:=true
```

## Agent Platform Direction

Future agents should call task-level primitives instead of hardware-level commands:

- `detect_object`
- `grasp_object`
- `place_in_basket`
- `drive_relative`
- `safe_home`

The chat UI, VLA/WAM, or policy layer should decide what to do; Arachne should keep deterministic detection, planning, execution, safety, and logs inside the robot stack.
