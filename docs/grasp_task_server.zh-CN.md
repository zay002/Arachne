# Grasp Task Server

`grasp_task_server` 把已经跑通的 `grasp_preview_real_sync.sh` 真机抓取流程封装成一个可重复调用的 ROS 2 任务服务。它不重写目标检测、MoveIt 规划或 Aubo SDK 执行逻辑，而是在外层增加安全检查、任务状态机、进程管理和日志记录。

## 定位

当前 `grasp_preview` 是单次 demo：启动脚本、锁定目标、规划、执行、结束。`grasp_task_server` 的职责是把这条链路变成稳定 primitive，后续 VLA/WAM、网页 UI 或聊天式智能体只需要调用任务接口，而不是直接控制底层硬件。

状态机：

```text
idle -> preflight -> running -> succeeded
                         |-> failed
                         |-> canceled
```

## 启动

先启动真机 bringup，并确认 Aubo、Scout、MS42DC、Gemini335 均处于可用状态：

```bash
./scripts/hardware/real_bringup.sh
```

另开终端启动任务服务器。默认不执行真机运动，只做受保护的预览入口：

```bash
./scripts/vision/grasp_task_server.sh
```

确认空间安全、急停可触达、夹具和机械臂状态正常后，才启用真机执行：

```bash
./scripts/vision/grasp_task_server.sh \
  execute_real:=true \
  confirm_execute_real:=true \
  with_rviz:=false
```

## 调用接口

运行前检查：

```bash
ros2 service call /arachne/grasp_task/preflight std_srvs/srv/Trigger {}
```

启动一次抓取任务：

```bash
ros2 service call /arachne/grasp_task/start std_srvs/srv/Trigger {}
```

查询任务状态：

```bash
ros2 service call /arachne/grasp_task/status std_srvs/srv/Trigger {}
```

取消当前任务：

```bash
ros2 service call /arachne/grasp_task/cancel std_srvs/srv/Trigger {}
```

状态也会持续发布到：

- `/arachne/grasp_task/state`
- `/arachne/grasp_task/event`

消息内容是 JSON 字符串，方便 UI 或外部智能体直接解析。

## 日志

每次任务会生成独立目录：

```text
log/grasp_tasks/YYYYMMDD_HHMMSS_xxxxxxxx/
├── task_request.json
├── preflight.json
├── runner.json
├── process.log
├── events.jsonl
└── summary.json
```

`process.log` 保存 `grasp_preview` 的完整 stdout/stderr；`events.jsonl` 保存任务状态、关键日志路径和执行事件；`summary.json` 保存最终结果。`grasp_preview` 自身的内部日志目录也会记录在 `summary.json` 的 `grasp_preview_log_dir` 字段中。

## 安全检查

preflight 默认检查：

- workspace 和 runner 脚本存在。
- 已构建 `install/setup.bash`。
- 真机执行必须显式 `confirm_execute_real:=true`。
- Aubo 状态话题可用。
- `/joint_states` 中能读到完整 Aubo 六轴关节。
- MS42DC 夹具状态话题可用。
- 如果 safety state machine 已启动，会尝试切到 autonomous，并在任务结束后切回 manual。

可选严格检查：

```bash
./scripts/vision/grasp_task_server.sh \
  execute_real:=true \
  confirm_execute_real:=true \
  require_safety_state_machine:=true \
  require_odom:=true \
  require_camera_topics:=true
```

## 面向智能体平台

后续建议把外部智能体接到任务层，而不是底层控制层：

- `detect_object`：视觉检测与目标锁定。
- `grasp_object`：调用 `/arachne/grasp_task/start`。
- `place_in_basket`：当前抓取 primitive 的投放阶段。
- `drive_relative`：底盘相对运动 primitive。
- `safe_home`：机械臂回安全位姿。

这样聊天式界面、VLA/WAM 或强化学习策略只负责任务决策；Arachne 内部继续负责确定性的检测、规划、执行、安全和日志。
