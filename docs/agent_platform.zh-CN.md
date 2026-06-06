# Agent Bridge

`arachne_agent_bridge` 是外部智能体进入 Arachne 的安全入口。它不包含 LLM SDK，也不读取 API key；它只把机器人侧能力整理成白名单工具，让 Hermes、网页 UI、MCP 服务或自写 Agent 通过 ROS JSON 命令调用。

## 设计边界

推荐结构：

```text
External Agent / UI / Hermes
        |
        | JSON tool call
        v
arachne_agent_bridge
        |
        | bounded ROS commands
        v
Arachne task servers / teach-style controls
```

Agent 不应直接控制串口、Aubo SDK、`/cmd_vel` 或关节控制器。所有动作都应先经过 bridge 的限幅、白名单和停止接口。

## 启动

默认启动是只读/拒绝运动模式：

```bash
./scripts/agent/agent_bridge.sh
```

真机或仿真需要允许运动时，必须显式打开双重门控：

```bash
./scripts/agent/agent_bridge.sh \
  motion_enabled:=true \
  confirm_agent_motion:=true
```

Aubo freedrive/teach mode 是额外的模式切换权限，默认关闭：

```bash
./scripts/agent/agent_bridge.sh \
  motion_enabled:=true \
  confirm_agent_motion:=true \
  allow_mode_change:=true
```

## API

命令入口：

```text
/arachne/agent/command    std_msgs/msg/String, JSON
```

状态和事件：

```text
/arachne/agent/status
/arachne/agent/event
/arachne/agent/tools
```

只读服务：

```bash
ros2 service call /arachne/agent/status std_srvs/srv/Trigger {}
ros2 service call /arachne/agent/tools std_srvs/srv/Trigger {}
```

安全停止：

```bash
ros2 service call /arachne/agent/safe_stop std_srvs/srv/Trigger {}
```

## 工具白名单

只读状态：

```bash
ros2 topic pub --once /arachne/agent/command std_msgs/msg/String \
  "{data: '{\"tool\":\"get_robot_state\"}'}"
```

底盘短时速度控制：

```bash
ros2 topic pub --once /arachne/agent/command std_msgs/msg/String \
  "{data: '{\"tool\":\"base_velocity\",\"linear_x\":0.05,\"angular_z\":0.0,\"duration_sec\":0.5}'}"
```

底盘相对移动，转发给 `grasp_task_server` 的底盘原语：

```bash
ros2 topic pub --once /arachne/agent/command std_msgs/msg/String \
  "{data: '{\"tool\":\"base_relative\",\"distance_m\":0.1}'}"
```

机械臂末端沿 x/y/z 小步移动：

```bash
ros2 topic pub --once /arachne/agent/command std_msgs/msg/String \
  "{data: '{\"tool\":\"arm_cartesian_jog\",\"axis\":\"z\",\"distance_m\":0.01}'}"
```

也可以给向量：

```bash
ros2 topic pub --once /arachne/agent/command std_msgs/msg/String \
  "{data: '{\"tool\":\"arm_cartesian_jog\",\"vector\":[0.0,0.0,0.01]}'}"
```

机械臂单关节小步转动：

```bash
ros2 topic pub --once /arachne/agent/command std_msgs/msg/String \
  "{data: '{\"tool\":\"arm_joint_jog\",\"joint\":\"wrist3_joint\",\"delta_rad\":0.03}'}"
```

夹具开合：

```bash
ros2 topic pub --once /arachne/agent/command std_msgs/msg/String \
  "{data: '{\"tool\":\"gripper\",\"command\":\"open\"}'}"
```

安全停止：

```bash
ros2 topic pub --once /arachne/agent/command std_msgs/msg/String \
  "{data: '{\"tool\":\"safe_stop\"}'}"
```

## API Key

API key 只属于外部 Agent 进程，不属于 Arachne ROS 节点。建议放在：

```text
~/.config/arachne/agent.env
/etc/arachne/agent.env
```

参考模板：

```text
config/agent/agent.example.env
```

本仓库忽略真实 `.env`、`*.env`、`secrets/`、`credentials/`、`*.key` 和 `*.pem`。日志和事件中会对 key、token、secret、password 字段做基本脱敏。

## 当前限制

- `arm_cartesian_jog` 依赖 `/joint_states` 中有完整 Aubo 六轴关节。
- 机械臂 xyz 移动使用阻尼 Jacobian 速度映射，只适合小步示教式控制。
- 底盘相对移动需要 `grasp_task_server` 已启动并具备 `/odom`。
- Agent 不应把连续大动作拆成大量低层 jog；应逐步转向任务级 primitive，如 `pull_charger`、`insert_charger`。
