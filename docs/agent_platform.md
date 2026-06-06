# Agent Bridge

`arachne_agent_bridge` is the safe entry point for external agents. It does not include any LLM SDK and does not read API keys. It only exposes a bounded ROS JSON tool surface so Hermes, a web UI, an MCP service, or a custom agent can call Arachne without touching low-level hardware directly.

## Boundary

Recommended structure:

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

Agents should not directly control serial devices, the Aubo SDK, `/cmd_vel`, or joint controllers. Motion goes through tool whitelists, limits, and stop paths.

## Launch

Read-only mode, motion rejected by default:

```bash
./scripts/agent/agent_bridge.sh
```

Enable motion explicitly:

```bash
./scripts/agent/agent_bridge.sh \
  motion_enabled:=true \
  confirm_agent_motion:=true
```

Aubo teach/freedrive mode also requires:

```bash
allow_mode_change:=true
```

## API

- Commands: `/arachne/agent/command`
- Status: `/arachne/agent/status`
- Events: `/arachne/agent/event`
- Tool list: `/arachne/agent/tools`
- Stop service: `/arachne/agent/safe_stop`

Example:

```bash
ros2 topic pub --once /arachne/agent/command std_msgs/msg/String \
  "{data: '{\"tool\":\"arm_cartesian_jog\",\"axis\":\"z\",\"distance_m\":0.01}'}"
```

Supported tools include:

- `get_robot_state`
- `safe_stop`
- `base_velocity`
- `base_relative`
- `base_turn`
- `arm_cartesian_jog`
- `arm_joint_jog`
- `arm_stop`
- `gripper`
- `aubo_teach`

## Secrets

API keys belong to the external agent process, not to this ROS node. Store real secrets outside the repository, for example:

```text
~/.config/arachne/agent.env
/etc/arachne/agent.env
```

Use `config/agent/agent.example.env` as the template. Real `.env`, key, token, and credential files are ignored by git.
