# Aubo 真实硬件只读检查

本文是 Phase 4B 的操作准备文档。目标是在连接 Aubo 真机后检查网络、mode/safety、ROS graph、`/joint_states`、AuboMoveJoint action server、teach bridge 和 velocity bridge 的状态。Phase 4B 不发送任何运动命令。

如果当前没有硬件，先运行 `docs/offline_regression.zh-CN.md` 中的离线回归；不要进入 Phase 4C。

## 禁止项

Phase 4B 禁止：

- 发送 `/arachne/aubo/move_joint` 真实 goal。
- 调用 `speedJoint`、`moveJoint`、`freedrive(true)`、`backdrive(true)`、`handguideMode`。
- 调用 `/arachne/demo/start_visual_grasp`。
- 调用 `/arachne/demo/start_road_cleanup`。
- 验证抓取、speedJoint jog、teach mode 或 road cleanup 执行。
- 关闭 teach panel、grasp task 或 pipeline 的 fallback。

## 连接硬件前检查

1. 确认 Aubo 控制柜、急停、示教器和网络连接处于人工可控状态。
2. 确认电脑与 Aubo 在同一网段，默认 Aubo IP 为 `192.168.127.128`。
3. 确认没有人正在通过示教器、SDK 或 ROS 发送运动命令。
4. 检查 stale 文件：

```bash
ls -l /tmp/arachne_aubo_teach_mode /tmp/arachne_aubo_control_owner 2>/dev/null || true
```

如果确认是 stale 文件，可由人工手动删除：

```bash
rm -f /tmp/arachne_aubo_teach_mode
rm -f /tmp/arachne_aubo_control_owner
```

不要让自动脚本删除这些文件。

## 一键只读检查脚本

```bash
source scripts/env/arachne_env.sh
source install/setup.bash
AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_readonly.sh
```

脚本只做：

- ping Aubo IP。
- 检查 TCP `30004` 可连接。
- 调用 `real_aubo_probe.py` 的只读 JSON-RPC：robot names、joint positions、tcp pose、RobotMode、SafetyMode。
- 检查 `arachne_hardware/action/AuboMoveJoint` interface。
- 观察 `/joint_states`、`/arachne/hardware/aubo_status`、`/arachne/aubo/move_joint` 是否已经存在。
- 列出 teach gate / control owner 文件，但不删除。

脚本不做：

- 不发送 action goal。
- 不调用 `speedJoint` / `moveJoint`。
- 不进入 freedrive/backdrive/handguide。
- 不写 teach gate。
- 不 claim real control owner。

## 网络检查

```bash
ping -c 2 -W 1 192.168.127.128
```

期望：有 ICMP reply。

```bash
python3 - <<'PY'
import socket
with socket.create_connection(("192.168.127.128", 30004), timeout=1.0):
    print("30004 open")
PY
```

期望：`30004 open`。

## Aubo Running / Normal 检查

只读 probe：

```bash
AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/real_aubo_probe.sh --ports 30004
```

期望：

- `RobotState.getRobotModeType` 能读到。
- `RobotState.getSafetyModeType` 能读到。
- 准备进入后续执行阶段前，RobotMode 应为 `Running`，SafetyMode 应为 `Normal` 或可接受的 ReducedMode。

Phase 4B 只记录状态，不执行恢复运动。

## 只读 ROS bringup 检查

推荐 dry-run graph 命令：

```bash
source scripts/env/arachne_env.sh
source install/setup.bash

ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=false \
  use_ms42dc:=false \
  use_aubo:=true \
  aubo_move_joint_dry_run:=true
```

说明：

- 该模式可启动 Aubo ROS graph 和 dry-run action server。
- `aubo_move_joint_dry_run:=true` 不代表真实 `moveJoint` 成功。
- Phase 4B 不允许发送真实 motion goal。
- 该模式只用于确认 ROS graph 能启动。

## `/joint_states` 检查

```bash
ros2 topic list | grep /joint_states
ros2 topic echo /joint_states --once
```

期望：

- topic 存在。
- message 中包含 Aubo 关节名称和当前位置。

失败时检查：

- `aubo_ros2_driver` 是否启动。
- 控制柜 IP 和网络是否正确。
- Aubo 是否 Running/Normal。
- driver 日志是否有 controller 或 RTDE/RPC 连接错误。

## `/arachne/hardware/aubo_status` 检查

```bash
ros2 topic list | grep /arachne/hardware/aubo_status
ros2 topic echo /arachne/hardware/aubo_status --once
```

期望：能看到 Aubo status/probe/bridge 输出。

如果没有输出：

- 检查 `aubo_official_status_probe` 是否启动。
- 检查 `real_bringup.launch.py use_aubo:=true`。
- 检查 Aubo IP、30004 和 mode/safety。

## `/arachne/aubo/move_joint` action server 检查

只检查存在性：

```bash
ros2 action list | grep /arachne/aubo/move_joint
ros2 action info /arachne/aubo/move_joint
```

禁止在 Phase 4B 发送：

```bash
ros2 action send_goal /arachne/aubo/move_joint ...
```

期望：

- action 存在。
- action type 为 `arachne_hardware/action/AuboMoveJoint`。
- 如果用 dry-run bringup，server 可启动但不会证明真实运动链路成功。

## teach bridge / velocity bridge 检查

只检查节点、topic 和日志存在性，不发布命令：

```bash
ros2 topic list | grep /arachne/aubo/teach_command
ros2 topic list | grep /arachne/aubo/joint_velocity_command
ros2 node list | grep aubo
```

禁止：

- 向 `/arachne/aubo/teach_command` 发布 `teach_on`。
- 向 `/arachne/aubo/joint_velocity_command` 发布速度。

## 常见失败症状

| 症状 | 可能原因 | 排查方向 |
| --- | --- | --- |
| ping 通但 `30004` 不通 | 控制柜 RPC 端口未开放、防火墙、Aubo 服务未运行 | 检查 Aubo 网络设置、控制柜服务、网段和线缆。 |
| `30004` 通但 mode/safety 读不到 | JSON-RPC 协议异常、robot name 不一致、控制柜状态异常 | 运行 `real_aubo_probe.sh --ports 30004`，查看具体 RPC error。 |
| driver 启动但没有 `/joint_states` | ros2_control driver 未连接、controller 未激活、Aubo 不在 Running/Normal | 查看 driver 日志、controller manager、Aubo mode/safety。 |
| action server 存在但状态不可用 | status probe 未启动或 Aubo RPC 不通 | 检查 `/arachne/hardware/aubo_status`、30004、launch 参数。 |
| teach_gate 文件残留 | 上次 teach/velocity/moveJoint 异常退出 | 先确认没有进程持有控制，再人工删除 `/tmp/arachne_aubo_teach_mode`。 |
| control_owner 文件残留 | 上次 SDK 控制进程异常退出或 pid stale | 先确认 owner pid 不存在，再人工删除 `/tmp/arachne_aubo_control_owner`。 |

## 恢复建议

1. 停止可能持有 Aubo 控制的 ROS node 或脚本。
2. 确认 Aubo 控制柜无运动、无远程控制请求。
3. 检查并人工清理 stale 文件。
4. 重启 Aubo driver / real bringup。
5. 重新确认 Aubo 控制柜 RobotMode/SafetyMode。
6. 重新检查 IP 网段、线缆和 30004。

当前 PowerOff/Normal 只读验证已通过；下一步是 Running/Normal 只读验证：

```bash
AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_running_readonly.sh
```

该检查只报告状态和 ROS graph，不尝试从 PowerOff 启动 Aubo。启动到 Running/Normal 需要现场控制柜操作，或人工确认后使用受控 remote start 流程。Phase 4C-2 才可以考虑进入“真实硬件低风险 hold/current-state 检查”，仍应从不改变目标姿态的状态保持类检查开始。

进入 Phase 4C 的最低条件：

- offline regression 通过。
- Aubo readonly check 通过。
- Running/Normal readonly check 通过。
- `/joint_states`、`/arachne/hardware/aubo_status`、`/arachne/aubo/move_joint` 只读检查正常。
- 人工确认不会发送新目标姿态或真实任务启动命令。
