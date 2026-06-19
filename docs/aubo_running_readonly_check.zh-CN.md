# Aubo Running/Normal 只读检查

本文记录 Phase 4C-1：从已验证的 PowerOff/Normal 只读状态，准备进入 Running/Normal 只读验证。该阶段仍然不发送任何运动命令。

## 当前已验证

- offline regression 通过。
- Aubo dry-run action smoke 通过。
- demo orchestrator offline smoke 通过。
- Aubo IP `192.168.127.128` ping 通，TCP `30004` open。
- 只读 RobotState 可读，已观察到 `RobotMode=PowerOff`、`SafetyMode=Normal`。
- dry-run real_bringup 下 `/joint_states`、`/arachne/hardware/aubo_status`、`/arachne/aubo/move_joint` 可观察。
- 未发送真实 motion goal。

## 目标

确认 Aubo 由人工或受控远程流程进入 `Running/Normal` 后，ROS graph 仍稳定：

- `/joint_states`
- `/arachne/hardware/aubo_status`
- `/arachne/aubo/move_joint`

Running/Normal 是 Phase 4C-2 current-state hold 的前置条件；它本身不代表可以发送 hold goal。

## 禁止项

- 不发送 `/arachne/aubo/move_joint` goal。
- 不调用 `speedJoint` / `moveJoint`。
- 不调用 Visual Grasp / Road Cleanup。
- 不做末端 jog。
- 不进入 freedrive/backdrive/handguide。
- 不运行 `grasp_preview_real_sync.sh --execute-real`。
- 不关闭 fallback。
- 不删除 `/tmp/arachne_aubo_teach_mode` 或 `/tmp/arachne_aubo_control_owner`，除非人工确认。

## 只读检查命令

```bash
AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_running_readonly.sh
```

脚本只做 TCP、只读 RobotState、ROS graph 和 stale 文件报告。若 RobotMode 不是 `Running`，脚本只报告，不尝试启动。

## 进入 Running/Normal 的方式

推荐由现场人员通过 Aubo 控制柜/示教器完成上电、启动和状态确认。

如确需远程启动，必须先由人工确认：

- 急停可用。
- 机械臂周围无人。
- 空间清空。
- 线缆不缠绕。
- 机器人无远程运动任务排队。

命令示例，仅供人工确认后执行；本阶段 Agent 不自动执行：

```bash
ARACHNE_CONFIRM_AUBO_DRIVER=YES ARACHNE_AUBO_ALLOW_PRESTART=YES \
./scripts/hardware/real_aubo_bringup.sh

ARACHNE_CONFIRM_AUBO_REMOTE_START=YES AUBO_ROBOT_IP=192.168.127.128 \
./scripts/hardware/real_aubo_remote_start.sh
```

remote start 不是运动目标，但可能涉及上电、启动、刹车/伺服状态变化，必须按真实硬件操作对待。

Gemini335 画面只能辅助观察，不替代现场急停、空间清空和人员确认。

## 下一阶段

Running/Normal 只读检查通过后，才允许讨论 Phase 4C-2 current-state hold。current-state hold 仍属于真实 command，必须单独由人工确认后执行。
