# Aubo Current-State Hold 检查

Phase 4C-2 的 current-state hold 是第一个真实 command，不是 dry-run。它把 `/joint_states` 中当前 Aubo 6 轴关节角原样作为 `/arachne/aubo/move_joint` goal 发送，用于验证 action -> SDK -> Aubo 真实链路。

## 边界

- 目标等于当前姿态，不验证末端位移。
- 不做新目标姿态。
- 不做 Teach Panel jog。
- 不调用 `speedJoint`。
- 不进入 freedrive/backdrive/handguide。
- 不运行 Visual Grasp、Road Cleanup 或 `grasp_preview_real_sync.sh --execute-real`。
- 不关闭 fallback。
- 不删除 `/tmp/arachne_aubo_teach_mode` 或 `/tmp/arachne_aubo_control_owner`。

## 前置条件

- 人工确认急停可触达。
- 机械臂周围无人。
- 线缆没有缠绕或拉紧。
- Gemini335 或现场摄像头画面可见。
- Aubo 为 `RobotMode=Running`、`SafetyMode=Normal`。
- `/joint_states` 稳定。
- `/arachne/aubo/move_joint` action server 存在。

## 命令

```bash
ARACHNE_CONFIRM_REAL_AUBO_HOLD=YES \
AUBO_ROBOT_IP=192.168.127.128 \
./scripts/hardware/real_aubo_current_hold_check.sh
```

脚本会：

1. 运行 `check_aubo_running_readonly.sh`。
2. 确认 `/arachne/aubo/move_joint` action 存在。
3. 从 `/joint_states` 读取 `shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`。
4. 再读一次当前关节角，若最大差值超过 `0.005 rad` 则退出。
5. 发送 current-state hold goal：

```text
speed_rad_sec=0.05
accel_rad_sec2=0.10
goal_tolerance_rad=0.03
timeout_sec=5.0
label=current_state_hold_check
```

## 通过后的含义

通过后只能说明 `/arachne/aubo/move_joint` action、SDK guarded `moveJoint` 和 Aubo 真实链路可用。它不能说明 Visual Grasp、Road Cleanup、jog 或末端位移安全。

下一阶段才允许讨论 5 mm 级别单次微动；仍需再次人工确认。
