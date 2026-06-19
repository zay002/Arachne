# 硬件

主入口：

```bash
./scripts/hardware/real_bringup.sh
./scripts/operator/teach_panel.sh
```

默认硬件：

- Scout 2.0：`/cmd_vel`，Waveshare USB-CAN-A。
- MS42DC：`/arachne/gripper/command`，状态 `/arachne/hardware/gripper_status`。
- Aubo i5：TCP `30004`，`/joint_states`，`/arachne/hardware/aubo_status`。
- Gemini335：末端 RGB-D。
- C16：雷达点云。

## Aubo Running/Normal

只读检查：

```bash
AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_readonly.sh
AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_running_readonly.sh
```

Running/Normal 是 current-state hold 和后续真实微动的前置条件，不等于允许自动抓取。

## 真机安全

启动前确认急停、空间、线缆和人员安全。`Gemini335` 画面只能辅助观察，不能替代现场确认。
