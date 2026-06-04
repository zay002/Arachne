# Aubo 示教柔顺控制方案

## 当前判断

机械臂“像踩着刹车开车”的核心原因不是 URDF，也不只是载荷参数，而是手动长按控制方式仍偏向短段位置轨迹：

- 底盘长按时持续发布速度命令，释放时发布 0，因此运动连续、停止自然。
- 机械臂长按时会不断生成很短的 `JointTrajectory` 位置目标，再由 `servoJoint` 追踪这些离散目标，容易表现为启停感、刹车感和抖动。
- 官方示教器/拖动/速度控制的手感更像连续速度或伺服流，而不是反复排队短轨迹。

因此 Arachne 的手动 jog 应改成“长按持续速度，释放归零”的模式；Home/Install/录制回放仍保留轨迹控制。

## 本版已补基础设施

1. `AuboHardwareInterface` 已接入 velocity command。
   - 当 ros2_control velocity command 非零时，驱动按控制周期积分成下一拍 `servoJoint` 目标。
   - 最大关节速度暂时钳制为 `0.45 rad/s`，避免误操作时过猛。
   - 非运行、teach gate、servo 恢复阶段会清零 velocity command 状态。

2. `aubo_smooth_controllers.yaml` 已预声明速度控制器。
   - 控制器名：`forward_command_controller_velocity`
   - 命令话题：`/forward_command_controller_velocity/commands`
   - 接口：`velocity`
   - 默认不激活，不影响当前一键示教启动。

3. 现有轨迹控制仍保留。
   - Home/Install、移动到指定关节角、录制回放继续走 `joint_trajectory_controller`。
   - 手动 jog 后续切换到 velocity controller。

## 推荐实现路径

### 第一步：真机小速度验证

启动当前真实硬件 bringup 后，先确认控制器：

```bash
ros2 control list_controllers
```

切到速度控制器：

```bash
ros2 control switch_controllers \
  --deactivate joint_trajectory_controller \
  --activate forward_command_controller_velocity
```

发布一个很小的 J6 速度，0.5 秒内立刻归零：

```bash
ros2 topic pub -r 20 /forward_command_controller_velocity/commands \
  std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 0, 0, 0.05]}"
```

另一个终端停止发布后，马上归零：

```bash
ros2 topic pub --once /forward_command_controller_velocity/commands \
  std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 0, 0, 0]}"
```

恢复轨迹控制：

```bash
ros2 control switch_controllers \
  --deactivate forward_command_controller_velocity \
  --activate joint_trajectory_controller
```

如果这一步明显比短轨迹 jog 柔顺，说明方向正确。

### 第二步：示教器改造成长按速度 jog

增加一个 `aubo_velocity_jog_bridge` 或直接在 `teach_panel.py` 内实现：

- 长按关节按钮时，以 `20-50 Hz` 发布 `Float64MultiArray` 速度。
- 释放按钮、窗口关闭、超过 `0.15-0.25 s` 没收到 UI 指令时，强制发布全 0。
- 按下第一个 jog 按钮时切到 `forward_command_controller_velocity`。
- 松开并稳定归零后，如需 Home/Install 或回放，再切回 `joint_trajectory_controller`。
- 每次切换前后都发布一次全 0，避免残留速度。

### 第三步：XYZ/RX/RY/RZ 连续 jog

笛卡尔 jog 不再每次求一个离散目标点，而是计算关节速度：

```text
qdot = J(q)^T * (J(q) * J(q)^T + lambda^2 I)^-1 * twist
```

其中：

- XYZ 移动时 `twist = [vx, vy, vz, 0, 0, 0]`，末端 RX/RY/RZ 保持不变。
- RX/RY/RZ 移动时只给角速度，位置保持不变。
- `lambda` 用 `0.05-0.12` 起步，靠近奇异点自动变大。
- `qdot` 再经过关节速度钳制、加速度限制和安全区预测。

安全区检查应按未来 `0.3-0.5 s` 的预测关节轨迹采样，如果末端、腕部或前臂采样点会进入置物架 keepout box，则把速度降为 0 并在 UI 告警。

### 第四步：如果仍抖，切到 SDK 速度接口

当前 Aubo SDK 头文件确认有：

- `MotionControl.speedJoint(qd, a, t)`
- `MotionControl.speedLine(xd, a, t)`
- `MotionControl.stopJoint(acc)`
- `MotionControl.stopLine(acc, acc_rot)`

如果 ros2_control velocity controller 仍然有刹车感，就新增一个直连 RPC/SDK 节点：

- `/arachne/aubo/joint_jog_velocity` -> `speedJoint`
- `/arachne/aubo/cartesian_jog_velocity` -> `speedLine`
- watchdog 超时 -> `stopJoint` 或 `stopLine`

这条路线更接近官方示教器速度控制，但需要更谨慎处理与 ros2_control 的互斥：进入 SDK 速度 jog 前暂停 `servoJoint` 写入，退出后同步 actual_q，再恢复轨迹控制。

## 建议默认参数

- 关节 jog 速度：`0.04-0.12 rad/s` 起步，确认稳定后再加。
- XYZ jog 速度：`0.015-0.04 m/s`。
- RX/RY/RZ jog 速度：`0.04-0.10 rad/s`。
- UI 发布频率：`30 Hz`。
- Watchdog：`0.2 s`。
- 关节加速度限制：`0.3-0.8 rad/s^2`。
- `servoJoint` 周期：继续保持 `0.005 s`。
- `servoJoint` lookahead：`0.12-0.16 s`。
- `servoJoint` gain：`100-150`，抖动时先降 gain 或增大 lookahead。

## 验证标准

1. 长按单关节时，关节速度曲线连续，没有明显一段一段的停顿。
2. 松开按钮后 0.2 秒内速度归零，不出现继续滑动。
3. XYZ jog 时末端姿态保持不变。
4. RX/RY/RZ jog 时末端位置基本保持不变。
5. 靠近置物架安全区时能主动限速或阻止。
6. Home/Install/回放仍可正常执行。
7. 带充电枪和相机负载时，运动不再出现明显“刹车式”抖动。
