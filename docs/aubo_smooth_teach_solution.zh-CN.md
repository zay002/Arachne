# Aubo 示教柔顺控制方案

## 当前判断

机械臂“像踩着刹车开车”的核心原因不是 URDF，也不只是载荷参数，而是手动长按控制一度依赖 Jetson 外部高频 `servoJoint` 追点：

- 底盘长按时持续发布速度命令，释放时发布 0，因此运动连续、停止自然。
- 机械臂如果反复生成短段位置目标，或由 ROS 控制循环持续 `servoJoint` 追位置点，Jetson 的非实时调度、网络抖动和单点目标不连续都会被放大。
- 官方示教器的手感更接近控制柜内部速度跟随或连续伺服流，而不是频繁排队短轨迹。

因此 Arachne 的手动 jog 采用“长按持续速度，释放立即 stop/hold”的模式；定点移动和回放继续保留反馈闭环，但后续应再升级为控制柜侧的平滑轨迹接口。

## 当前实现

1. 手动 jog 默认走 SDK 速度桥。
   - 节点：`aubo_sdk_velocity_bridge`
   - 输入话题：`/arachne/aubo/joint_velocity_command`
   - SDK 调用：`MotionControl.speedJoint(qd, a, t)`
   - 松手、全零命令、watchdog 超时、节点退出都会优先调用 `MotionControl.stopJoint(acc)`。

2. SDK 速度桥会暂停 ROS Driver 的 `servoJoint` 写入。
   - 进入速度 jog 前写入 `/tmp/arachne_aubo_teach_mode`，让 `AuboHardwareInterface` 停止外部 `servoJoint` 保持。
   - SDK 速度桥只清理自己持有的 gate，避免误清 handguide/freedrive 的 gate。
   - 退出速度 jog 后同步回 ROS Driver 的 measured-position hold。

3. 示教器发布端做了 deadman 和 generation 防抖。
   - 长按按钮每 50 ms 刷新 deadman。
   - 松手立即发布 3 次零速度。
   - 松手后旧定时器中已经计算好的非零速度会因 generation 不匹配被丢弃，避免“轻点一下还一直动”。

4. `forward_command_controller_velocity` 保留为启动保持和回退路径。
   - 控制器名：`forward_command_controller_velocity`
   - 命令话题：`/forward_command_controller_velocity/commands`
   - 当前一键示教仍等待它 active，保证 AUBO ROS2 Driver 已经进入可控状态。

5. 相机点云默认关闭。
   - `real_full_teach.sh` 中 `ARACHNE_TEACH_CAMERA_POINTCLOUD=false`。
   - `teach_panel.launch.py` 中 `with_camera=false`、`camera_publish_pointcloud=false`。
   - 真机示教如果需要观察 2D 画面，可显式打开 color view，但点云不建议在手动精细控制时开启。

## 一键示教默认路径

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/arachne_env.sh
./scripts/real_full_teach.sh --yes
```

当前默认会：

- 启动 AUBO ROS2 Driver，并远程 power/startup。
- 配置 payload。
- 启动 SDK 速度桥。
- 启动底盘和夹具。
- 打开示教器和 RViz 可视化。
- 示教器机械臂 jog 话题默认为 `/arachne/aubo/joint_velocity_command`。

## 真机 A/B 验证建议

只读状态检查：

```bash
python3 scripts/real_aubo_probe.py --ip 192.168.127.128 --timeout 1.0
```

确认 SDK 桥是否启动：

```bash
ros2 node list | grep aubo_sdk_velocity_bridge
ros2 topic info /arachne/aubo/joint_velocity_command
```

在示教器中做最小风险验证：

1. 确认机器人 Running / Normal，急停在手边。
2. 先长按 J6 正负方向，每次不超过 1 秒。
3. 松手后确认 0.2 秒内停止，没有继续滑动。
4. 再测试 XYZ 和 RX/RY/RZ，观察末端是否连续、姿态/位置约束是否符合预期。
5. 最后再测试带充电枪和相机负载的低速连续 jog。

## 后续优化方向

- 定点移动和录制回放应从反馈速度法升级到 AUBO 控制柜侧的平滑轨迹接口，例如 `moveJoint`/`pathBuffer`/blend，而不是长期依赖 Jetson 反馈循环。
- 相机节点当前 V4L2 depth 路径仍偏重，后续应优先接 Orbbec 官方 SDK 或独立低优先级进程，避免影响机械臂控制实时性。
- 如果仍有轻微卡顿，优先调小 UI smoothing tau、调大 `speedJoint` 加速度，或把 `aubo_sdk_velocity_send_period_sec` 与控制柜实测周期做 A/B 对齐。
- payload 需要实测质量、重心和惯量；当前 3.5 kg / `0,0,0.18` 只是保守估计。

## 验收标准

1. 长按单关节时速度曲线连续，没有明显分段停顿。
2. 松开按钮后快速归零，不出现继续运动。
3. XYZ jog 时末端姿态保持稳定。
4. RX/RY/RZ jog 时末端位置基本保持稳定。
5. 靠近置物架安全区时主动限速或阻止。
6. Home/Install/回放仍可正常执行。
7. 带充电枪和相机负载时，不再出现明显“刹车式”抖动。
