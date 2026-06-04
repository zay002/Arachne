# Jetson 分支 Aubo 抖动排查记录

本文记录 2026-06-03 对 `origin/jetson` 分支 Aubo i5 真机控制抖动问题的只读排查结论，供 Jetson 侧继续修改代码时参考。

## 现象

- Jetson 分支控制 Aubo 时抖动明显，柔顺性不如官方示教器。
- 末端有荷载时问题更突出。
- 这会影响后续演示，以及充电枪拔插这类需要稳定接触和低冲击运动的验收任务。

## 总体判断

这不像单个业务参数写错，更像是以下因素叠加：

1. 当前 ROS 路径使用外部高频 `servoJoint` 追点，而官方示教器大概率使用控制柜内部运动规划和轨迹执行。
2. Jetson / Ubuntu 22.04 / ROS 2 Humble 默认不是实时系统，200 Hz 外部控制周期容易被调度和网络抖动放大。
3. 当前轨迹目标多为 position-only 单点目标，缺少速度、加速度连续性。
4. 末端 payload 只是粗估，质量、重心和惯量不准会进一步放大跟踪误差。

所以，ROS 版本和 Jetson 平台可能是放大因素，但核心风险仍然是“外部高频 `servoJoint` + 非实时系统 + 轨迹不连续 + payload 粗估”的组合。

## 2026-06-04 当前修正

- 手动 jog 已改为默认发布到 `/arachne/aubo/joint_velocity_command`，由 `aubo_sdk_velocity_bridge` 调用 AUBO SDK `MotionControl.speedJoint(qd, a, t)`。
- SDK 速度桥进入手动速度控制前会暂停 ROS Driver 的 `servoJoint` 写入，松手、零速度、watchdog 超时和节点退出都会调用 `MotionControl.stopJoint(acc)`。
- 示教器增加速度命令 generation，松手后旧定时器中已经算出的非零速度会被丢弃，避免“轻点一下仍继续动”。
- 相机点云在一键示教和示教器 launch 中默认关闭，减少 Jetson Orin Nano 上对控制实时性的干扰。
- `check_real_hardware_env.sh` 的 AUBO TCP 默认检查端口改为 RPC `30004`，因为当前控制器 `80` 端口拒绝连接但 `30004` 正常。

## 关键证据

### 1. `servoJoint` 调用周期和 ROS 控制周期不匹配

相关文件：

- `third_party/aubo_ros2_driver/aubo_ros2_driver/config/aubo_controllers.yaml`
- `third_party/aubo_ros2_driver/aubo_ros2_driver/src/aubo_hardware_interface.cpp`

观察：

- `controller_manager.update_rate` 为 `200 Hz`，即控制周期约 `5 ms`。
- driver 的 `Servoj()` 中调用：

```cpp
servoJoint(traj, 0.2, 0.2, 0.01, 0.1, 200);
```

其中 `t=0.01`，即 `10 ms`。这和 `200 Hz` 的 `5 ms` 写周期并不一致。

AUBO ServoJ 官方文档强调，`t` 应与连续调用 `servoJoint` 的时间间隔匹配；轨迹点时间间隔不均匀会导致抖动。参考：

- https://docs.aubo-robotics.cn/aubo_sdk_docs/6_servoj_func_test/

### 2. `Servoj()` 内部存在阻塞循环

相关文件：

- `third_party/aubo_ros2_driver/aubo_ros2_driver/src/aubo_hardware_interface.cpp`

观察：

```cpp
while (true) {
  int servoJoint_num = ...->servoJoint(...);
  if (servoJoint_num != 2) {
    break;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
}
```

这段逻辑运行在 ros2_control hardware `write()` 链路里。它可能阻塞或拉长一次 write 调用，从而破坏 controller 的周期确定性。对 `servoJoint` 这种高频追点接口来说，这非常敏感。

### 3. driver 在启动时关闭了 `vff_enable`

相关文件：

- `third_party/aubo_ros2_driver/aubo_ros2_driver/src/aubo_hardware_interface.cpp`

观察：

```cpp
setHardwareCustomParameters("[joint_func] \n vff_enable = false\n");
```

目前没有在公开文档中确认该参数的精确定义。按命名推测可能与速度前馈有关。如果确实关闭了关节前馈，带负载轨迹跟踪可能更依赖反馈误差修正，表现会更硬、更容易抖。这个点不能直接改，需要在 AUBO 文档或官方支持里确认。

### 4. 示教面板发送的是短单点轨迹

相关文件：

- `src/arachne_operator/arachne_operator/teach_panel.py`
- `src/arachne_operator/launch/teach_panel.launch.py`

观察：

- TCP jog 默认 `0.003 m / 0.60 s`。
- wrist jog 默认 `0.75 deg / 0.60 s`。
- joint jog 默认 `0.5 deg / 0.60 s`。
- 每次 jog 都发送一个单点 `FollowJointTrajectory` goal。
- `JointTrajectoryPoint` 只填了 `positions` 和 `time_from_start`，没有填 `velocities`、`accelerations`。

这会让上层命令变成“短段目标、频繁重新规划/重定目标”。即使每段很小，也可能在起停处产生不连续。官方示教器的连续手动运动通常不会这样。

### 5. 回放轨迹也是单点 waypoint

相关文件：

- `src/arachne_operator/arachne_operator/teach_panel.py`
- `recordings/teach/demo_real_1.json`

观察：

- replay 默认对每个 waypoint 发送一个单点轨迹，默认时长 `3.75 s`。
- 录制文件中一些相邻 waypoint 的最大关节变化达到约 `1.9 rad`。
- 单点 waypoint 缺少速度/加速度边界和段间 blend，起停处容易顿挫。

### 6. payload 只是粗估，惯量为零

相关文件：

- `scripts/real_aubo_payload.py`
- `scripts/real_full_teach.sh`
- `scripts/real_full_acceptance.sh`

观察：

Jetson 分支默认：

```text
mass = 0.818 kg
cog = 0.039927,0.045067,0.143233
inertia = 0,0,0,0,0,0
```

质量和重心已按当前末端实测值更新。末端夹具、夹具安装板、线缆、被抓物体都会改变质量、重心和惯量；惯量目前仍为零估计，后续若继续抖动，应补充测量或估算主要惯量。

### 7. Jetson 环境缺少实时性约束

相关文件：

- `scripts/setup_jetson_humble.sh`
- `scripts/check_real_hardware_env.sh`

观察：

当前 Jetson setup 主要适配 Ubuntu 22.04 / ROS 2 Humble 软件环境，没有看到：

- 实时内核检查；
- CPU governor / nvpmodel / jetson_clocks 检查；
- CPU 隔离或进程优先级设置；
- ROS 控制周期 jitter 统计；
- Aubo TCP 网络延迟和丢包诊断。

在 `servoJoint` 高频外部控制模式下，这些都会影响实际手感。

## ROS 版本是否可能相关

可能相关，但更像放大因素。

Humble 和 Jazzy 的差异可能影响：

- `ros2_control` 和 `joint_trajectory_controller` 的插值/goal 处理；
- executor 和 timer 调度；
- DDS 默认行为；
- Ubuntu 22.04 与 24.04 的内核和系统调度差异。

但即便升级 ROS，也不一定解决。只要主链路仍然是非实时 Linux 上通过 TCP/RPC 高频喂 `servoJoint`，抖动风险仍然存在。

## 建议 Jetson 侧优先处理顺序

### 第一优先级：做无运动诊断

先不要盲目调参数。建议记录：

```bash
uname -r
ros2 control list_controllers
ros2 topic hz /joint_states
python3 scripts/real_aubo_payload.py --check-only
```

如果可以，再增加：

```bash
ping -i 0.01 <AUBO_IP>
sudo chrt -p $$
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null
```

目标是确认：controller 是否稳定 active、joint state 是否稳定、网络是否抖、payload 是否真的写入。

### 第二优先级：不要把演示主路径押在 `servoJoint`

对于演示和充电枪拔插，建议优先尝试 AUBO 控制器内部运动原语：

- `moveJoint`
- `moveLine`
- 控制器侧 blended waypoint

ROS 侧负责下发目标、监控状态、管理录制回放，但尽量让控制柜内部做轨迹插值和伺服控制。这样更接近官方示教器的手感。

### 第三优先级：如果继续保留 `servoJoint`

需要系统性改，而不是只调 gain：

1. 移除 hardware `write()` 里的阻塞 `while`。
2. 让 `servoJoint` 的 `t` 与真实下发周期一致。
3. 统计实际 write 周期 jitter。
4. 对输入轨迹做连续时间参数化，填充速度和加速度。
5. 避免频繁发送短单点 goal，改为连续 jog streaming 或控制柜侧 motion primitive。
6. 根据负载降低 gain / 增大 lookahead，并实际测振动响应。

### 第四优先级：校准 payload

需要实测或较准确估算：

- 夹具质量；
- 转接板质量；
- 线缆和夹持物体质量；
- 工具坐标系下 CoG；
- 主要惯量。

如果暂时无法测惯量，至少不要再回退到旧的 `3.5 kg, 0,0,0.18` 粗估值。

## 推荐给 Jetson Codex 的结论

优先不要在现有参数上小修小补。建议先实现一个对比验证：

1. 同一姿态下，Jetson ROS `servoJoint` hold 是否抖。
2. 同一姿态下，AUBO 控制器内部 `moveJoint` 到极小偏移是否顺滑。
3. 同一段 demo 轨迹，分别用 ROS `FollowJointTrajectory` 和 AUBO 内部 motion primitive 执行。

如果内部 motion primitive 明显顺滑，就应把演示/验收主路径迁移到控制柜侧轨迹执行；ROS 外部 `servoJoint` 只保留低速调试和小幅 jog。
