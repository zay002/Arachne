# Phase 4A 验证矩阵

本文定义 AuboMoveJoint action、demo orchestrator、teach panel、grasp task server 和 grasp preview pipeline 的 dry-run / mock / ROS graph 回归流程。

Phase 4A 只验证接口链路稳定性，不验证真实机械臂运动成功。不得运行真实 Aubo/Scout/MS42DC motion goal，不得调用真实 `start_visual_grasp` 或 `start_road_cleanup`。

## 总原则

- `/arachne/aubo/move_joint` 是标准 Aubo joint execution 入口。
- `dry_run:=true` 只用于 ROS action 链路验证；它不连接 Aubo SDK、不调用 JSON-RPC、不写 teach gate、不 claim 真实 owner 文件。
- `dry_run:=false` 是默认值，真机默认安全策略不变。
- teach panel、grasp task server、grasp preview pipeline 的 fallback 保留；Phase 4A 不关闭 fallback。

## Mock / 静态验证

| 验证项 | 命令 | 期望输出 | 失败症状 | 排查方向 |
| --- | --- | --- | --- | --- |
| 离线总回归 | `./scripts/build/check_offline_regression.sh` | 静态、workspace、可用 ROS build 检查通过 | Python/shell/build 失败 | 优先修复第一个错误；该脚本不连接硬件、不发 goal。 |
| Aubo dry-run action smoke | `./scripts/test/smoke_aubo_move_joint_dry_run.sh` | `dry-run completed` 且 `success=true` | action server 未启动或 result 不匹配 | 确认已 build/source，且 server 参数为 `dry_run:=true`。 |
| Demo orchestrator offline smoke | `./scripts/test/smoke_demo_orchestrator_offline.sh` | `/status` 成功，`/preflight` 返回 checks payload | service/topic 不存在 | 确认 `arachne_operator` 已 build/source；不要调用 start services。 |
| Python 编译 | `python3 -m compileall src/arachne_hardware/arachne_hardware src/arachne_operator/arachne_operator scripts/vision` | 所有目标目录编译完成，无 SyntaxError | import/syntax error | 查看报错文件，优先检查 action import、Phase 3C client helper 和脚本缩进。 |
| shell 语法 | `bash -n scripts/build/check_aubo_action_stack.sh` | 无输出，退出码 0 | bash 语法错误 | 检查数组、引号、`set -u` 下的变量展开。 |
| workspace 静态检查 | `./scripts/build/check_workspace.sh` | workspace check 通过 | package/interface/build 检查失败 | 先确认环境 source，再看失败 package 的首个错误。 |
| Aubo action stack | `./scripts/build/check_aubo_action_stack.sh` | `Aubo action stack checks passed.` | interface 或 executable 查不到 | 先执行 colcon build 并 source `install/setup.bash`，确认 `arachne_hardware`、`arachne_operator` 已安装。 |

## ROS Graph Dry-run 验证

只允许在 `aubo_move_joint_dry_run:=true` 下发送 mock goal。该 goal 不代表真实机械臂到位。

启动 dry-run action 链路：

```bash
source scripts/env/arachne_env.sh
source install/setup.bash

ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:=false \
  use_ms42dc:=false \
  use_aubo:=true \
  aubo_move_joint_dry_run:=true
```

接口检查：

```bash
ros2 action list | grep /arachne/aubo/move_joint
ros2 action info /arachne/aubo/move_joint
```

期望：

- action list 中出现 `/arachne/aubo/move_joint`。
- action info 显示 action type 为 `arachne_hardware/action/AuboMoveJoint`。
- server 日志包含 dry-run ready/accepted/completed 等状态。

dry-run mock goal，仅限上面的 dry-run server：

```bash
ros2 action send_goal /arachne/aubo/move_joint arachne_hardware/action/AuboMoveJoint "{target_joints: [0,0,0,0,0,0], speed_rad_sec: 0.1, accel_rad_sec2: 0.1, blend_radius: 0.0, duration_sec: 0.0, goal_tolerance_rad: 0.04, timeout_sec: 3.0, label: 'dry_run_test'}"
```

期望 feedback：

```text
accepted
checking_state
motion_started
waiting_arrival
completed
```

期望 result：

```text
success: true
message: dry-run completed
final_error_rad: 0.0
```

失败症状与排查：

- action 不存在：确认 launch 是否传入 `use_aubo:=true`，并检查 `aubo_move_joint_action_server` 是否安装。
- interface type 不匹配：重新 colcon build `arachne_hardware` 并 source install。
- goal rejected 或无 feedback：查看 server 日志；确认 `target_joints` 长度为 6。
- 出现 JSON-RPC 或 owner/gate 日志：说明没有启用 dry-run，立即停止该验证。

## Demo Orchestrator Preflight-only 验证

Phase 4A 不调用 `/arachne/demo/start_visual_grasp` 或 `/arachne/demo/start_road_cleanup`。只允许检查 preflight/status 链路。

```bash
source scripts/env/arachne_env.sh
source install/setup.bash
ros2 launch arachne_operator demo_orchestrator.launch.py autostart:=false
```

安全服务检查：

```bash
ros2 service call /arachne/demo/preflight std_srvs/srv/Trigger {}
ros2 service call /arachne/demo/status std_srvs/srv/Trigger {}
```

期望：

- `/arachne/demo/status` 返回当前编排状态 JSON。
- `/arachne/demo/preflight` 只检查 action/service/process 可用性，不触发真实 motion。
- 如果底层 task server 未启动，preflight 可以失败，但失败信息应指出缺失服务，而不是崩溃。

失败症状与排查：

- `service not available`：确认 `demo_orchestrator` executable 已安装并启动。
- preflight action unavailable：先启动 dry-run Aubo action server，再重试。
- status JSON 缺字段：检查 `demo_orchestrator.py` 的 state publish 和 status response。

## Sim 验证

仿真仍应走接近真机的入口顺序，不添加绕过真实流程的 sim-only shortcut。

| 验证项 | 命令 | 期望输出 | 失败症状 | 排查方向 |
| --- | --- | --- | --- | --- |
| road cleanup 语义仿真 | `./scripts/sim/urban_trash_sorting_demo.sh` | RViz/Gazebo 中出现相机观察、目标获取、规划、drop-off 语义流程 | 直接跳到抓取或投篮 | 检查仿真流程是否绕过 camera-first / task server 风格顺序。 |
| mock road cleanup task | `python3 scripts/vision/mock_road_cleanup_task_test.py` | mock task smoke test 通过 | 状态机卡住或服务失败 | 查看 road cleanup task 的 preflight/status 服务和 mock 依赖。 |

## Real-dry-run 验证

real-dry-run 可以连接或观察真机状态，但不得发真实 motion goal。

| 验证项 | 命令 | 期望输出 | 失败症状 | 排查方向 |
| --- | --- | --- | --- | --- |
| Aubo 只读硬件检查 | `AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_readonly.sh` | ping、30004、只读 RPC、ROS interface 检查通过；不发送 goal | 网络/RPC/interface 失败 | 见 `docs/aubo_readonly_check.zh-CN.md`，先排查 IP、30004、RobotMode/SafetyMode、install/source。 |
| 真机底层接口只读 | `./scripts/hardware/real_bringup.sh` | driver/status topic 启动；不主动执行任务运动 | Aubo 状态不可用 | 检查网络、Aubo IP、driver package、急停/模式状态。 |
| Aubo action dry-run graph | `ros2 launch arachne_hardware real_bringup.launch.py use_scout:=false use_ms42dc:=false use_aubo:=true aubo_move_joint_dry_run:=true` | `/arachne/aubo/move_joint` 可见 | action server 不可见或接口错误 | 检查 install/source 和 launch 参数。 |
| teach panel 链路观察 | `./scripts/operator/teach_panel.sh` | GUI 启动，entrypoint 不变 | action unavailable 时应保留 fallback | 检查 `aubo_move_joint_action_timeout_sec` 和 `aubo_move_joint_fallback_internal`。 |

Phase 4B 只读硬件检查不验证抓取、不验证 speedJoint jog、不验证 teach mode，也不关闭 fallback。

Phase 5A 无硬件时使用 offline regression；有硬件但不允许运动时再使用 readonly check。当前 PowerOff/Normal 只读验证已通过；下一步是 Phase 4C-1 Running/Normal 只读验证，见 `docs/aubo_running_readonly_check.zh-CN.md`。current-state hold 还未执行，真机 motion 尚未验证。

Phase 4C-1 只读命令：

```bash
AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_running_readonly.sh
```

该脚本只报告 RobotMode/SafetyMode、`/joint_states`、`/arachne/hardware/aubo_status` 和 `/arachne/aubo/move_joint`，不启动 Aubo、不发 goal、不写 gate/owner。

## Real-execute 禁止项

Phase 4A 禁止以下操作：

- 禁止在 `dry_run:=false` 或不确定模式下发送 `/arachne/aubo/move_joint` goal。
- 禁止调用 `/arachne/demo/start_visual_grasp`。
- 禁止调用 `/arachne/demo/start_road_cleanup`。
- 禁止运行会运动 Scout/Aubo/MS42DC 的 acceptance 或 test goal。
- 禁止关闭 teach panel、grasp task 或 pipeline 的 fallback。
- 禁止修改 YOLO、点云、抓取规划和 road cleanup 状态机语义。

如果需要进入真实硬件只读检查，应进入 Phase 4B：只连接状态、`/joint_states`、mode/safety，不发送运动。
