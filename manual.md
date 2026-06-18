# Arachne Jetson 操作手册

更新日期：2026-06-18

本手册适用于当前 Jetson Orin Nano 上的 Arachne `jetson` 分支。

- 工作目录：`/home/jetson/zhaoyang/Arachne`
- 系统：Ubuntu 22.04
- ROS 2：Humble
- 机器人：Scout 2.0 底盘 + Aubo i5 机械臂 + MS42DC 夹具
- 当前模型：底盘、Aubo i5、夹具、后置传感器架、C16 雷达、末端 Orbbec 相机、车头吊篮
- 当前重点：真机施教、RViz/地图定位、Gemini335/C16 感知、TACO 垃圾 3D 候选过滤、Aubo SDK 抓取执行、road_clean 巡检/暂停/返航

## 1. 基础准备

进入工程目录：

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
```

如果远端 `jetson` 分支有更新，先同步并构建：

```bash
git fetch origin jetson
git pull --ff-only origin jetson
./scripts/build/build_workspace.sh
```

如果只改了施教器或描述模型，也可以局部构建：

```bash
./scripts/build/build_selected.sh arachne_description arachne_operator
source install/setup.bash
```

不要在已经 `source install/setup.bash` 的终端里直接裸跑 `colcon build --packages-select ...`。如果需要局部构建，优先使用 `./scripts/build/build_selected.sh`，它会先清理当前工作区的 underlay 残留，避免 `Some selected packages are already built in one or more underlay workspaces` 这类 warning。

当前 `scripts/` 已按功能分类整理，`scripts/` 根目录只保留说明文件和子目录：

```text
scripts/env        环境加载
scripts/build      构建和工作区检查
scripts/hardware   真机 bringup、Aubo、串口和验收
scripts/operator   施教器入口
scripts/vision     Gemini335、YOLO、TensorRT 和实时检测
scripts/model      URDF/TF/RViz 模型检查
scripts/sim        Gazebo 仿真验证
scripts/godot      Godot 展示前端
```

日常直接使用分类路径，例如 `source scripts/env/arachne_env.sh`、`./scripts/hardware/real_grasp_console.sh --yes --quick`、`./scripts/vision/gemini_yolo_live.sh`。新增脚本也应放进对应子目录，并同步更新 README 和 manual。

## 2. 真机测试前检查

开始前请确认：

- Scout 底盘、Aubo 控制柜、MS42DC 夹具已接线并供电。
- Aubo 控制柜网线已连接，默认 IP 为 `192.168.127.128`。
- 底盘 USB-CAN 和夹具串口已接到 Jetson。
- 急停或断电方式在手边。
- 机器人周围留出安全空间，尤其是后置传感器架和末端充电枪附近。

当前默认硬件参数：

```text
Scout USB-CAN: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
MS42DC:        /dev/serial/by-id/usb-1a86_USB_Single_Serial_58EB003416-if00
Aubo IP:       192.168.127.128
Aubo 型号:     aubo_i5
Aubo 负载:     0.818 kg, CoG 0.039927,0.045067,0.143233 m
```

检查环境：

```bash
./scripts/hardware/check_real_hardware_env.sh --strict
```

如果机械臂型号需要指定：

```bash
export AUBO_TYPE=aubo_i5
```

## 3. 不接真机的可视化

只查看带后置架、雷达、末端相机的新模型：

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
./scripts/model/view_sensor_model.sh
```

## 3.1 YOLO 垃圾/充电枪识别环境

YOLO 相关内容放在独立目录：

```text
/home/jetson/zhaoyang/Arachne/yolo_workspace
```

初始化 YOLO 环境：

```bash
cd /home/jetson/zhaoyang/Arachne
./scripts/vision/setup_yolo_env.sh
```

当前默认使用：

```text
yolo26n_seg_taco_best.pt   当前垃圾抓取默认模型，YOLO segmentation，TACO 垃圾类别
yolo26n.pt                 通用 COCO 检测备用权重
yolo26n-seg.pt             通用 COCO segmentation 备用权重
```

当前抓取链路默认使用 TACO segmentation 权重 `yolo26n_seg_taco_best.pt`，先用 mask 锁定垃圾轮廓，再在 mask 内做 depth ROI 和 3D 点云；如果 mask 不可用，才退回检测框 ROI。充电枪不是通用 COCO 类别，后续需要采集本机 Gemini335 下的充电枪样本并微调专用权重。INT8 导出前需要先采集本机 Gemini335 的代表性图片作为 calibration/validation 数据，不建议用通用样例数据做最终量化。

抓取链路不会在权重缺失时自动下载官方 YOLO 权重；如果 `ARACHNE_GRASP_YOLO_MODEL` 指向的本地文件不存在，会直接停止并提示修正路径。只有明确调试通用权重时才设置 `ARACHNE_GRASP_ALLOW_MODEL_DOWNLOAD=true`。

同步新代码后请确认本地权重存在：

```bash
ls -lh yolo_workspace/weights/yolo26n_seg_taco_best.pt
```

如果临时调试通用 YOLO 权重，才运行：

```bash
ARACHNE_GRASP_ALLOW_MODEL_DOWNLOAD=true ./scripts/vision/download_yolo_weights.sh
```

FP16 TensorRT 测试导出：

```bash
./scripts/vision/export_yolo_engine.sh yolo26n_seg_taco_best.pt fp16
```

INT8 导出需要先准备 `yolo_workspace/datasets/trash_mvp/images/val`：

```bash
./scripts/vision/export_yolo_engine.sh yolo26n_seg_taco_best.pt int8
```

只用 Gemini335 做实时 YOLO 标注预览，不启动底盘和机械臂：

```bash
./scripts/vision/gemini_yolo_live.sh
```

默认实时预览使用 `yolo26n_seg_taco_best.pt`、`imgsz=640`、`conf=0.25`，并且不做类别过滤。关闭检测窗口、按 `Esc` 或按 `q` 都会退出检测进程。

如果需要临时调整类别：

```bash
ARACHNE_YOLO_CLASSES=trash ./scripts/vision/gemini_yolo_live.sh
```

如果需要不做类别过滤：

```bash
ARACHNE_YOLO_CLASSES= ./scripts/vision/gemini_yolo_live.sh --classes ""
```

停止实时预览：

```bash
./scripts/vision/stop_gemini_yolo_live.sh
```

### Trash 抓取入篮路径预览

如果镜头里已经放了一个可抓取垃圾，可以启动快速 MVP 链路：

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
./scripts/vision/grasp_preview.sh
```

如果真机 Aubo 已经连上，先同步真实 6 轴姿态再启动同一套预览：

```bash
./scripts/vision/grasp_preview_real_sync.sh
```

只验证能否读到真机姿态，不启动相机/RViz/MoveIt：

```bash
./scripts/vision/grasp_preview_real_sync.sh --sync-only
```

这个 wrapper 会先清理旧的预览 display 节点，优先从真实 `/joint_states` 读取 Aubo 6 轴；如果 ROS driver 还没发布关节状态，则尝试通过 Aubo `30004` JSON-RPC 只读接口读取 `RobotState.getJointPositions`。读取成功后会把结果作为 `ARACHNE_GRASP_ARM_JOINTS` 传给 `grasp_preview.sh`，只同步 RViz/MoveIt 预览模型，不会向真机发送运动命令。

确认真机工作空间清空、急停可触达、Aubo driver 和夹具驱动已启动后，才允许让同一套预览规划自动下发到真机：

```bash
ARACHNE_CONFIRM_GRASP_EXECUTE_REAL=YES \
  ./scripts/vision/grasp_preview_real_sync.sh --execute-real
```

真机执行仍会先同步真实 6 轴姿态，再规划。当前默认抓取补偿为 `ARACHNE_GRASP_BASE_OFFSET=0,0,0`，现场偏差应优先通过 AprilTag 手眼标定更新相机外参，而不是长期依赖 base offset。轨迹下发前会检查真实 `/joint_states` 与规划第一帧是否接近；partial 轨迹默认拒绝执行。默认机械臂执行后端为 AUBO SDK JSON-RPC `MotionControl.moveJoint(q, a, v, blend_radius, duration)`：节点会选取少量关键关节目标，写入 `/tmp/arachne_aubo_teach_mode` 暂停 ROS driver 的 `servoJoint` 保持，逐段等待到位后再进入下一段；夹具命令走 `/arachne/gripper/command`。投放开爪后默认追加一次项目 home 姿态，home 来自 `scripts/env/arachne_real_defaults.sh` 的 `ARACHNE_AUBO_HOME_JOINTS_RAD`，可用 `ARACHNE_GRASP_REAL_RETURN_HOME=false` 临时关闭，或用 `ARACHNE_GRASP_REAL_HOME_JOINTS` 覆盖。普通 `grasp_preview.sh` 和不带 `--execute-real` 的 `grasp_preview_real_sync.sh` 仍然只做 RViz 预览。

### 人工操作 Grasp Task Server

`grasp_task_server` 是真实抓取的常驻入口。服务启动一次后，每次调用 `/arachne/grasp_task/start` 都会执行一轮完整流程：同步真机姿态 -> YOLO-SEG/TACO 分割 -> depth ROI 定位 -> MoveIt/本地规划 -> Aubo SDK 运动和夹具开合 -> 投放 -> 回 home。重复抓取时不需要重启 server。

现场优先使用施教器总控 console。这个入口会先打开施教器和 RViz，不再等待 Aubo 完全上电或等待 grasp server；相机、2D raw 画面、Localization/Nav、grasp server 都在施教器 `Home -> Runtime Services` 里按需启动/停止，Aubo 上电/启动也在施教器按钮里完成。

本机规划/调试：

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
./scripts/hardware/real_grasp_console.sh --yes --quick
```

如果要使用远端 MoveIt 2 + OMPL 规划，使用 remote wrapper。它会读取本地 `.env.local`，启动服务器规划栈，建立本地 SSH tunnel，然后进入同一个施教器总控：

```bash
./scripts/hardware/real_grasp_console_remote.sh
```

本地 `.env.local` 不提交到 Git，用来保存服务器地址、用户、端口和默认远端规划开关。首次配置可以参考 `.env.local.example`：

```bash
cp .env.local.example .env.local
```

如果真机或服务器启动慢，把 `.env.local` 里的 `ARACHNE_CONSOLE_WAIT_TIMEOUT_SEC` 调大；设为 `0` 表示各窗口一直等待依赖 topic/service，不主动超时退出。

常用控制：

```bash
./scripts/hardware/real_grasp_console_remote.sh status
./scripts/hardware/real_grasp_console_remote.sh stop
./scripts/hardware/real_grasp_console_remote.sh restart
```

`real_grasp_console_remote.sh` 默认等价于 `--yes --quick --terminal background`。console 只负责拉起底层守护和施教器，避免一次弹出大量终端；具体功能从施教器 Runtime Services 里启动。console 后台日志在 `log/real_grasp_console/latest/`，施教器自己启动的服务日志在 `log/teach_panel/latest/`。

如果需要现场看某个后台进程输出：

```bash
tail -f log/real_grasp_console/latest/*.log
```

如果临时想恢复多终端调试：

```bash
./scripts/hardware/real_grasp_console_remote.sh --terminal auto
```

当前 `--planner-backend remote` 会把本地感知得到的 tool0 关键位姿和当前 6 轴关节发给远端 `/plan`，由服务器上的 MoveIt 2/OMPL 做 `aubo_arm` 规划，再把返回的 joint frames 交给 Jetson 真机执行。Jetson 只负责相机、YOLO、真机执行和 UI，不再在本机跑 MoveIt 规划。

如果要临时禁用服务器规划，仅用 Jetson 本机调试：

```bash
ARACHNE_USE_REMOTE_PLANNER_DEFAULT=false ./scripts/hardware/real_grasp_console.sh --yes --quick
```

启动后先看总状态：

```bash
./scripts/hardware/real_grasp_status.sh
```

如果远程桌面/terminal 弹窗不稳定，用后台日志模式：

```bash
./scripts/hardware/real_grasp_console.sh --yes --quick --terminal background
tail -f log/real_grasp_console/latest/*.log
```

执行一轮视觉抓取推荐直接按施教器里的 `Visual Grasp`。它会自动启动相机、raw 画面和 grasp server，并等待 preflight 通过。命令行底层调试入口是：

```bash
ros2 service call /arachne/grasp_task/start std_srvs/srv/Trigger "{}"
```

停止当前任务：

```bash
ros2 service call /arachne/grasp_task/stop std_srvs/srv/Trigger "{}"
```

恢复规划恢复时的小幅底盘移动：

```bash
ros2 service call /arachne/grasp_task/restore std_srvs/srv/Trigger "{}"
```

查看任务状态和日志目录：

```bash
ros2 service call /arachne/grasp_task/status std_srvs/srv/Trigger "{}"
tail -f log/grasp_tasks/*/process.log
```

`start` 返回 `success=True` 只表示后台任务已启动，不表示抓取已完成。等状态进入 `succeeded`、`failed` 或 `canceled` 后，重新摆放目标，再按一次 `Visual Grasp` 或再次调用 start 服务。

首次同步新代码后构建一次：

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
colcon build --packages-select arachne_operator arachne_agent_bridge --symlink-install
source install/setup.bash
```

日常视觉抓取直接点击施教器顶部或 `Home` 页里的 `Visual Grasp`。它会自动启动 Gemini Camera、2D Raw View 和 Grasp Server，等待相机 color/depth、Aubo、夹具等 preflight 通过后再开始抓取；默认使用娃娃机式垂直逼近，close 后通过 MS42DC 反馈判断是否空抓，空抓时会重新拍摄并最多重试 3 次。`Grasp Start` 是底层服务调试入口，不会自动拉起依赖服务。raw 画面订阅 `/camera/color/image_raw`，抓取开始后可以继续动态观察末端运动；YOLO 在目标锁定后会暂停重复检测，但 raw 画面不依赖标注图刷新。`real_grasp_console.sh` 默认使用 320x240 彩色流保证远程桌面流畅，深度仍保持 640x480；如需高分辨率彩色流，可设置 `ARACHNE_CONSOLE_CAMERA_COLOR_WIDTH=640 ARACHNE_CONSOLE_CAMERA_COLOR_HEIGHT=480`。

道路垃圾巡检使用同一个施教器入口：

1. 确认 `Camera`、`2D Raw View`、`Grasp Server`、`Road Cleanup Server` 已启动；`grasp_server` 空闲时会持续用 YOLO-SEG 发布 `/arachne/perception/taco_instances`。
2. 点击 `Road Preflight` 做抓取 primitive 检查。若这个按钮偶发超时，先看 `Grasp Preflight` 或 `ros2 service call /arachne/grasp_task/preflight std_srvs/srv/Trigger "{}"`；底层抓取 preflight 通过即可继续测试。
3. 点击顶部 `Road` 或 Home 页 `Road Start`，底盘开始按仿真 `box_entry` 轨迹巡检：入口前进 `0.3 m`，再进入 `1.0 m x 1.2 m` 矩形环绕。
4. 测试中需要立刻暂停时点 `Road Pause`；它会停止底盘和当前抓取，状态保持 `paused`，便于现场观察或人工干预。
5. 需要回到 road_clean 起点时点 `Return` / `Road Return`；server 会按已完成的底盘段反向 replay。若还没有完成任何底盘段，不会移动，只会返回 `return home complete: no completed base legs`。
6. 检测到带 3D 位姿且可达的目标后任务服务器会停底盘，调用 `/arachne/grasp_task/start` 完成点云 ROI、规划、Aubo SDK `MotionControl.moveJoint` 抓取和投篮。

当前 road_clean 会忽略过远或不可达目标，避免看到 2D 检测后机械臂原地不动还反复失败。默认过滤条件：

```text
require_3d_candidate = true
candidate_min_base_x_m = 0.25
candidate_max_base_x_m = 1.03
candidate_max_abs_base_y_m = 0.60
candidate_min_base_z_m = -0.18
candidate_max_reach_m = 1.03
candidate_max_depth_m = 0.85
```

`candidate_max_reach_m` 来自 2026-06-18 的实机边界标定：人工把机械臂摆到最远可抓位置后，用 Aubo FK 算得 `grasp_frame` 在 `base_link` 下为 `[1.086, -0.014, -0.096] m`，水平半径 `1.0865 m`。road_clean 默认使用 `1.03 m`，留约 5 cm 裕量；标定记录在 `config/real_road_demo_grasp.yaml`。`candidate_min_base_z_m=-0.18` 允许低位抓取，同时继续挡掉接近 `z=-0.19 m` 的地面反光点。

2D-only 事件只用于视觉预览，不触发抓取；3D 事件来自 `grasp_preview_pipeline`，包含 `depth_m`、`base_grasp_xyz` 和规划关键点。被过滤的候选会在 `/arachne/road_cleanup/event` 里以 `candidate_ignored` 记录原因。

如果 YOLO 和点云正常但目标在过滤范围内仍规划失败，`road_cleanup_task_server` 会进入 reach recovery：沿当前巡检方向小步移动底盘，向 `/arachne/grasp_preview/restart_search` 周期性发重搜信号，清掉旧候选，等待新检测后重新计算点云和抓取规划。默认最多 3 次，每次 0.10 m；超过次数后记录 skip 并继续巡检。

命令行底层调试入口：

```bash
./scripts/vision/road_cleanup_task_server.sh \
  patrol_pattern:=box_entry \
  patrol_box_width_m:=1.0 \
  patrol_box_height_m:=1.2 \
  patrol_entry_m:=0.3 \
  require_3d_candidate:=true \
  candidate_max_depth_m:=0.85
ros2 service call /arachne/road_cleanup/start std_srvs/srv/Trigger "{}"
ros2 service call /arachne/road_cleanup/pause std_srvs/srv/Trigger "{}"
ros2 service call /arachne/road_cleanup/return_home std_srvs/srv/Trigger "{}"
ros2 service call /arachne/road_cleanup/status std_srvs/srv/Trigger "{}"
```

不接真机的状态机回归：

```bash
source install/setup.bash
python3 scripts/vision/mock_road_cleanup_task_test.py
```

示教器可以和 server 同时开着，但不能同时下发机械臂动作。示教器空闲时不占控制权；只有 `teach_on/teach_off` 或手动 jog 正在执行时，才会写入 `/tmp/arachne_aubo_control_owner`。真实抓取发 `moveJoint` 前也会独占这个 owner，并写 `/tmp/arachne_aubo_teach_mode=1` 暂停 ROS driver 的 `servoJoint` 保持。若 preflight 提示 `aubo_control_owner` 或 `aubo_teach_gate` busy，先停止手动 jog 或发送 teach off，再重新 start。

如果中途目标放错或想重新识别：

```bash
ros2 service call /arachne/grasp_task/stop std_srvs/srv/Trigger "{}"
ros2 service call /arachne/grasp_task/start std_srvs/srv/Trigger "{}"
```

#### 5. 日志

```bash
TASK_DIR="$(ls -td log/grasp_tasks/* | head -1)"
tail -f "${TASK_DIR}/process.log"
tail -f "${TASK_DIR}/events.jsonl"
```

真实动作完成时，`process.log` 里会出现：

```text
REAL moveJoint ... grasp:close
REAL gripper command: close
REAL moveJoint ... basket_over:open
REAL gripper command: open
REAL moveJoint ... home
REAL arm SDK moveJoint sequence complete
```

坐标系参考：

- `base_link`：小车车体坐标，+X 指向车头，+Y 指向小车左侧，+Z 向上。篮筐、keepout、安全区、抓取规划路径都最终落在这个坐标系下。
- `odom` / `map`：`odom -> base_link` 来自底盘里程计；`map -> odom` 属于定位系统，不写进 URDF。
- `aubo_base_link`：Aubo 底座坐标，固定挂在 `base_link` 上；MoveIt 规划会把 `base_link` 下的抓取目标转换到这里求解。
- `tool0`：Aubo 法兰中心。当前夹具/相机安装角度通过 `tool_adapter_rpy` 修正，默认绕法兰盘逆时针 45 度。
- `gripper_adapter_link` / `grasp_frame`：夹具安装座和抓取 TCP。MS42DC 的 `grasp_frame` 按闭合夹具模型计算：取闭合指尖端点平面中心，再沿指尖到法兰中心方向内收 2 cm，当前为法兰坐标下约 `(0, 0, 0.138692)` m；规划默认使用这个 URDF frame。
- `ee_camera_link` 和 depth camera frame：末端 RGB-D 相机坐标。YOLO 只锁 2D 目标，深度 ROI 先在相机深度 frame 中投影成 3D 点，再经 TF 转到 `base_link`。

抓取位置的现场补偿使用 `base_link` 下的米制偏置 `ARACHNE_GRASP_BASE_OFFSET=x,y,z`，只移动规划用的 approach/grasp/lift 目标，不移动原始 ROI 点云或篮筐。当前默认设置为 `0,0,0`；如果只做短期现场补偿，可以这样覆盖：

```bash
ARACHNE_GRASP_BASE_OFFSET=0.04,0.10,-0.06 ./scripts/vision/grasp_preview_real_sync.sh
```

AprilTag 手眼标定用于求解真实 `tool0 -> camera_color_optical_frame` 外参。当前实机使用 `tagStandard41h2`，可直接用交互脚本同时看相机画面、按空格采样、按 `s` 求解：

```bash
./scripts/vision/apriltag_hand_eye_interactive.sh
```

快捷键：

```text
Space  capture 当前 tag + 当前 Aubo 姿态
s      solve 并保存 hand_eye_*.json
r      reset 清空样本
q      quit
```

建议至少采 12 组以上，姿态要有明显平移和旋转变化。最新一次采用 H12 / `tagStandard41h2` 标定后，外参已写入 `src/arachne_description/config/physical_parameters.yaml`，并同步到 Gemini335 launch 和施教器相机命令。当前默认发布：

```text
tool0 -> camera_color_optical_frame
xyz = -0.239469796, 0.181459396, 0.190102132
rpy =  0.083404947,-0.300045345, 3.128380060
```

AprilTag 也曾用于教室建图前确定初始朝向：车头正对墙面 tag，按 `tagStandard41h2` 初始化建图姿态。但这是一次性建图辅助，不属于正常 road_clean 流程。后续默认使用已保存地图 `src/arachne_nav/maps/road_lab_apriltag.yaml` 和定位链路同步 RViz 中小车位姿，不再在每次启动 road_clean 时依赖 AprilTag。

抓取姿态可以在 RX/RY/RZ 上搜索多个候选，但默认按“娃娃机”方式从上往下接近。`--grasp-topdown-max-tilt-deg` 限制夹具 z 轴偏离向下方向的最大角度，`--ground-min-z-base`、`--ground-clearance` 和 `--tool-ground-clearance` 会拒绝任何机械臂连杆、tool0 到 grasp TCP 的夹具线段低于地面安全线的候选。

真实抓取时夹具事件按语义关键点触发：到达 `grasp` 后才 close，`safe_mid` 保持抓取姿态抬离目标，不在合爪/刚抬起阶段切换释放姿态；到 `basket_over` 后才 open。规划如果在真机开始运动前失败，server 会按 `planning_recovery_base_sequence` 做小幅底盘移动、重新拍摄和重新规划；默认序列是 `forward:0.04,back:0.08,turn_left:5deg,turn_right:10deg`，全部失败后会按反向动作尽量恢复原位并宣布失败。

只调矿泉水瓶这类目标的抓取点/方向时，可以只启动相机和感知，不做 IK/MoveIt/真机执行：

```bash
ARACHNE_GRASP_START_MOVEIT=false \
ARACHNE_GRASP_WITH_RVIZ=false \
ARACHNE_GRASP_EXECUTE_REAL=false \
./scripts/vision/grasp_preview.sh --planner-backend none
```

检测锁定后看 `yolo_workspace/runs/grasp_preview/latest_grasp_preview.json` 里的 `pointcloud_grasp_shape`。其中 `axis_confidence` 来自 ROI 点云 3D PCA，`visual_axis_confidence` 来自 YOLO mask 的 2D PCA；矿泉水瓶点云只看到上半部分时，系统会用可见高度做小幅向下补偿，并优先用 2D+3D 方向作为 top-down 抓取候选。

这个脚本会默认启动：

- Arachne 模型 TF，不控制真机。
- MoveIt 2 `move_group`，使用 OMPL 做 Aubo 轨迹规划。
- Gemini335 RGB-D 相机。
- RViz 抓取预览界面。
- YOLO-SEG/TACO 分割、mask ROI 点云、深度测量和抓取入篮路径预览节点。

抓取预览依赖 `base_link -> ee_camera_link` 的 TF。真机没电或没有 Aubo driver 时，RViz 里的机械臂不会自动同步到真机末端姿态，相机坐标会随模型关节角偏掉。当前模型默认 Home 已固定为 2026-06-05 从真机读取的位姿：

```text
source: Aubo RPC RobotState.getJointPositions
robot: rob1, mode=Running, safety=Normal
joints: -1.5707963267949,0.201570428261868,1.65970467002488,0.485178041391533,1.67675136677345,0.76432946885334
```

真机在线时建议使用同步封装脚本，它会先读取当前 `/joint_states` 或 Aubo RPC，再启动预览：

```bash
./scripts/vision/grasp_preview_real_sync.sh
```

如果需要手动指定机械臂当前姿态：

```bash
ARACHNE_GRASP_ARM_JOINTS="-1.5707963267949,0.201570428261868,1.65970467002488,0.485178041391533,1.67675136677345,0.76432946885334" \
  ./scripts/vision/grasp_preview.sh
```

这些默认值记录在 `src/arachne_description/config/real_hardware_defaults.yaml` 和 `scripts/env/arachne_real_defaults.sh`。

RViz 中重点看：

```text
/arachne/grasp_preview/markers    检测框、抓取点、入篮路径、篮子 keepout
/arachne/grasp_preview/roi_cloud  mask 内用于估深的点云；无 mask 时退回检测框 ROI
/arachne/grasp_preview/path       base_link 下的抓取到入篮路径
```

节点会把最近一次结果保存到：

```text
yolo_workspace/runs/grasp_preview/latest_annotated.jpg
yolo_workspace/runs/grasp_preview/latest_grasp_preview.json
```

当前路径只是可视化规划，不会控制机械臂。默认流程是：

```text
SEARCH_2D: YOLO-SEG 持续寻找 TACO 垃圾目标
-> SNAPSHOT_3D: 目标确认后只拍一次深度 ROI / 点云
-> PLAN_LOCKED: 锁定抓取点和入篮路径，暂停 YOLO 2D 推理
-> 执行/调试结束后重新开始 SEARCH_2D
```

这样做的目的是把 YOLO 当作高速 2D 搜索前端：视野里看到目标后，只在目标变化时触发 3D；规划和抓取阶段不再持续跑 2D 推理，减少 Jetson 负载，也避免深度噪声让抓取点每帧跳动。当前 demo 默认会在第一次有效 3D 快照后暂停 YOLO，并继续发布缓存的点云、抓取点和曲线轨迹。

重新开始搜寻：

```bash
ros2 topic pub --once /arachne/grasp_preview/restart_search std_msgs/msg/Empty "{}"
```

入篮轨迹当前不是简单直线连接，而是由语义关键点和约束共同生成的预览轨迹：短距离接近和释放段保留可控直线，抓取后转运到篮子上方先用抬高控制点的曲线作为参考输入。RViz 里的 `/arachne/grasp_preview/path` 是用于显示和任务理解的采样 base_link 参考曲线；机械臂动画不会实时追这个小球，而是在锁定计划时把关键目标交给 MoveIt 2 `/plan_kinematic_path`，由 OMPL 在完整 Arachne URDF 碰撞模型里规划 Aubo 关节轨迹，再按预览速度、加速度和 jerk 限制重新生成固定频率 joint trajectory。

默认 planner 是快速的：

```text
RRTConnectkConfigDefault
```

PRM 和 RRTstar 已经在 OMPL 配置里保留；需要对比时可以手动指定：

```bash
./scripts/vision/grasp_preview.sh \
  --moveit-planners RRTConnectkConfigDefault,PRMkConfigDefault,RRTstarkConfigDefault
```

如果需要临时退回旧的本地 IK 预览，可追加：

```bash
./scripts/vision/grasp_preview.sh --planner-backend local
```

如果只想调相机/YOLO，不启动 MoveIt：

```bash
ARACHNE_GRASP_START_MOVEIT=false ./scripts/vision/grasp_preview.sh --planner-backend local
```

当前完整任务路径包含这些语义关键点：

```text
start_ee       当前末端起始位置，默认使用 grasp_frame
approach       抓取前接近位
grasp          抓取位置
safe_mid       中间安全位置
drop           篮子上方丢放位置
observe_start  回到观察/初始位置
```

RViz 中会同时显示：

- `/arachne/grasp_preview/path`：完整采样路径。
- `/arachne/grasp_preview/markers` 里的 `task_waypoints`：上述关键点和文字标签。
- `/arachne/grasp_preview/markers` 里的 `path_playback`：洋红色播放游标，会按已生成的受限关节轨迹进度移动，表示执行顺序；默认播放到末端后保持，不自动回初始位。
- RViz 的 RobotModel：机械臂会按默认 80 Hz 播放 MoveIt 返回并重新限速后的 joint trajectory frame。每帧包含关节位置、速度和加速度，位置/速度发布到 `/arachne/grasp_preview/joint_states`；默认播放一次并保持末端姿态，经过模型用的 joint_state_mux 进入 RViz，不控制真机。
- 真机执行模式：只有显式设置 `ARACHNE_CONFIRM_GRASP_EXECUTE_REAL=YES` 并使用 `--execute-real` 时才会下发轨迹；执行前会拒绝起始关节误差过大的轨迹，投放开爪后默认回到项目 home 姿态。

Grasp preview 的模型可视化使用专用 `/arachne/display/joint_states` 和 `/arachne/display/robot_description`，并把预览机器人 link/TF frame 加上 `grasp_preview_` 前缀；全局 `base_link` 只通过一条静态 TF 接到 `grasp_preview_base_link`。这样全局 `/joint_states`、真机/mock 的 `robot_state_publisher`、以及预览 RobotModel 使用的 TF 树彼此隔离。启动脚本会清理旧的预览专用 display RSP/static bridge，避免残留预览 TF 与新预览 TF 冲突。

为了避免夹具碰到篮子，预览节点会画出 `basket_keepout` 半透明红色盒子。URDF 中车头吊篮碰撞体约为 `x=[0.4415,0.6455] y=[-0.09,0.09] z=[-0.0735,0.0135]`；当前默认 keepout 在 X/Y 方向各外扩 2 cm，在 Z 方向各外扩 5 cm，即 `x=[0.4215,0.6655] y=[-0.11,0.11] z=[-0.1235,0.0635]`。路径检查时会额外按 `gripper_radius` 扩展这一区域做采样判断；如果 RViz 文本显示 `basket collision risk`，先不要按这个轨迹做真机运动。

实时日志：

```bash
tail -f yolo_workspace/runs/gemini_yolo_live/latest.log
```

用 mock 硬件打开施教器和融合 RViz，不会控制真机。当前推荐让 mock `robot_state_publisher` 作为唯一整车 TF 源；施教器只开面板，RViz 单独打开，避免两个 `robot_state_publisher` 同时发布同名 TF：

终端 1：

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
ros2 launch arachne_control mock_ros2_control.launch.py gripper_type:=ms42dc
```

终端 2：

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
ros2 launch arachne_operator teach_panel.launch.py \
  with_camera:=false \
  with_visualization:=false \
  arm_replay_backend:=velocity_stream \
  arm_manual_prefer_topic:=true
```

终端 3，可选打开 C16 雷达。如果只想看模型可跳过这一步：

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
ros2 launch lslidar_driver lslidar_cx_launch.py
```

终端 4，打开融合 RViz：

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
rviz2 -d src/arachne_description/rviz/arachne_lidar_fusion.rviz
```

当前 C16 配置位于 `third_party/Lslidar_ROS2_driver_C16_V4/lslidar_driver/params/lslidar_cx.yaml`：`device_ip=192.168.1.200`、`msop_port=2368`、`difop_port=2369`、`frame_id=lidar_link`、`topic_name=/lslidar_point_cloud`、`distance_unit=0.4`。融合 RViz 固定坐标为 `lidar_link`，即“雷达图中放入整车模型”；如果现场比例不对，优先调整 `distance_unit`，不是缩放车模。

这个模式适合检查：

- RViz 中新模型是否正常加载。
- `base_link -> rear_rack_link -> lidar_link` 是否存在。
- `tool0 -> gripper_adapter_link -> ee_camera_support_link -> ee_camera_link` 是否存在。
- 施教器的 X/Y/Z、RX/RY/RZ、关节点动和 Home 是否能在 RViz 中连续运动。

查看 ROS 节点结构：

```bash
rqt_graph
```

生成 TF 图：

```bash
mkdir -p log/live_tools
cd log/live_tools
ros2 run tf2_tools view_frames
```

生成后查看：

```bash
xdg-open frames.pdf
```

## 4. 一键启动真机施教器

确认环境安全后执行：

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
./scripts/hardware/real_grasp_console.sh --yes --quick
```

这个入口会自动完成：

1. 读取 `.env.local` 和真机默认参数。
2. 停掉旧的 real stack 残留进程。
3. 启动施教器和 RViz 可视化。
4. 启动 Aubo ROS2 driver 的 guarded prestart。
5. 启动 Scout 底盘和 MS42DC 夹具 bringup。
6. 把相机、2D raw 画面、Localization/Nav、grasp server 的启停交给施教器 Runtime Services。

默认不会自动远程执行 Aubo `poweron/startup`，施教器不需要等机械臂完全上电才能打开。需要远程上电时，在施教器里点 `Aubo On` / `Aubo Start`；如果确实要恢复旧的自动上电流程：

```bash
ARACHNE_CONSOLE_AUTO_AUBO_START=true ./scripts/hardware/real_grasp_console.sh --yes --quick
```

如果启动时就自动打开相机、2D 画面或 grasp server：

```bash
ARACHNE_CONSOLE_AUTO_CAMERA=true \
ARACHNE_CONSOLE_WITH_VIEWER=true \
ARACHNE_CONSOLE_AUTO_GRASP_SERVER=true \
./scripts/hardware/real_grasp_console.sh --yes --quick
```

## 5. 施教器操作要点

施教器当前版本的行为：

- 底盘控制：长按前后左右按钮运动，松开停止。
- 机械臂 X/Y/Z：长按运动，松开后发送 hold current。
- 机械臂 X/Y/Z 点动时会保持当前末端 RX/RY/RZ 不变。
- RX/RY/RZ：长按调整末端姿态。
- 单关节：可长按 J1-J6 单独点动。
- 目标移动：可输入指定关节角度或指定 TCP 的 X/Y/Z 位置。
- Home / Install：顶部、`Home` 页和 `Move` 页面都有长按移动按钮。
- Aubo On / Aubo Start / Aubo Off：施教器内远程上电、启动和断电，不要求面板启动前机械臂已经 Running。
- Runtime Services：在 `Home` 页启动/停止 Gemini Camera、2D Raw View、Localization/Nav、Grasp Server。Quick Control 里的 `Visual Grasp` 是推荐抓取入口，会自动启动相机、raw 画面和 grasp server 并等待 preflight；`Camera` 只启动相机驱动和 raw 画面。raw viewer 默认限制到 15 FPS，console 默认彩色流为 320x240、深度为 640x480，避免远程桌面卡顿。
- Road Start / Road Pause / Return / Road Stop：调用 road cleanup server 启动巡检、暂停、按已完成底盘段返航、取消任务。现场调试优先用 `Road Pause`，保留现场状态；`Road Stop` 用于直接取消。
- Grasp Start / Grasp Stop / Restore：调用常驻 grasp server 执行抓取、停止任务、恢复规划恢复时的小幅底盘移动。
- 预设配置：`Configure` 页面可设置 Home / Install 位姿，并保存/加载本地配置。
- 录制回放：`Program` 页面录制 waypoint、wait、保存、加载、回放。
- 日志：面板状态和硬件状态变化写入 `log/teach_panel/latest/events.jsonl`；施教器启动的服务各自写入 `log/teach_panel/latest/<service>.log`。
- 窗口缩放：各页面支持滚动，小窗口或远程桌面下不会裁掉底部控件。

Localization/Nav 按钮会启动 `real_lidar_nav.sh`，默认使用 `src/arachne_nav/maps/road_lab_apriltag.yaml` 进入定位模式，链路是 `LSlidar C16 /lslidar_point_cloud -> pointcloud_to_laserscan /scan -> AMCL map->odom -> Nav2`。当前真机施教器默认不自动打开 topdown RViz，避免 Jetson 负载过高；需要俯视定位窗口时再显式启动 `scripts/hardware/real_lidar_nav.sh` 或设置对应 Nav/RViz 开关。topdown RViz 固定坐标系为 `map`，小车位姿来自 `map -> odom -> base_link`。使用 RViz 顶部 `Nav2 Goal` 工具可以给小车定点导航；如需重新建图，显式设置 `ARACHNE_NAV_MODE=mapping` 后再启动 `scripts/hardware/real_lidar_nav.sh`。

默认 Home 和 Install 位姿：

```text
J1=-90.00, J2=11.55, J3=95.09, J4=27.80, J5=96.07, J6=43.79 deg
```

Home / Install 是 6 个关节角，单位为 degree。使用方式：

- 长按 `Hold Home` 或 `Hold Install` 才会自动移动。
- 松开按钮会发送停止/保持当前位置。
- 在 `Configure` 页面编辑 `Home joints deg` 或 `Install joints deg`，然后点击 `Apply`。
- 也可以点击 `Set Home From Current` / `Set Install From Current`，用当前机械臂姿态写入预设。

当前默认点动参数：

```text
TCP step:       0.008 m
TCP duration:   0.11 s
Wrist step:     0.7 deg
Wrist duration: 0.11 s
Joint step:     0.4 deg
Hold period:    0.07 s
```

可以在 `Configure` 页面临时调节：

- `TCP step m`
- `Wrist step deg`
- `Joint step deg`
- `Hold period s`
- `Waypoint duration s`

如果现场觉得仍然慢，优先小幅增加 `TCP step m` 或 `Wrist step deg`。如果真机出现抖动、急促或保护停止，先降低这些值。

### 本地配置保存和默认加载

施教器默认会在启动时加载：

```text
recordings/teach/teach_panel_config.json
```

配置文件会保存：

- 底盘和机械臂点动速度/步长。
- Home / Install 预设位姿。
- 后置架安全区参数。

保存当前配置：

1. 在 `Configure` 页面确认参数。
2. 点击 `Apply`。
3. 点击 `Save Config`。

加载指定配置：

1. 在 `Configure` 页面点击 `Browse` 选择 JSON。
2. 点击 `Load Config`。

如果要启动时加载其它配置文件：

```bash
./scripts/hardware/real_grasp_console.sh --yes --quick -- \
  teach_config_path:=recordings/teach/my_teach_config.json
```

## 6. 后置架安全区

当前施教器默认启用后置架防碰撞安全区，避免机械臂、夹具、相机或充电枪扫到后置传感器架。

默认安全区在 `base_link` 坐标系下：

```text
rear_rack_keepout_min_xyz = -0.41,-0.22,0.04
rear_rack_keepout_max_xyz =  0.09, 0.22,0.82
```

安全区会检查：

- X/Y/Z 点动后的 IK 目标。
- RX/RY/RZ 姿态点动目标。
- 单关节点动目标。
- Home / Install 和任意关节目标。
- TCP 目标。
- 回放 waypoint。

如果目标会进入安全区，施教器日志会出现类似：

```text
arm motion blocked by safety zone: ... enters rear rack keepout ...
```

这时不要强行继续，先在 RViz 中确认姿态，再换方向运动或调整目标。

如果后置架实际安装位置和模型有偏差，可以启动时微调安全区：

```bash
./scripts/hardware/real_grasp_console.sh --yes --quick -- \
  rear_rack_keepout_min_xyz:="-0.43,-0.24,0.03" \
  rear_rack_keepout_max_xyz:="0.11,0.24,0.84"
```

只有在不装后置架或离线调试时，才建议关闭安全区：

```bash
./scripts/operator/teach_panel.sh arm_keepout_enabled:=false
```

真机施教时不建议关闭。

## 7. Aubo 负载设置

当前脚本默认按实测末端设置：

```text
mass = 0.818 kg
cog  = 0.039927,0.045067,0.143233 m
```

这些值同时记录在：

- `src/arachne_description/config/real_hardware_defaults.yaml`
- `scripts/env/arachne_real_defaults.sh`
- `recordings/teach/teach_panel_config.json`

日常入口 `real_grasp_console.sh` 会读取共享 defaults；历史示教/验收脚本 `real_full_teach.sh`、`real_full_acceptance.sh` 也会读取同一份配置。当启用自动 Aubo startup 或运行验收流程时，会按这里的 payload 写入控制器。若更换充电枪、相机或夹具，需要调整：

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
ARACHNE_CONFIRM_AUBO_PAYLOAD=YES python3 scripts/hardware/real_aubo_payload.py \
  --mass 0.818 \
  --cog 0.039927,0.045067,0.143233
```

也可以通过环境变量覆盖一键脚本默认值：

```bash
ARACHNE_AUBO_PAYLOAD_MASS=0.818 \
ARACHNE_AUBO_PAYLOAD_COG=0.039927,0.045067,0.143233 \
ARACHNE_CONSOLE_AUTO_AUBO_START=true \
./scripts/hardware/real_grasp_console.sh --yes --quick
```

说明：

- `--mass` 是末端总负载，单位 kg。
- `--cog` 是负载重心相对工具法兰中心的偏移，单位 m。
- 如果充电枪主要沿工具前方伸出，优先调第三个值。
- 如果负载明显左右偏心，再调前两个值。

## 8. 一键完整联合验收

真机验收命令：

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
./scripts/hardware/real_full_acceptance.sh --yes
```

这个脚本会自动完成：

1. 检查 ROS、串口、Aubo 网络和 vendor 链接。
2. 启动 Aubo ROS2 driver。
3. 远程执行 Aubo `poweron` 和 `startup`。
4. 写入 Aubo payload。
5. 验证 Aubo hold 控制。
6. 启动 Scout 底盘和 MS42DC 夹具驱动。
7. 执行联合验收测试。
8. 测试结束后自动停止后台 bringup 进程。

联合测试动作包括：

- 底盘前进约 `0.2 m`，再后退约 `0.2 m`。
- 底盘左转约 `30 deg` 并回正。
- 底盘右转约 `30 deg` 并回正。
- Aubo 机械臂执行小幅 tool-y circle 轨迹。
- MS42DC 夹具开合 5 次，最终保持打开。

看到下面这行表示测试完成：

```text
acceptance test complete
```

如果测试完成后想继续保持 ROS driver 运行：

```bash
./scripts/hardware/real_full_acceptance.sh --yes --keep-running
```

## 9. 分步运行方式

如果一键脚本失败，可以分终端运行。

终端 1：Aubo driver

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
ARACHNE_CONFIRM_AUBO_DRIVER=YES ARACHNE_AUBO_ALLOW_PRESTART=YES ./scripts/hardware/real_aubo_bringup.sh
```

终端 2：Aubo 远程上电和 startup

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
ARACHNE_CONFIRM_AUBO_REMOTE_START=YES ./scripts/hardware/real_aubo_remote_start.sh
```

看到下面输出后继续：

```text
Aubo remote startup complete
```

终端 3：底盘和夹具 bringup

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
./scripts/hardware/real_bringup.sh --no-aubo
```

终端 4：施教器

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
./scripts/operator/teach_panel.sh
```

或者联合验收：

```bash
ARACHNE_CONFIRM_REAL_MOTION=YES ./scripts/hardware/real_hardware_acceptance_test.sh
```

测试结束后，在终端 1 和终端 3 按 `Ctrl+C` 停止 bringup。

## 10. 日志位置

施教器总控日志：

```text
/home/jetson/zhaoyang/Arachne/log/real_grasp_console/latest/
/home/jetson/zhaoyang/Arachne/log/teach_panel/latest/
```

一键验收日志：

```text
/home/jetson/zhaoyang/Arachne/log/real_full_acceptance/YYYYMMDD_HHMMSS/
```

常见日志：

```text
01_check_real_hardware_env.log
02_aubo_driver.log
03_aubo_remote_start.log
04_aubo_prepare.log
05_base_gripper_bringup.log
06_real_hardware_acceptance_test.log
07_teach_panel.log
```

如果失败，先看终端最后输出，再看对应日志。

## 11. 常见问题

### 需要立即停止

最高优先级是按下急停或切断硬件电源。

如果只是停止 ROS 脚本，在运行脚本的终端按：

```text
Ctrl+C
```

也可以清理旧栈：

```bash
cd /home/jetson/zhaoyang/Arachne
./scripts/hardware/stop_real_stack.sh
```

### Aubo 处于 PowerOff

一键脚本可以处理。它会先启动 Aubo driver 的 prestart 模式，然后调用：

```text
RobotManage.poweron
RobotManage.startup
```

正常情况下会进入：

```text
mode: Running
safety: Normal
```

### 找不到串口

先看设备：

```bash
ls -l /dev/serial/by-id /dev/ttyUSB* /dev/ttyACM*
```

再运行严格检查：

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
./scripts/hardware/check_real_hardware_env.sh --strict
```

### 没有 joint_states

确认 Aubo driver 已启动，并且控制器 active：

```bash
ros2 control list_controllers
ros2 topic list | grep joint_states
```

如果刚刚改过代码，重新构建：

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
./scripts/build/build_workspace.sh
```

### RViz 中模型卡顿或两个状态源打架

检查 `/joint_states` 是否有多个发布者：

```bash
ros2 topic info -v /joint_states
```

如果看到旧的 bringup 或旧的 robot_state_publisher，先清理：

```bash
./scripts/hardware/stop_real_stack.sh
```

然后重新启动：

```bash
./scripts/hardware/real_grasp_console.sh --yes --quick
```

### XYZ 运动时姿态不应变化

当前版本的 X/Y/Z 点动会用 pose IK 保持当前 RX/RY/RZ。若仍看到姿态变化，先确认使用的是最新构建：

```bash
git log --oneline -3
./scripts/build/build_selected.sh arachne_operator
source install/setup.bash
```

### 安全区误触发

如果日志显示 `arm motion blocked by safety zone`，先不要关闭安全区。按顺序检查：

1. RViz 中机械臂是否真的靠近后置架。
2. 后置架实际安装位置是否和模型一致。
3. `rear_rack_keepout_min_xyz` / `rear_rack_keepout_max_xyz` 是否需要小幅调整。

真机上只有确认后置架不存在或安全区明显错误时，才临时关闭 `arm_keepout_enabled`。
