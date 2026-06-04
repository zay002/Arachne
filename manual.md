# Arachne Jetson 操作手册

更新日期：2026-06-04

本手册适用于当前 Jetson Orin Nano 上的 Arachne `jetson` 分支。

- 工作目录：`/home/jetson/zhaoyang/Arachne`
- 系统：Ubuntu 22.04
- ROS 2：Humble
- 机器人：Scout 2.0 底盘 + Aubo i5 机械臂 + MS42DC 夹具
- 当前模型：底盘、Aubo i5、夹具、后置传感器架、C16 雷达、末端 Orbbec 相机、车头吊篮
- 当前重点：真机施教、联合验收、RViz 可视化、后置架防碰撞安全区、Gemini335/C16 感知、静态垃圾拾取与充电枪拔插任务准备

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

日常直接使用分类路径，例如 `source scripts/env/arachne_env.sh`、`./scripts/hardware/real_full_teach.sh --yes`、`./scripts/vision/gemini_yolo_live.sh`。新增脚本也应放进对应子目录，并同步更新 README 和 manual。

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

下载当前推荐权重：

```bash
./scripts/vision/download_yolo_weights.sh
```

当前默认使用：

```text
yolo26n.pt      第一版快速通用检测，用于垃圾拾取 MVP
yolo26n-seg.pt  后续 mask + depth ROI 的抓取定位候选
```

第一版先用 YOLO26n 跑通检测和 3D 定位；如果 TensorRT 后帧率还有余量，再尝试 YOLO26s 或 YOLO26n-seg。充电枪不是通用 COCO 类别，后续需要采集本机 Gemini335 下的充电枪样本并微调专用权重。INT8 导出前需要先采集本机 Gemini335 的代表性图片作为 calibration/validation 数据，不建议用通用样例数据做最终量化。

FP16 TensorRT 测试导出：

```bash
./scripts/vision/export_yolo_engine.sh yolo26n.pt fp16
```

INT8 导出需要先准备 `yolo_workspace/datasets/trash_mvp/images/val`：

```bash
./scripts/vision/export_yolo_engine.sh yolo26n.pt int8
```

只用 Gemini335 做实时 YOLO 标注预览，不启动底盘和机械臂：

```bash
./scripts/vision/gemini_yolo_live.sh
```

默认实时预览使用 `YOLO26n`、`imgsz=640`、`conf=0.25`，并只显示 `bottle,cup,bowl` 三类，避免 COCO 预训练模型把人、桌子、遥控器等低置信度误检都画出来。关闭检测窗口、按 `Esc` 或按 `q` 都会退出检测进程。

如果需要临时调整类别：

```bash
ARACHNE_YOLO_CLASSES=bottle,cup ./scripts/vision/gemini_yolo_live.sh
```

如果需要看所有 COCO 类：

```bash
ARACHNE_YOLO_CLASSES= ./scripts/vision/gemini_yolo_live.sh --classes ""
```

停止实时预览：

```bash
./scripts/vision/stop_gemini_yolo_live.sh
```

实时日志：

```bash
tail -f yolo_workspace/runs/gemini_yolo_live/latest.log
```

用 mock 硬件打开施教器和 RViz，不会控制真机：

终端 1：

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
ros2 launch arachne_hardware mock_bringup.launch.py
```

终端 2：

```bash
cd /home/jetson/zhaoyang/Arachne
source scripts/env/arachne_env.sh
./scripts/operator/teach_panel.sh
```

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
./scripts/hardware/real_full_teach.sh --yes
```

这个脚本会自动完成：

1. 检查 ROS、串口、Aubo 网络和 vendor 链接。
2. 启动 Aubo ROS2 driver。
3. 远程执行 Aubo `poweron` 和 `startup`。
4. 写入 Aubo 负载参数。
5. 验证 Aubo hold 控制。
6. 启动 Scout 底盘和 MS42DC 夹具 bringup。
7. 打开施教器，并同时启动 RViz 可视化。
8. 施教器退出后自动停止后台 bringup 进程。

如果要保留 bringup 进程：

```bash
./scripts/hardware/real_full_teach.sh --yes --keep-running
```

如果要把录制文件放到指定目录：

```bash
./scripts/hardware/real_full_teach.sh --yes -- recording_dir:=recordings/teach_demo
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
- 预设配置：`Configure` 页面可设置 Home / Install 位姿，并保存/加载本地配置。
- 录制回放：`Program` 页面录制 waypoint、wait、保存、加载、回放。
- 窗口缩放：各页面支持滚动，小窗口或远程桌面下不会裁掉底部控件。

默认 Home 和 Install 位姿：

```text
J1=-88.28, J2=3.40, J3=116.60, J4=103.48, J5=88.33, J6=-0.13 deg
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
./scripts/hardware/real_full_teach.sh --yes -- \
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
./scripts/hardware/real_full_teach.sh --yes -- \
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

`real_full_teach.sh` 和 `real_full_acceptance.sh` 会在启动流程中写入该 payload。若更换充电枪、相机或夹具，需要调整：

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
./scripts/hardware/real_full_teach.sh --yes
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

一键施教日志：

```text
/home/jetson/zhaoyang/Arachne/log/real_full_teach/YYYYMMDD_HHMMSS/
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
./scripts/hardware/real_full_teach.sh --yes
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
