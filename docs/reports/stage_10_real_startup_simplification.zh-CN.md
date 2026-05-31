# Stage 10：真机启动简化

## 目标

把日常真机演示从多终端、多参数 launch 简化成稳定入口，减少 WSL2 重启和 USB 重新透传后的重复操作。

## 核心文件

- `scripts/real_bringup.sh`：自动加载 ROS 环境，探测 Scout 和 MS42DC 串口，检查 Aubo 状态，并启动 `arachne_hardware real_bringup.launch.py`。
- `scripts/real_teach_demo.sh`：在 `real_bringup.sh` 之上等待 `/odom`、`/joint_states`、Aubo action 和夹具状态，然后打开示教回放面板；面板关闭后自动停止 bringup。
- `scripts/check_real_hardware_env.sh`：复用 MS42DC 自动候选探测，避免只因为没有 `/dev/motor_serial` 别名而误报。
- `README.md` / `docs/hardware.zh-CN.md`：把日常真机启动改为一键入口，同时保留环境变量覆盖方式。

## 文件关系

底层 `real_bringup.launch.py` 仍然保留完整参数，适合调试；新脚本只负责把实验室常用配置固化成默认流程，并在缺少串口时提示使用 `hurry-porter` 重新透传 USB。
