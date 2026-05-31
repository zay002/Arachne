# Stage 17：示教机械臂速度微调

## 目标

在继续使用安全 trajectory controller 路径的前提下，将 Aubo 示教面板中的机械臂运动速度提高约 20%。

## 核心文件

- `src/arachne_operator/arachne_operator/teach_panel.py`：Aubo jog 时长从 `1.0 s` 降到 `0.83 s`，回放 waypoint 时长从 `4.5 s` 降到 `3.75 s`。
- `src/arachne_operator/launch/teach_panel.launch.py`：同步 launch 默认参数。
- `docs/control.md` / `docs/control.zh-CN.md`：更新机械臂回放时长说明。

## 文件关系

这次只调整轨迹时间参数。Waypoint 记录、Teach On/Off 处理、反馈校验以及底盘/夹具逻辑保持不变。
