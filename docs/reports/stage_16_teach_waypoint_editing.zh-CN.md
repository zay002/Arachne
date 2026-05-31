# Stage 16：示教 Waypoint 单点编辑

## 目标

让真机演示录制后更容易局部修正：可以单独覆盖某个 waypoint 的状态，并在保持保守的前提下略微提高机械臂回放速度。

## 核心文件

- `src/arachne_operator/arachne_operator/teach_panel.py`：新增 `Update WP`，可用当前机器人状态覆盖一个选中的姿态 waypoint；如果选中的是 wait 步骤，则用 `Wait s` 更新等待时间。
- `src/arachne_operator/launch/teach_panel.launch.py`：同步暴露更快的默认启动参数。
- `docs/control.md` / `docs/control.zh-CN.md`：记录单点编辑入口和更新后的回放速度。

## 文件关系

示教面板仍然是唯一面向操作者的编辑入口。Record 继续追加新点，Duplicate 继续复用已有点，Update WP 则负责在不重建整条序列的情况下修改一个选中的点。
