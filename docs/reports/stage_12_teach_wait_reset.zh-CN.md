# Stage 12：示教等待与重置

## 目标

补全真机示教面板的两个演示常用动作：在回放序列中插入等待 N 秒，以及一键重置示教列表和编号。

## 核心文件

- `src/arachne_operator/arachne_operator/teach_panel.py`：`TeachWaypoint` 增加 `kind` 与 `wait_sec` 字段；UI 增加 `Add Wait` 和 `Reset`；回放遇到 wait 步骤时只等待，不发运动命令。
- `docs/control.zh-CN.md` / `docs/control.md`：记录 wait、clear 和 reset 的行为。

## 文件关系

保存的 JSON 仍然使用同一组 waypoint 列表。旧文件没有 `kind` 时会按普通位姿 waypoint 加载；新 wait waypoint 通过 `kind=wait` 与 `wait_sec` 表示，能和普通机器人状态 waypoint 混排。
