# Stage 14：相对式示教回放

## 目标

修复 Scout 移动后示教回放失效的问题，让底盘记录为可复现的相对动作，并确保机械臂回放仍基于关节角而不是笛卡尔坐标。

## 核心文件

- `src/arachne_operator/arachne_operator/teach_panel.py`：每次长按底盘按钮并松开时，自动生成一个包含前进/后退距离或左/右转角的 waypoint；回放时不再使用历史绝对 `base_pose`，而是用当前 `/odom` 闭环执行相对移动。
- `docs/control.md` / `docs/control.zh-CN.md`：记录 v2 示教文件的语义。

## 文件关系

示教面板仍然只使用已有 ROS 接口：`/cmd_vel`、`/odom`、`/joint_states`、Aubo trajectory action/topic 和 `/arachne/gripper/command`。本阶段只改变记录层：底盘变成动作段，机械臂保持关节空间 waypoint。

