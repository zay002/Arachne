# Stage 9：真机示教与回放面板

## 目标

为演示和小型实验提供一个可视化示教入口：手动控制 Scout 底盘、Aubo 末端和 MS42DC 夹具，手动记录当前状态，并一键按记录顺序回放。

## 核心文件

- `src/arachne_operator/arachne_operator/teach_panel.py`：Tk UI 和 ROS2 节点。订阅 `/odom`、`/joint_states` 和硬件状态，发布 `/cmd_vel`、Aubo trajectory/action 和 `/arachne/gripper/command`。
- `src/arachne_operator/launch/teach_panel.launch.py`：示教面板 launch，暴露底盘速度、Aubo 关节名、action 名称、jog 步长和记录目录参数。
- `scripts/teach_panel.sh`：仓库级启动脚本，先加载 `scripts/arachne_env.sh`，再启动 launch。
- `docs/control.zh-CN.md` / `docs/control.md`：记录示教面板的控制契约、启动方法和 waypoint 文件格式位置。

## 文件关系

`teach_panel.py` 与已有真机 bringup 共享同一组 ROS 契约：底盘使用 `/cmd_vel` 和 `/odom`，机械臂使用 `/joint_states` 与 `FollowJointTrajectory`，夹具使用 `/arachne/gripper/command`。UI 保存为 JSON；机械臂回放基于关节角，底盘回放基于相对移动段，因此同一套记录可以用于演示复现和后续调参。

## 当前边界

底盘相对移动仍依赖 `/odom` 闭环估计距离和角度，但不再保存历史绝对 base 坐标。Aubo 末端 jog 使用本地位置 IK，不做完整避障；正式自动任务仍应交给 MoveIt2/Nav2。
