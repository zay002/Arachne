# Stage 11：安全示教回放默认值

## 目标

让示教例程稳定保存在项目本地，并让一键回放默认足够慢，适合真机演示前的保守操作。

## 核心文件

- `src/arachne_operator/arachne_operator/teach_panel.py`：默认本地录制目录改为 `recordings/teach`，底盘回放保持保守速度，机械臂每个 waypoint 默认 `6.0 s`。
- `src/arachne_operator/launch/teach_panel.launch.py`：同步 launch 默认参数。
- `scripts/teach_panel.sh` / `scripts/real_teach_demo.sh`：启动时传入仓库内绝对录制目录 `${ROOT_DIR}/recordings/teach`，避免从不同工作目录启动时记录文件分散。

## 文件关系

示教 UI 仍然只通过 `/cmd_vel`、Aubo trajectory action 和 `/arachne/gripper/command` 发送命令。脚本负责固定本地保存路径；UI 负责保存、加载和慢速回放 JSON waypoint。
