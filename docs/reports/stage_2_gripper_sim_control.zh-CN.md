# Stage 2 报告：夹爪仿真控制

## 结果

MS42DC 和 AG95 现在可以通过同一套 demo 接口在 RViz 仿真中开闭；两套模型唯一差异是所选夹爪。MS42DC 使用项目作者手动拆分的 CAD 零件，左指为旋转关节，右指反向 mimic。AG95 驱动 vendor 模型中的 knuckle/finger 关节。小型 `Arachne Gripper` GUI 提供 Open/Close 按钮，用于可视化 demo。

## 核心文件

- `src/arachne_gripper/`：夹爪仿真工具 ROS2 Python 包。
- `arachne_gripper/gripper_sim_controller.py`：为 `ms42dc` 和 `ag95` 发布仿真 joint state。
- `arachne_gripper/gripper_state_gui.py`：双按钮 GUI，调用夹爪 open/close 服务。
- `arachne_gripper/joint_state_mux.py`：合并 GUI/default joint state 与夹爪状态，并发布单一 `/joint_states`。
- `urdf/gripper/ms42dc.urdf.xacro`：加载项目作者手动拆分的 MS42DC mesh，并定义左右旋转夹指关节。
- `launch/display.launch.py`：当 `with_gripper_sim:=true` 时，将夹爪仿真 joint state 合入统一 `/joint_states`。
- `scripts/test_gripper_sim.sh`：MS42DC 和 AG95 仿真开闭 smoke test。

## 接口

- 用户侧服务：`/arachne/gripper/open`、`/arachne/gripper/close`
- 状态话题：`/arachne/gripper/joint_states`、`/arachne/default_joint_states`、`/arachne/gui_joint_states`、`/joint_states`

MS42DC mesh 是真实的、由项目作者手动拆分的 CAD 零件。铰链方向现在为 `0 0 -1`，默认闭合角为 `0.6 rad`；后续重新调参可通过 `gripper_closed_position` 或 `GRIPPER_CLOSED_POSITION` 覆盖。
