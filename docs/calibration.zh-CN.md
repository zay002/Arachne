# 标定

第一项标定目标是 Scout 到 Aubo 的固定变换：

```text
top_plate_link -> arm_mount_link -> aubo_base_link
```

当前默认值是近似值：

```text
arm_mount_xyz = 0.22 0.0 0.155
arm_mount_rpy = 0.0 0.0 1.57079632679
```

真实安装板测量完成后，应更新 launch 参数或 `config/physical_parameters.yaml`。

## MS42DC 夹指行程

易爪机器人 MS42DC 二指柔性伺服电机夹爪模型使用 `third_party/MS42DC_SPLIT` 中由项目作者手动拆分的零件。当前仿真设置为：

```text
ms42dc_left_finger_joint:  0.0 到 1.0 rad，轴 0 0 -1
ms42dc_right_finger_joint: -1.0 到 0.0 rad，轴 0 0 -1，mimic 左指，multiplier -1.0
默认仿真闭合目标: 0.6 rad
```

经过 RViz 检查，铰链轴确认为 CAD Z 轴，视觉闭合方向使用 `0 0 -1` 修正，`0.6 rad` 作为当前默认闭合角。之后如果真实夹爪或 CAD 拆分发生变化，可以用 `gripper_closed_position:=...` 或 `GRIPPER_CLOSED_POSITION=... ./scripts/view_model.sh` 重新调整闭合角。
