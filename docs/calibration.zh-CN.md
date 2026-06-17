# 标定

第一项标定目标是 Scout 到 Aubo 的固定变换：

```text
top_plate_link -> aubo_base_link
```

当前默认值是近似值：

```text
arm_mount_xyz = 0.22 0.0 0.105
arm_mount_rpy = 0.0 0.0 1.57079632679
```

真实安装板测量完成后，应更新 launch 参数或 `config/physical_parameters.yaml`。

## 地面 AprilTag 手眼标定板

项目内提供了一张适合贴在地面上的 A3 横向 AprilTag 标定板：

```text
assets/calibration/arachne_floor_apriltag_board_a3.pdf
```

默认参数：

```text
family: tagStandard41h12
tag ids: 0 到 11
页面: A3 横向，420 x 297 mm
tag 边长: 70 mm
tag 中心间距: 100 mm
板坐标系: 原点在页面中心，+X 向右，+Y 向上，+Z 垂直离开纸面
```

配套文件：

```text
assets/calibration/arachne_floor_apriltag_board_a3.svg
assets/calibration/arachne_floor_apriltag_board_a3.png
assets/calibration/arachne_floor_apriltag_board_a3.yaml
```

打印时使用 PDF，选择“实际大小 / 100% / 不缩放”。打印后用尺量左上角 `100 mm check scale`，确认它确实是 100 mm；如果打印机强制缩放，后续外参会带入比例误差。贴地时尽量覆膜或贴在平整硬板上，避免纸张起皱；把 `yaml` 里的 `tag_size_m: 0.070000` 作为 AprilTag 位姿估计的 tag size。

如需重新生成或改尺寸：

```bash
scripts/calibration/generate_apriltag_floor_board.py
scripts/calibration/generate_apriltag_floor_board.py --tag-size-mm 60 --pitch-mm 90 --output-prefix assets/calibration/custom_floor_board
```

## MS42DC 夹指行程

易爪机器人 MS42DC 二指柔性伺服电机夹爪模型使用 `third_party/MS42DC_SPLIT` 中由项目作者手动拆分的零件。当前仿真设置为：

```text
ms42dc_left_finger_joint:  0.0 到 1.0 rad，轴 0 0 -1
ms42dc_right_finger_joint: -1.0 到 0.0 rad，轴 0 0 -1，mimic 左指，multiplier -1.0
默认仿真闭合目标: 0.6 rad
```

经过 RViz 检查，铰链轴确认为 CAD Z 轴，视觉闭合方向使用 `0 0 -1` 修正，`0.6 rad` 作为当前默认闭合角。之后如果真实夹爪或 CAD 拆分发生变化，可以用 `gripper_closed_position:=...` 或 `GRIPPER_CLOSED_POSITION=... ./scripts/model/view_model.sh` 重新调整闭合角。
