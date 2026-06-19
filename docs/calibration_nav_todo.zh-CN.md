# 标定与导航 TODO

本文只整合待办，不实现算法。

| 项目 | 当前状态 | 缺失内容 | 建议输出文件 |
| --- | --- | --- | --- |
| `top_plate_link -> aubo_base_link` | URDF 中已有标称安装参数，真实安装偏差未系统求解。 | 实测安装位姿、重复测量误差、版本化标定记录。 | `config/calibration/aubo_base_extrinsics.yaml` |
| `tool0 -> ee_camera_link` | URDF 中已有末端相机标称位姿，抓取预览可用 offset 修正。 | 手眼标定结果、温漂/重装后的校验流程。 | `config/calibration/ee_camera_hand_eye.yaml` |
| `base_link -> lidar_link` | 模型中有 C16/lidar_link 标称位姿。 | 实测外参、与车体中心/雷达点云地面对齐校验。 | `config/calibration/lidar_extrinsics.yaml` |
| Gemini335 内参/深度对齐 | `gemini335_v4l2_node` 支持参数化内参、深度比例和点云生成。 | 标定内参、depth/color 对齐模型、有效深度范围校验。 | `config/calibration/gemini335_intrinsics.yaml` |
| AprilTag 手眼标定 | 已有 `apriltag_hand_eye_calibrator` 和 shell 入口。 | 标定板固定流程、样本采集规范、solve 结果落盘约定。 | `config/calibration/apriltag_hand_eye.yaml` |
| `map -> odom -> base_link` | Nav2/SLAM 负责 `map -> odom`，底盘里程计负责 `odom -> base_link`。 | 定位来源切换策略、AprilTag 初始化与 SLAM 接管的状态机。 | `src/arachne_nav/config/localization_sources.yaml` |
| SLAM/Nav2 接管 mock `map->odom` | `prehardware_control.launch.py` 可用 `with_mock_map_odom:=false` 关闭 mock 变换。 | 何时关闭 mock、如何验证定位发布稳定、失败回退策略。 | `docs/nav_handover_checklist.zh-CN.md` |

## 建议流程

1. 固定机器人机械结构后，先确认 URDF 标称 TF 与实物方向一致。
2. 标定 Gemini335 内参和深度比例。
3. 用 AprilTag 做 `tool0 -> ee_camera_link` 手眼标定。
4. 标定 C16 `base_link -> lidar_link`。
5. 建立 `map -> odom` 来源切换规则：mock、AprilTag 初始化、SLAM/Nav2。
6. 将所有标定输出纳入 `config/calibration/`，由 launch 参数显式加载。

## 注意

仿真和真机都应使用同一套 frame 名称。任何临时 offset 只能作为调试参数，不能替代长期标定文件。
