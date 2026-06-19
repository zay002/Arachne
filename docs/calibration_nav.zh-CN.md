# 标定与导航

常用入口：

```bash
./scripts/vision/apriltag_nav_initialize.sh
./scripts/vision/apriltag_nav_start_mapping.sh
./scripts/vision/apriltag_hand_eye_calibration.sh
./scripts/hardware/real_lidar_nav.sh
./scripts/hardware/real_lidar_save_map.sh
```

坐标原则：

- `base_link`：底盘坐标，前方 +X，左侧 +Y，上方 +Z。
- `aubo_base_link`：Aubo 底座。
- `tool0`：法兰中心，不等于相机。
- Gemini335 点云必须通过实际相机 TF 转到 `base_link` 后再参与抓取。

地面高度、手眼外参和抓取半径都属于真机标定量，不要硬编码到任务逻辑里。
