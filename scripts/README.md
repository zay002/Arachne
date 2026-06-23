# Arachne Scripts

Shell scripts are thin entrypoints. Shared checks live in the `arachne` CLI:

```bash
ros2 run arachne_operator arachne --help
```

## 主入口

```bash
./scripts/model/view_model.sh
./scripts/hardware/real_bringup.sh
./scripts/operator/teach_panel.sh
./scripts/vision/grasp_task_server.sh
./scripts/vision/road_cleanup_task_server.sh
./scripts/build/check_offline_regression.sh
```

## 开发检查

```bash
./scripts/build/check_offline_regression.sh
./scripts/build/check_aubo_action_stack.sh
./scripts/test/smoke_aubo_move_joint_dry_run.sh
./scripts/test/smoke_demo_orchestrator_offline.sh
```

## 真机 Bringup

```bash
AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_readonly.sh
AUBO_ROBOT_IP=192.168.127.128 ./scripts/hardware/check_aubo_running_readonly.sh
./scripts/hardware/real_bringup.sh
./scripts/hardware/stop_real_stack.sh
```

## 示教器

```bash
./scripts/operator/teach_panel.sh
./scripts/hardware/real_teach_demo.sh
./scripts/operator/start_real_teach_with_bringup.sh
```

桌面启动（可选）：

```bash
chmod +x scripts/operator/start_real_teach_with_bringup.sh
cp scripts/operator/arachne-real-teach.desktop ~/Desktop/
```

双击桌面图标即可启动：环境加载 + real_bringup + 示教器。

## 视觉任务

```bash
./scripts/vision/gemini335_bringup.sh
./scripts/vision/gemini_yolo_live.sh
./scripts/vision/grasp_preview.sh
./scripts/vision/grasp_preview_real_sync.sh --sync-only
./scripts/vision/grasp_task_server.sh
./scripts/vision/road_cleanup_task_server.sh
```

## 导航/标定

```bash
./scripts/hardware/real_lidar_nav.sh
./scripts/hardware/real_lidar_save_map.sh
./scripts/vision/apriltag_nav_initialize.sh
./scripts/vision/apriltag_nav_start_mapping.sh
./scripts/vision/apriltag_hand_eye_calibration.sh
```

## 兼容 Wrapper

These stay for old commands or low-frequency checks:

- `scripts/hardware/real_grasp_console.sh`
- `scripts/hardware/real_grasp_console_remote.sh`
- `scripts/model/view_sensor_model.sh`
- `scripts/test/*`

## 废弃/归档脚本

Do not add new phase-only shell scripts. Prefer:

```bash
ros2 run arachne_operator arachne check offline
ros2 run arachne_operator arachne check aubo-readonly
ros2 run arachne_operator arachne smoke aubo-dry-run
ros2 run arachne_operator arachne smoke demo-orchestrator
```

真实运动仍必须由人工确认安全后通过对应真机入口执行。
