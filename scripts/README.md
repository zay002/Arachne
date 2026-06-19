# Arachne Scripts

Scripts are organized by function. Use the categorized paths directly. The
Chinese repository audit with ROS executable/topic/service details lives in
[`docs/audit_repo_structure.zh-CN.md`](../docs/audit_repo_structure.zh-CN.md).

```bash
source scripts/env/arachne_env.sh
./scripts/build/build_workspace.sh
./scripts/build/check_offline_regression.sh
./scripts/test/smoke_aubo_move_joint_dry_run.sh
./scripts/test/smoke_demo_orchestrator_offline.sh
./scripts/operator/teach_panel.sh
./scripts/vision/gemini_yolo_live.sh
./scripts/vision/grasp_preview.sh
./scripts/vision/grasp_preview_real_sync.sh --sync-only
./scripts/vision/grasp_task_server.sh
./scripts/vision/road_cleanup_task_server.sh
python3 scripts/vision/mock_road_cleanup_task_test.py
./scripts/agent/agent_bridge.sh
```

| Directory | Purpose |
| --- | --- |
| `env/` | ROS and workspace environment setup |
| `build/` | Colcon build, setup, and workspace checks |
| `hardware/` | Real hardware bringup, Aubo helpers, acceptance tests, serial checks |
| `operator/` | Teach-panel launch entry |
| `agent/` | Safe Agent Bridge launch entry |
| `vision/` | Gemini335, YOLO/TACO segmentation, TensorRT export, live detection, grasp preview, grasp task server, road cleanup task server |
| `model/` | URDF, TF, gripper, and RViz model checks |
| `sim/` | Gazebo demos and simulation validation |
| `test/` | Offline ROS smoke tests that do not require real hardware |
| `godot/` | Godot showcase setup and bridge helpers |
| `remote/` | Remote MoveIt/planner experiments and deployment helpers |
| `calibration/` | Calibration asset generation |

When adding a new script, put it in the matching subdirectory and update README/manual command examples to use that categorized path.

Aubo Phase 2 note: script entrypoints and real-machine behavior are unchanged.
Shared JSON-RPC, `control_owner`, `teach_gate`, `speedJoint`, `stopJoint`, and
guarded `moveJoint` helpers now live under
`src/arachne_hardware/arachne_hardware/aubo_sdk/`.

Aubo Phase 3A note: `real_bringup.sh` starts
`aubo_move_joint_action_server` when `use_aubo:=true`, exposing guarded
SDK `moveJoint` as `/arachne/aubo/move_joint`. Script entrypoints are unchanged,
and teach panel replay falls back to the internal SDK helper if the action
server is unavailable.

Demo Phase 3B note: `teach_panel.launch.py` can start
`demo_orchestrator` with `with_demo_orchestrator:=true`, exposing
`/arachne/demo/*` Trigger services for camera, visual grasp, road cleanup,
preflight, status, and stop. Existing script entrypoints are unchanged.

Aubo Phase 3C note: grasp task real `sdk_move_joint` execution now prefers
`/arachne/aubo/move_joint` and keeps the previous guarded SDK path as fallback.
Existing script entrypoints and confirmation variables are unchanged.

Aubo Phase 4A note: `scripts/build/check_aubo_action_stack.sh` checks the
Aubo action interface, installed executables, key files, and Python compile
status without sending any motion goal. `aubo_move_joint_action_server` also
supports `dry_run:=true` for ROS action graph validation only; default remains
`false`, and fallback paths remain enabled.

Aubo Phase 4B note: `scripts/hardware/check_aubo_readonly.sh` prepares the
real-hardware read-only check flow. It checks network, TCP 30004, read-only
RobotState JSON-RPC, ROS interfaces, and graph presence only. It does not send
goals, enter teach mode, run speedJoint/moveJoint, or remove fallback paths.

Aubo Phase 4C-1 note: `scripts/hardware/check_aubo_running_readonly.sh` checks
whether Aubo is Running/Normal and whether `/joint_states`,
`/arachne/hardware/aubo_status`, and `/arachne/aubo/move_joint` are observable.
It only reports state; it does not start Aubo, send goals, write gate/owner
files, or validate current-state hold.

Phase 5A note: `scripts/build/check_offline_regression.sh`,
`scripts/test/smoke_aubo_move_joint_dry_run.sh`, and
`scripts/test/smoke_demo_orchestrator_offline.sh` provide the no-hardware
regression path. The action smoke sends only a dry-run goal to a server started
with `dry_run:=true`; the orchestrator smoke calls only status/preflight.

## Full Index

Status values: `primary`, `helper`, `deprecated`, `experimental`, `unknown`.
Profiles: `mock`, `sim`, `real-dry-run`, `real-execute`, `mixed`.

| Path | Status | Profile | Hardware | Safe by default | Main purpose | Deprecated replacement / confirmation |
| --- | --- | --- | --- | --- | --- | --- |
| `agent/agent_bridge.sh` | primary | mixed | scout,aubo,ms42dc | yes | Launch safe Agent Bridge with motion disabled unless explicitly enabled. | Motion requires launch args such as `motion_enabled:=true`. |
| `build/build_selected.sh` | helper | mock | none | yes | Build selected packages. | n/a |
| `build/build_workspace.sh` | helper | mock | none | yes | Build the workspace. | n/a |
| `build/check_aubo_action_stack.sh` | helper | mock | none | yes | Check AuboMoveJoint action interface, executables, key files, and compile status without sending motion goals. | n/a |
| `build/check_offline_regression.sh` | primary | mock | none | yes | Run offline static/workspace/build regression without connecting hardware or sending motion goals. | n/a |
| `build/check_workspace.sh` | helper | mock | none | yes | Static/light workspace checks. | n/a |
| `build/setup_jetson_humble.sh` | helper | mock | none | yes | Install Jetson/Humble dependencies. | n/a |
| `build/setup_ubuntu.sh` | helper | mock | none | yes | Install Ubuntu/ROS dependencies. | n/a |
| `calibration/generate_apriltag_floor_board.py` | experimental | mock | none | yes | Generate AprilTag floor-board assets. | n/a |
| `env/arachne_env.sh` | helper | mixed | none | yes | Load ROS/workspace Python environment. | n/a |
| `env/arachne_real_defaults.sh` | helper | real-dry-run | scout,aubo,ms42dc,gemini335,c16 | yes | Shared real-machine defaults. | n/a |
| `env/load_local_env.sh` | helper | mixed | none | yes | Load local `.env` overrides. | n/a |
| `env/ros_env.sh` | helper | mixed | none | yes | ROS distro and setup helpers. | n/a |
| `godot/fetch_godot_assets.sh` | helper | sim | none | yes | Fetch Godot showcase assets. | n/a |
| `godot/godot_gamepad_bridge.py` | experimental | sim | none | yes | Browser/gamepad bridge for Godot showcase. | n/a |
| `godot/godot_showcase.sh` | experimental | sim | none | yes | Launch Godot visual showcase. | n/a |
| `godot/install_godot4.sh` | helper | sim | none | yes | Install Godot 4. | n/a |
| `godot/test_godot_showcase.sh` | helper | sim | none | yes | Test Godot showcase launch. | n/a |
| `hardware/check_real_hardware_env.sh` | helper | real-dry-run | scout,aubo,ms42dc,gemini335,c16 | yes | Check real-hardware environment and device aliases. | n/a |
| `hardware/check_aubo_readonly.sh` | helper | real-dry-run | aubo | yes | Read-only Aubo network, RobotState RPC, interface, and ROS graph check; sends no motion goals. | n/a |
| `hardware/check_aubo_running_readonly.sh` | helper | real-dry-run | aubo | yes | Read-only Running/Normal and ROS graph check; reports state only and sends no motion goals. | n/a |
| `hardware/fetch_third_party.sh` | helper | mixed | none | yes | Fetch or link third-party packages. | n/a |
| `hardware/find_aubo_by_mac.py` | helper | real-dry-run | aubo | yes | Discover Aubo controller by MAC/IP. | n/a |
| `hardware/prepare_ms42dc_ros2.sh` | helper | mock | ms42dc | yes | Prepare vendor MS42DC ROS2 packages. | n/a |
| `hardware/prepare_real_hardware_ros.sh` | helper | mock | scout,aubo,ms42dc,c16 | yes | Prepare real-hardware ROS dependencies. | n/a |
| `hardware/real_arm_test.sh` | helper | real-dry-run | aubo | yes | Arm test wrapper, dry-run unless motion confirmation is provided. | Motion requires forwarded confirmation. |
| `hardware/real_aubo_bringup.sh` | helper | real-dry-run | aubo | yes | Start Aubo driver after explicit driver confirmation. | Requires `ARACHNE_CONFIRM_AUBO_DRIVER=YES`. |
| `hardware/real_aubo_payload.py` | helper | real-execute | aubo | no | Set Aubo payload over SDK. | Requires payload confirmation in wrappers. |
| `hardware/real_aubo_prepare.py` | helper | real-dry-run | aubo | yes | Check Aubo Running/SafetyMode state. | n/a |
| `hardware/real_aubo_prepare.sh` | helper | real-dry-run | aubo | yes | Shell wrapper for Aubo prepare check. | n/a |
| `hardware/real_aubo_probe.py` | helper | real-dry-run | aubo | yes | Probe Aubo RPC/status. | n/a |
| `hardware/real_aubo_probe.sh` | helper | real-dry-run | aubo | yes | Shell wrapper for Aubo probe. | n/a |
| `hardware/real_aubo_remote_start.py` | helper | real-execute | aubo | no | Guarded Aubo remote power/startup state machine. | Requires `ARACHNE_CONFIRM_AUBO_REMOTE_START=YES`. |
| `hardware/real_aubo_remote_start.sh` | helper | real-execute | aubo | no | Shell wrapper for guarded Aubo remote startup. | Requires `ARACHNE_CONFIRM_AUBO_REMOTE_START=YES`. |
| `hardware/real_aubo_z_test.sh` | helper | real-execute | aubo | no | Small real Aubo Z-motion test. | Requires `ARACHNE_CONFIRM_REAL_MOTION=YES`. |
| `hardware/real_base_test.sh` | helper | real-execute | scout | no | Real Scout base motion test. | Requires explicit confirmation in script/env. |
| `hardware/real_bringup.sh` | primary | real-dry-run | scout,aubo,ms42dc | yes | Start real Scout/MS42DC/Aubo hardware layer. | Does not intentionally move hardware; checks Aubo state. |
| `hardware/real_full_acceptance.sh` | primary | real-execute | scout,aubo,ms42dc | no | Full real-hardware acceptance flow. | Requires `--yes` or `ARACHNE_CONFIRM_REAL_MOTION=YES`. |
| `hardware/real_full_teach.sh` | helper | real-execute | scout,aubo,ms42dc,gemini335 | no | Full real teach stack including Aubo startup. | Requires `--yes` or `ARACHNE_CONFIRM_REAL_TEACH=YES`; use `operator/teach_panel.sh` as the panel entry. |
| `hardware/real_grasp_console.sh` | deprecated | real-execute | scout,aubo,ms42dc,gemini335,c16 | no | Compatibility console wrapper kept for old commands. | Use `operator/teach_panel.sh`; old wrapper requires `--yes` or `ARACHNE_CONFIRM_REAL_GRASP_CONSOLE=YES`. |
| `hardware/real_grasp_console_remote.sh` | deprecated | real-execute | scout,aubo,ms42dc,gemini335,c16 | no | Compatibility/helper wrapper for remote planner plus old console. | Use `operator/teach_panel.sh`; wrapper starts old console with `--yes --quick`. |
| `hardware/real_grasp_status.sh` | helper | real-dry-run | scout,aubo,ms42dc,gemini335 | yes | Snapshot grasp-related topics/services. | n/a |
| `hardware/real_gripper_test.sh` | helper | real-execute | ms42dc | no | Real MS42DC gripper test. | Requires explicit confirmation in script/env. |
| `hardware/real_hardware_acceptance_test.sh` | helper | real-dry-run | scout,aubo,ms42dc | yes | Acceptance test wrapper, dry-run unless motion confirmation is set. | Motion requires `ARACHNE_CONFIRM_REAL_MOTION=YES`. |
| `hardware/real_lidar_nav.sh` | primary | real-dry-run | c16,scout | yes | Start lidar/Nav2 localization/navigation stack. | Navigation motion still depends on downstream commands. |
| `hardware/real_lidar_save_map.sh` | primary | real-dry-run | c16 | yes | Save lidar/SLAM map output. | n/a |
| `hardware/real_teach_demo.sh` | primary | real-execute | scout,aubo,ms42dc | no | One-command real teach demo: bringup, wait, panel, cleanup. | Assumes operator-supervised real hardware. |
| `hardware/stop_real_stack.sh` | primary | mixed | scout,aubo,ms42dc,gemini335,c16 | yes | Stop known Arachne real-stack processes. | n/a |
| `model/check_model.sh` | helper | mock | none | yes | Check xacro/URDF. | n/a |
| `model/check_tf.sh` | helper | mock | none | yes | Generate/check TF frames. | n/a |
| `model/convert_ms42dc_step.sh` | helper | mock | none | yes | Convert MS42DC STEP to STL. | n/a |
| `model/use_gripper.sh` | helper | mixed | none | yes | Dispatch model/demo commands by gripper type. | n/a |
| `model/view_model.sh` | primary | mock | none | yes | Primary URDF/TF/mesh model viewer. | n/a |
| `model/view_sensor_model.sh` | deprecated | mock | none | yes | Compatibility sensor-model view wrapper. | Use `model/view_model.sh`. |
| `operator/teach_panel.sh` | primary | real-execute | scout,aubo,ms42dc,gemini335,c16 | no | Primary real teach/replay panel entrypoint. | Operator must verify hardware state before motion/replay. |
| `remote/deploy_remote_planner.sh` | helper | mixed | none | yes | Deploy remote planner stack. | n/a |
| `remote/remote_moveit_planner_server.py` | experimental | mixed | none | yes | Remote MoveIt planner server. | n/a |
| `remote/remote_moveit_planner_stack.sh` | experimental | mixed | none | yes | Start/stop remote MoveIt planner stack. | n/a |
| `remote/remote_planner_client.py` | experimental | mixed | none | yes | Remote planner client/health check. | n/a |
| `remote/remote_planner_server.py` | experimental | mixed | none | yes | Lightweight remote planner server. | n/a |
| `remote/sync_remote_planner.sh` | helper | mixed | none | yes | Sync remote planner files. | n/a |
| `sim/gazebo_autopick_demo.sh` | primary | sim | none | yes | Gazebo autonomous pick validation. | n/a |
| `sim/moveit_grasp_planning_demo.sh` | primary | sim | none | yes | MoveIt grasp planning demo. | n/a |
| `sim/switch_demo.sh` | primary | sim | none | yes | Switch/RViz/Gazebo playable demo. | n/a |
| `sim/test_gripper_sim.sh` | helper | sim | none | yes | Gripper simulation regression. | n/a |
| `sim/urban_trash_sorting_demo.sh` | primary | sim | none | yes | Road-cleanup semantic simulation. | n/a |
| `test/smoke_aubo_move_joint_dry_run.sh` | helper | mock | none | yes | Start only dry-run AuboMoveJoint action server and send a dry-run mock goal. | Requires `dry_run:=true`; never use for real motion. |
| `test/smoke_demo_orchestrator_offline.sh` | helper | mock | none | yes | Start demo_orchestrator offline and call only status/preflight. | Does not call visual grasp or road cleanup start. |
| `vision/apriltag_hand_eye_calibration.sh` | helper | real-dry-run | gemini335,aubo | yes | AprilTag hand-eye calibration capture/solve entry. | n/a |
| `vision/apriltag_nav_initialize.sh` | primary | real-dry-run | gemini335,c16 | yes | AprilTag navigation initialization. | n/a |
| `vision/apriltag_nav_start_mapping.sh` | primary | real-dry-run | gemini335,c16 | yes | AprilTag-assisted mapping start. | n/a |
| `vision/download_yolo_weights.sh` | helper | mock | none | yes | Download YOLO weights. | n/a |
| `vision/export_yolo_engine.sh` | helper | mock | none | yes | Export TensorRT engine. | n/a |
| `vision/gemini335_bringup.sh` | primary | real-dry-run | gemini335 | yes | Start Gemini335 camera node. | n/a |
| `vision/gemini_yolo_detect.py` | helper | mixed | gemini335 | yes | YOLO detection implementation. | n/a |
| `vision/gemini_yolo_live.sh` | primary | real-dry-run | gemini335 | yes | Gemini335 + YOLO live preview. | n/a |
| `vision/gemini_yolo_test.sh` | experimental | mock | none | yes | Offline/single-machine YOLO test. | n/a |
| `vision/grasp_preview.sh` | helper | mixed | gemini335,aubo,ms42dc | yes | Camera/YOLO/point-cloud/MoveIt grasp preview pipeline. | Real execution is through `grasp_preview_real_sync.sh --execute-real`. |
| `vision/grasp_preview_pipeline.py` | helper | mixed | gemini335,aubo,ms42dc | yes | Core grasp preview/planning/execution implementation. | Use shell wrappers directly. |
| `vision/grasp_preview_real_sync.sh` | primary | mixed | gemini335,aubo,ms42dc | yes | Sync preview from real Aubo pose; optional guarded execution. | Real execution requires `--execute-real` and `ARACHNE_CONFIRM_GRASP_EXECUTE_REAL=YES`. |
| `vision/grasp_task_server.sh` | primary | mixed | scout,aubo,ms42dc,gemini335 | yes | Launch grasp task server. | Real execution requires launch args `execute_real:=true confirm_execute_real:=true`. |
| `vision/mock_road_cleanup_task_test.py` | experimental | mock | none | yes | Mock road-cleanup smoke test. | n/a |
| `vision/raw_image_viewer.py` | helper | mixed | gemini335 | yes | OpenCV raw image topic viewer. | n/a |
| `vision/road_cleanup_task_server.sh` | primary | mixed | scout,aubo,ms42dc,gemini335 | yes | Launch road-cleanup task server. | Calls grasp task server; real motion depends on running services. |
| `vision/setup_yolo_env.sh` | helper | mock | none | yes | Set up YOLO virtual environment. | n/a |
| `vision/stop_gemini_yolo_live.sh` | helper | mixed | gemini335 | yes | Stop Gemini YOLO live process. | n/a |
