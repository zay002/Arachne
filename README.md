# Arachne

Arachne is a ROS 2 Humble workspace for a Scout 2.0 mobile base, Aubo i5 arm,
MS42DC gripper, Gemini335 RGB-D camera, and C16 lidar.

## Build

```bash
source scripts/env/arachne_env.sh
colcon build --base-paths src --packages-up-to arachne_operator
source install/setup.bash
```

## Stable Entrypoints

Use ROS 2 package entrypoints, not shell wrappers:

```bash
ros2 run arachne_operator teach_panel
ros2 run arachne_operator grasp_task_server
ros2 run arachne_operator road_cleanup_task_server
```

Launch files:

```bash
ros2 launch arachne_operator teach_panel.launch.py
ros2 launch arachne_operator grasp_task_server.launch.py
ros2 launch arachne_operator road_cleanup_task_server.launch.py
```

Use `teach_panel.launch.py` for the normal operator workflow: it starts the
panel, RViz/model visualization, managed camera/viewer controls, and the real
hardware bringup that publishes arm/gripper/base status. Direct
`ros2 run arachne_operator teach_panel` starts only the panel process, so
hardware status can stay `waiting`.

For offline UI checks without real drivers:

```bash
ros2 launch arachne_operator teach_panel.launch.py with_real_bringup:=false
```

Headless/dry-run checks:

```bash
ros2 run arachne_operator teach_panel --headless-check
ros2 run arachne_operator grasp_task_server --dry-run-check
ros2 run arachne_operator road_cleanup_task_server --dry-run-check
```

## Offline Checks

```bash
ros2 run arachne_operator arachne check entrypoints
ros2 run arachne_operator arachne smoke teach-panel --headless
ros2 run arachne_operator arachne smoke grasp-task --dry-run
ros2 run arachne_operator arachne smoke road-cleanup --dry-run
ros2 run arachne_operator arachne check offline
```

## Safety

Dry-run and headless checks never start real motion. Real motion paths must keep
their existing explicit confirmation parameters and field safety checks.

## Layout

| Path | Purpose |
| --- | --- |
| `src/arachne_operator` | Teach panel and task servers |
| `src/arachne_hardware` | Real hardware drivers/actions |
| `src/arachne_description` | Robot model and RViz assets |
| `src/arachne_sensors` | Gemini335 camera nodes |
| `src/arachne_nav` | Navigation starter configs |
| `scripts` | Bootstrap, install, and field helper scripts only |
| `third_party` | Vendor packages required for builds |
| `yolo_workspace` | Local model weights and datasets |

## Deprecated

Runtime shell wrappers under `scripts/` are not stable interfaces. Prefer the
ROS 2 commands above. Historical docs were removed; keep current operator
instructions here.
