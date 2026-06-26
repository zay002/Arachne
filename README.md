# Arachne

![Arachne](docs/demo/arachne.png)

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://releases.ubuntu.com/22.04/)

![Real robot](docs/demo/realbot.PNG)

Arachne is a ROS 2 Humble workspace for a Scout 2.0 mobile base, Aubo i5 arm,
MS42DC gripper, Gemini335 RGB-D camera, and C16 lidar.

## Build

```bash
source scripts/env/arachne_env.sh
./scripts/build/build_workspace.sh
source install/setup.bash
```

Build logs are written under `log/build/`.

## Run The Full Operator

Use this as the normal entrypoint:

```bash
source scripts/env/arachne_env.sh
source install/setup.bash
ros2 launch arachne_operator teach_panel.launch.py
```

This starts the operator panel, RViz/model view, camera controls, task servers,
and real hardware bringup. Start/stop camera, SLAM, grasp, road cleanup, gripper,
base, and arm actions from the panel.

The new `Step Demo` button runs a simplified road-cleanup flow: observe once,
stop detection, move the base forward by small steps, re-detect, then trigger
one or more normal grasp attempts when the trash is close enough.

## Maintenance Checks

These are for development only, not field operation:

```bash
ros2 run arachne_operator teach_panel --headless-check
ros2 run arachne_operator grasp_task_server --dry-run-check
ros2 run arachne_operator road_cleanup_task_server --dry-run-check
ros2 run arachne_operator arachne check entrypoints
ros2 run arachne_operator arachne check offline
```

## Safety

Dry-run and headless checks never start real motion. Real motion paths must keep
their existing explicit confirmation parameters and field safety checks.

## Layout

| Path                      | Purpose                                           |
| ------------------------- | ------------------------------------------------- |
| `src/arachne_operator`    | Teach panel and task servers                      |
| `src/arachne_hardware`    | Real hardware drivers/actions                     |
| `src/arachne_description` | Robot model and RViz assets                       |
| `src/arachne_sensors`     | Gemini335 camera nodes                            |
| `src/arachne_nav`         | Navigation starter configs                        |
| `scripts`                 | Bootstrap, install, and field helper scripts only |
| `third_party`             | Vendor packages required for builds               |
| `yolo_workspace`          | Local model weights and datasets                  |

## Deprecated

Runtime shell wrappers under `scripts/` are not stable interfaces. Use
`ros2 launch arachne_operator teach_panel.launch.py` for normal operation.
