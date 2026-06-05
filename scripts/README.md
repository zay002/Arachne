# Arachne Scripts

Scripts are organized by function. Use the categorized paths directly:

```bash
source scripts/env/arachne_env.sh
./scripts/build/build_workspace.sh
./scripts/hardware/real_full_teach.sh --yes
./scripts/vision/gemini_yolo_live.sh
./scripts/vision/grasp_preview.sh
./scripts/vision/grasp_preview_real_sync.sh --sync-only
```

| Directory | Purpose |
| --- | --- |
| `env/` | ROS and workspace environment setup |
| `build/` | Colcon build, setup, and workspace checks |
| `hardware/` | Real hardware bringup, Aubo helpers, acceptance tests, serial checks |
| `operator/` | Teach-panel launch entry |
| `vision/` | Gemini335, YOLO, TensorRT export, live detection, grasp-to-basket preview |
| `model/` | URDF, TF, gripper, and RViz model checks |
| `sim/` | Gazebo demos and simulation validation |
| `godot/` | Godot showcase setup and bridge helpers |

When adding a new script, put it in the matching subdirectory and update README/manual command examples to use that categorized path.
