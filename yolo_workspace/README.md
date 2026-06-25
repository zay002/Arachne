# Arachne YOLO Workspace

This directory keeps the YOLO side of the static trash picking and charging-gun
perception projects separate from ROS 2 packages.

## Layout

- `.venv/`: local Python environment, reusing Jetson system torch/TensorRT.
- `weights/`: downloaded `.pt` weights.
- `engines/`: TensorRT `.engine` exports.
- `datasets/`: labeled trash, workpiece, and charging-gun datasets.
- `calibration/`: representative Gemini335 images for INT8 calibration.
- `runs/`: Ultralytics training, validation, export, and benchmark outputs.
- `ultralytics_config/`: local Ultralytics and matplotlib cache/config.

Only this README, `.gitignore`, and empty directory markers are intended to be
tracked. Weights, engines, datasets, and runs stay local.

## Current Choice

Use YOLO26 as the default family.

- First MVP: `yolo26n.pt` for fast detection.
- Grasp-quality candidate: `yolo26n-seg.pt` for mask-based 3D localization.
- Later comparison: `yolo26s.pt` or `yolo26s-seg.pt` only if TensorRT FPS has
  enough headroom.

The final trash and charging-gun models should be fine-tuned on
Arachne/Gemini335 images. COCO weights are only the bootstrap baseline; a
charging gun needs a dedicated local dataset.

## Commands

```bash
cd /home/jetson/zhaoyang/Arachne
./scripts/vision/setup_yolo_env.sh
./scripts/vision/download_yolo_weights.sh
```

Activate the environment:

```bash
source /home/jetson/zhaoyang/Arachne/yolo_workspace/.venv/bin/activate
```

Use representative local validation images for INT8 calibration. Do not export
the final INT8 engine with a generic sample dataset unless it is only a quick
smoke test.

Export a fast FP16 TensorRT smoke-test engine:

```bash
./scripts/vision/export_yolo_engine.sh yolo26n.pt fp16
```

Export INT8 only after `datasets/trash_mvp/images/val` contains representative
Gemini335 images:

```bash
./scripts/vision/export_yolo_engine.sh yolo26n.pt int8
```

Run the package-level grasp task server after the workspace is built:

```bash
ros2 run arachne_operator grasp_task_server
```
