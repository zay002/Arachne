# Stage 0 Report: Repository Foundation

## Result

The repository is now a ROS2 workspace with a first package, reproducible setup scripts, and documentation placeholders for hardware, modeling, calibration, control, and references.

## Core Files

- `README.md`: current system overview and out-of-box usage path.
- `scripts/setup_ubuntu.sh`: installs ROS2 Humble or Jazzy dependencies.
- `scripts/check_model.sh`: generates the URDF and runs `check_urdf` when available.
- `docs/*.md`: concise notes for hardware facts, modeling policy, calibration, control, and references.
- `third_party/README.md`: records how vendor files should be tracked.

## Relationships

The root README is the user entry point. Scripts make the environment reproducible. The docs record which hardware models come from upstream repositories and where future measured hardware data should be stored.
