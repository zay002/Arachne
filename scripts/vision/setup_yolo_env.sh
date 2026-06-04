#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROJECT_DIR="${ARACHNE_YOLO_PROJECT_DIR:-${ROOT_DIR}/yolo_workspace}"
VENV_DIR="${ARACHNE_YOLO_VENV:-${PROJECT_DIR}/.venv}"
ULTRALYTICS_VERSION="${ARACHNE_ULTRALYTICS_VERSION:-8.4.60}"

mkdir -p \
  "${PROJECT_DIR}/weights" \
  "${PROJECT_DIR}/engines" \
  "${PROJECT_DIR}/datasets" \
  "${PROJECT_DIR}/calibration" \
  "${PROJECT_DIR}/runs" \
  "${PROJECT_DIR}/ultralytics_config"

python3 -m venv --system-site-packages "${VENV_DIR}"

# Reuse NVIDIA Jetson system packages for torch, torchvision, TensorRT, CUDA,
# and OpenCV. Installing Ultralytics without dependencies avoids replacing
# Jetson-specific wheels with generic pip wheels.
"${VENV_DIR}/bin/python" -m pip install --upgrade pip
"${VENV_DIR}/bin/python" -m pip install --no-deps "ultralytics==${ULTRALYTICS_VERSION}"

export YOLO_CONFIG_DIR="${PROJECT_DIR}/ultralytics_config"
export MPLCONFIGDIR="${PROJECT_DIR}/ultralytics_config/matplotlib"

"${VENV_DIR}/bin/python" - <<'PY'
import importlib
import torch

checks = {
    "torch": torch.__version__,
    "torchvision": importlib.import_module("torchvision").__version__,
    "tensorrt": importlib.import_module("tensorrt").__version__,
    "ultralytics": importlib.import_module("ultralytics").__version__,
    "cv2": importlib.import_module("cv2").__version__,
}
for name, version in checks.items():
    print(f"{name}: {version}")

print(f"cuda_available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"cuda_device: {torch.cuda.get_device_name(0)}")
PY

cat <<EOF

YOLO environment ready.
Project directory:
  ${PROJECT_DIR}
Activate with:
  source "${VENV_DIR}/bin/activate"
EOF
