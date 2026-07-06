#!/usr/bin/env bash
# Arachne helper only; not a stable runtime entrypoint. Prefer ROS2 package entrypoints in README.md.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROJECT_DIR="${ARACHNE_YOLO_PROJECT_DIR:-${ROOT_DIR}/yolo_workspace}"
VENV_DIR="${ARACHNE_YOLO_VENV:-${PROJECT_DIR}/.venv}"
MODELS="${ARACHNE_YOLO_MODELS:-yolo26m.pt}"

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  "${ROOT_DIR}/scripts/vision/setup_yolo_env.sh"
fi

mkdir -p "${PROJECT_DIR}/weights" "${PROJECT_DIR}/ultralytics_config"
export YOLO_CONFIG_DIR="${PROJECT_DIR}/ultralytics_config"
export MPLCONFIGDIR="${PROJECT_DIR}/ultralytics_config/matplotlib"

cd "${PROJECT_DIR}/weights"

"${VENV_DIR}/bin/python" - ${MODELS} <<'PY'
import sys
from pathlib import Path

from ultralytics import YOLO

for model_name in sys.argv[1:]:
    print(f"loading {model_name}")
    model = YOLO(model_name)
    path = Path(model.ckpt_path or model_name)
    if path.exists():
        print(f"ready {path.resolve()} ({path.stat().st_size / 1024 / 1024:.1f} MiB)")
    else:
        print(f"ready {model_name}")
    onnx_path = path.with_suffix(".onnx")
    if not onnx_path.exists():
        exported = Path(model.export(format="onnx", imgsz=640))
        if exported != onnx_path:
            exported.replace(onnx_path)
    print(f"onnx {onnx_path.resolve()} ({onnx_path.stat().st_size / 1024 / 1024:.1f} MiB)")
PY
