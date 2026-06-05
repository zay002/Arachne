#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROJECT_DIR="${ARACHNE_YOLO_PROJECT_DIR:-${ROOT_DIR}/yolo_workspace}"
VENV_DIR="${ARACHNE_YOLO_VENV:-${PROJECT_DIR}/.venv}"
MODELS="${ARACHNE_YOLO_MODELS:-yolo26n.pt yolo26n-seg.pt}"

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
PY
