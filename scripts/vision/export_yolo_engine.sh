#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROJECT_DIR="${ARACHNE_YOLO_PROJECT_DIR:-${ROOT_DIR}/yolo_workspace}"
VENV_DIR="${ARACHNE_YOLO_VENV:-${PROJECT_DIR}/.venv}"
MODEL="${1:-yolo26n.pt}"
PRECISION="${2:-fp16}"
IMGSZ="${ARACHNE_YOLO_IMGSZ:-320}"
BATCH="${ARACHNE_YOLO_BATCH:-1}"
WORKSPACE="${ARACHNE_YOLO_TRT_WORKSPACE_GB:-2}"
DATA="${ARACHNE_YOLO_DATA:-${PROJECT_DIR}/configs/trash_mvp.yaml}"

case "${PRECISION}" in
  fp32|fp16|int8) ;;
  *)
    echo "Usage: ./scripts/vision/export_yolo_engine.sh [model.pt] [fp32|fp16|int8]" >&2
    exit 2
    ;;
esac

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  "${ROOT_DIR}/scripts/vision/setup_yolo_env.sh"
fi

mkdir -p "${PROJECT_DIR}/weights" "${PROJECT_DIR}/engines" "${PROJECT_DIR}/runs" "${PROJECT_DIR}/ultralytics_config"
export YOLO_CONFIG_DIR="${PROJECT_DIR}/ultralytics_config"
export MPLCONFIGDIR="${PROJECT_DIR}/ultralytics_config/matplotlib"

MODEL_PATH="${MODEL}"
if [[ ! -f "${MODEL_PATH}" && -f "${PROJECT_DIR}/weights/${MODEL}" ]]; then
  MODEL_PATH="${PROJECT_DIR}/weights/${MODEL}"
fi

"${VENV_DIR}/bin/python" - "${MODEL_PATH}" "${PRECISION}" "${IMGSZ}" "${BATCH}" "${WORKSPACE}" "${DATA}" "${PROJECT_DIR}/engines" <<'PY'
import shutil
import sys
from pathlib import Path

from ultralytics import YOLO

model_path = Path(sys.argv[1])
precision = sys.argv[2]
imgsz = int(sys.argv[3])
batch = int(sys.argv[4])
workspace = float(sys.argv[5])
data = Path(sys.argv[6])
engine_dir = Path(sys.argv[7])

kwargs = {
    "format": "engine",
    "imgsz": imgsz,
    "batch": batch,
    "workspace": workspace,
    "verbose": False,
}
if precision == "fp16":
    kwargs["half"] = True
elif precision == "int8":
    if not data.exists():
        raise SystemExit(f"INT8 requires calibration data yaml: {data}")
    kwargs["int8"] = True
    kwargs["data"] = str(data)

model = YOLO(str(model_path))
out = Path(model.export(**kwargs))
engine_dir.mkdir(parents=True, exist_ok=True)
target = engine_dir / f"{model_path.stem}_{precision}_{imgsz}.engine"
if out.resolve() != target.resolve():
    shutil.move(str(out), str(target))
print(target)
PY
