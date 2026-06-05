#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROJECT_DIR="${ARACHNE_YOLO_PROJECT_DIR:-${ROOT_DIR}/yolo_workspace}"
VENV_DIR="${ARACHNE_YOLO_VENV:-${PROJECT_DIR}/.venv}"
MODEL="${ARACHNE_YOLO_MODEL:-${PROJECT_DIR}/weights/yolo26n.pt}"

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  "${ROOT_DIR}/scripts/vision/setup_yolo_env.sh"
fi
if [[ ! -f "${MODEL}" ]]; then
  "${ROOT_DIR}/scripts/vision/download_yolo_weights.sh"
fi

mkdir -p "${PROJECT_DIR}/runs/gemini_yolo" "${PROJECT_DIR}/ultralytics_config"
export YOLO_CONFIG_DIR="${PROJECT_DIR}/ultralytics_config"
export MPLCONFIGDIR="${PROJECT_DIR}/ultralytics_config/matplotlib"

exec "${VENV_DIR}/bin/python" "${ROOT_DIR}/scripts/vision/gemini_yolo_detect.py" \
  --model "${MODEL}" \
  --output-dir "${PROJECT_DIR}/runs/gemini_yolo" \
  "$@"
