#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROJECT_DIR="${ARACHNE_YOLO_PROJECT_DIR:-${ROOT_DIR}/yolo_workspace}"
LOG_DIR="${PROJECT_DIR}/runs/gemini_yolo_live"
LOG_FILE="${LOG_DIR}/latest.log"
PID_FILE="${LOG_DIR}/latest.pid"

mkdir -p "${LOG_DIR}"

if [[ -f "${PID_FILE}" ]]; then
  OLD_PID="$(cat "${PID_FILE}" 2>/dev/null || true)"
  if [[ -n "${OLD_PID}" ]] && kill -0 "${OLD_PID}" 2>/dev/null; then
    kill "${OLD_PID}" 2>/dev/null || true
    sleep 0.5
  fi
fi

cd "${ROOT_DIR}"
export DISPLAY="${DISPLAY:-:0}"
if [[ -z "${XAUTHORITY:-}" && -f "${HOME}/.Xauthority" ]]; then
  export XAUTHORITY="${HOME}/.Xauthority"
fi
export PYTHONUNBUFFERED=1

CLASS_ARGS=()
if [[ -v ARACHNE_YOLO_CLASSES ]]; then
  if [[ -n "${ARACHNE_YOLO_CLASSES}" ]]; then
    CLASS_ARGS+=(--classes "${ARACHNE_YOLO_CLASSES}")
  fi
else
  CLASS_ARGS+=(--classes "bottle,cup,bowl")
fi

nohup setsid ./scripts/vision/gemini_yolo_test.sh \
  --duration 0 \
  --every "${ARACHNE_YOLO_EVERY:-5}" \
  --imgsz "${ARACHNE_YOLO_IMGSZ:-640}" \
  --conf "${ARACHNE_YOLO_CONF:-0.25}" \
  "${CLASS_ARGS[@]}" \
  --show \
  "$@" >"${LOG_FILE}" 2>&1 &

PID="$!"
echo "${PID}" > "${PID_FILE}"
echo "Gemini YOLO live started: pid=${PID}"
echo "Log: ${LOG_FILE}"
