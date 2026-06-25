#!/usr/bin/env bash
# Arachne helper only; not a stable runtime entrypoint. Prefer ROS2 package entrypoints in README.md.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROJECT_DIR="${ARACHNE_YOLO_PROJECT_DIR:-${ROOT_DIR}/yolo_workspace}"
LOG_DIR="${PROJECT_DIR}/runs/gemini_yolo_live"
PID_FILE="${LOG_DIR}/latest.pid"

if [[ ! -f "${PID_FILE}" ]]; then
  echo "No Gemini YOLO live pid file."
  exit 0
fi

PID="$(cat "${PID_FILE}" 2>/dev/null || true)"
if [[ -n "${PID}" ]] && kill -0 "${PID}" 2>/dev/null; then
  kill "${PID}" 2>/dev/null || true
  echo "Stopped Gemini YOLO live: pid=${PID}"
else
  echo "Gemini YOLO live is not running."
fi
rm -f "${PID_FILE}"
