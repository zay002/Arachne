#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
set -u

RECORDING_DIR="${ARACHNE_TEACH_RECORDING_DIR:-${ROOT_DIR}/recordings/teach}"
START_REAL_BRINGUP="${ARACHNE_TEACH_START_REAL_BRINGUP:-true}"
STOP_EXISTING="${ARACHNE_TEACH_STOP_EXISTING:-true}"
mkdir -p "${RECORDING_DIR}"

PANEL_ARGS=()
while (($#)); do
  case "$1" in
    --panel-only|--no-real-bringup)
      START_REAL_BRINGUP=false
      ;;
    --no-stop-existing)
      STOP_EXISTING=false
      ;;
    --)
      shift
      PANEL_ARGS+=("$@")
      break
      ;;
    *)
      PANEL_ARGS+=("$1")
      ;;
  esac
  shift
done

if [[ "${STOP_EXISTING}" == "true" ]]; then
  "${ROOT_DIR}/scripts/hardware/stop_real_stack.sh" || true
fi

has_recording_dir=false
for arg in "${PANEL_ARGS[@]}"; do
  if [[ "${arg}" == recording_dir:=* || "${arg}" == --recording_dir:=* ]]; then
    has_recording_dir=true
    break
  fi
done

bringup_pid=""
if [[ "${START_REAL_BRINGUP}" == "true" ]]; then
  "${ROOT_DIR}/scripts/hardware/real_bringup.sh" &
  bringup_pid=$!
  trap '[[ -n "${bringup_pid}" ]] && kill "${bringup_pid}" 2>/dev/null || true' EXIT
  sleep 2
  if ! kill -0 "${bringup_pid}" 2>/dev/null; then
    wait "${bringup_pid}" || true
    echo "real_bringup exited before teach panel startup; fix the hardware bringup error first." >&2
    exit 1
  fi
fi

if [[ "${has_recording_dir}" == "true" ]]; then
  exec ros2 launch arachne_operator teach_panel.launch.py "${PANEL_ARGS[@]}"
fi

exec ros2 launch arachne_operator teach_panel.launch.py \
  recording_dir:="${RECORDING_DIR}" "${PANEL_ARGS[@]}"
