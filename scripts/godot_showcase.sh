#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_DIR="${ROOT_DIR}/godot/arachne_showcase"
ARACHNE_SYSTEM_PYTHON="${ARACHNE_SYSTEM_PYTHON:-/usr/bin/python3}"

"${PROJECT_DIR}/tools/link_assets.sh"

if command -v ros2 >/dev/null 2>&1; then
  export ARACHNE_ROS2_AVAILABLE="${ARACHNE_ROS2_AVAILABLE:-1}"
fi

IS_WSL=false
if [[ -r /proc/sys/kernel/osrelease ]] && grep -qi "microsoft\\|wsl" /proc/sys/kernel/osrelease; then
  IS_WSL=true
fi

GODOT_ARGS=()
START_GAMEPAD_BRIDGE=false
if [[ "${IS_WSL}" == "true" ]]; then
  export LD_LIBRARY_PATH="/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}"
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}"
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
  if [[ -x /usr/lib/wsl/lib/nvidia-smi ]]; then
    export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
  fi
  export ARACHNE_GODOT_PROFILE="${ARACHNE_GODOT_PROFILE:-wsl}"
  GODOT_ARGS+=(--rendering-driver opengl3 --rendering-method gl_compatibility --render-thread "${GODOT_RENDER_THREAD:-safe}" --disable-vsync)
  START_GAMEPAD_BRIDGE=true
else
  export ARACHNE_GODOT_PROFILE="${ARACHNE_GODOT_PROFILE:-cinematic}"
  GODOT_ARGS+=(--render-thread "${GODOT_RENDER_THREAD:-safe}")
fi

case "${GODOT_GAMEPAD_BRIDGE:-auto}" in
  1|true|TRUE|yes|YES|web|udp)
    START_GAMEPAD_BRIDGE=true
    ;;
  0|false|FALSE|no|NO|off)
    START_GAMEPAD_BRIDGE=false
    ;;
esac

if [[ -n "${GODOT_MAX_FPS:-}" ]]; then
  GODOT_ARGS+=(--max-fps "${GODOT_MAX_FPS}")
fi

GODOT_BIN="${GODOT_BIN:-}"
if [[ -z "${GODOT_BIN}" ]]; then
  if command -v godot4 >/dev/null 2>&1; then
    GODOT_BIN="godot4"
  elif command -v godot >/dev/null 2>&1; then
    GODOT_BIN="godot"
  else
    echo "Godot 4.x was not found. Set GODOT_BIN=/path/to/godot4 or install Godot 4.x." >&2
    exit 1
  fi
fi

BRIDGE_PID=""
cleanup() {
  if [[ -n "${BRIDGE_PID}" ]]; then
    kill "${BRIDGE_PID}" >/dev/null 2>&1 || true
  fi
}

if [[ "${START_GAMEPAD_BRIDGE}" == "true" ]]; then
  export ARACHNE_GODOT_GAMEPAD="${ARACHNE_GODOT_GAMEPAD:-udp}"
  export ARACHNE_GODOT_GAMEPAD_PORT="${ARACHNE_GODOT_GAMEPAD_PORT:-8791}"
  WEB_GAMEPAD_HOST="${WEB_GAMEPAD_HOST:-127.0.0.1}"
  WEB_GAMEPAD_PORT="${WEB_GAMEPAD_PORT:-8790}"
  "${ARACHNE_SYSTEM_PYTHON}" "${ROOT_DIR}/scripts/godot_gamepad_bridge.py" \
    --host "${WEB_GAMEPAD_HOST}" \
    --port "${WEB_GAMEPAD_PORT}" \
    --udp-port "${ARACHNE_GODOT_GAMEPAD_PORT}" &
  BRIDGE_PID="$!"
  trap cleanup EXIT
  echo "Open http://${WEB_GAMEPAD_HOST}:${WEB_GAMEPAD_PORT} in a browser for Switch Pro / web gamepad control."
fi

if [[ -n "${BRIDGE_PID}" ]]; then
  "${GODOT_BIN}" --path "${PROJECT_DIR}" "${GODOT_ARGS[@]}" "$@"
else
  exec "${GODOT_BIN}" --path "${PROJECT_DIR}" "${GODOT_ARGS[@]}" "$@"
fi
