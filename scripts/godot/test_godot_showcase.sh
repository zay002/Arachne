#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROJECT_DIR="${ROOT_DIR}/godot/arachne_showcase"

"${PROJECT_DIR}/tools/link_assets.sh"

if [[ -r /proc/sys/kernel/osrelease ]] && grep -qi "microsoft\\|wsl" /proc/sys/kernel/osrelease; then
  export LD_LIBRARY_PATH="/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}"
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}"
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
  if [[ -x /usr/lib/wsl/lib/nvidia-smi ]]; then
    export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
  fi
  export ARACHNE_GODOT_PROFILE="${ARACHNE_GODOT_PROFILE:-wsl}"
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

exec "${GODOT_BIN}" --headless --path "${PROJECT_DIR}" -- --self-test
