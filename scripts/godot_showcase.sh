#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_DIR="${ROOT_DIR}/godot/arachne_showcase"

"${PROJECT_DIR}/tools/link_assets.sh"

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

exec "${GODOT_BIN}" --path "${PROJECT_DIR}" "$@"
