#!/usr/bin/env bash
# Arachne helper only; not a stable runtime entrypoint. Prefer ROS2 package entrypoints in README.md.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ASSET_DIR="${ROOT_DIR}/third_party/kenney/furniture-kit"
ZIP_PATH="${ROOT_DIR}/third_party/kenney/kenney_furniture-kit.zip"
URL="https://kenney.nl/media/pages/assets/furniture-kit/440e0608a4-1677580847/kenney_furniture-kit.zip"

mkdir -p "$(dirname "${ZIP_PATH}")"

if [[ ! -f "${ASSET_DIR}/License.txt" ]]; then
  echo "Downloading Kenney Furniture Kit (CC0) for the Godot office showcase..."
  curl -L --fail --retry 3 -o "${ZIP_PATH}" "${URL}"
  rm -rf "${ASSET_DIR}"
  mkdir -p "${ASSET_DIR}"
  unzip -oq "${ZIP_PATH}" -d "${ASSET_DIR}"
else
  echo "Kenney Furniture Kit is already present: ${ASSET_DIR}"
fi

"${ROOT_DIR}/godot/arachne_showcase/tools/link_assets.sh"

GODOT_BIN="${GODOT_BIN:-}"
if [[ -z "${GODOT_BIN}" ]]; then
  if command -v godot4 >/dev/null 2>&1; then
    GODOT_BIN="godot4"
  elif command -v godot >/dev/null 2>&1; then
    GODOT_BIN="godot"
  fi
fi

if [[ -n "${GODOT_BIN}" ]]; then
  "${GODOT_BIN}" --headless --path "${ROOT_DIR}/godot/arachne_showcase" --import --quit >/dev/null
else
  echo "Godot was not found; the assets will be imported the next time the project is opened."
fi
