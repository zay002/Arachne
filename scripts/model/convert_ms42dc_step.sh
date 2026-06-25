#!/usr/bin/env bash
# Arachne helper only; not a stable runtime entrypoint. Prefer ROS2 package entrypoints in README.md.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SOURCE_STEP="${SOURCE_STEP:-${ROOT_DIR}/third_party/MS42DC.step}"
OUTPUT_STL="${OUTPUT_STL:-${ROOT_DIR}/src/arachne_description/meshes/gripper/ms42dc/MS42DC.stl}"

if ! command -v gmsh >/dev/null; then
  echo "gmsh is required to convert STEP to STL. Install it with: sudo apt-get install -y gmsh" >&2
  exit 1
fi

if [[ ! -f "${SOURCE_STEP}" ]]; then
  echo "Missing source STEP file: ${SOURCE_STEP}" >&2
  exit 1
fi

mkdir -p "$(dirname "${OUTPUT_STL}")"
gmsh -2 "${SOURCE_STEP}" -format stl -o "${OUTPUT_STL}"
echo "Wrote ${OUTPUT_STL}"
