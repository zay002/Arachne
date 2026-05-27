#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT_DIR="$(cd "${PROJECT_DIR}/../.." && pwd)"
VENDOR_DIR="${PROJECT_DIR}/assets/vendor"
GENERATED_DIR="${PROJECT_DIR}/assets/generated"

mkdir -p "${VENDOR_DIR}" "${GENERATED_DIR}"

link_dir() {
  local name="$1"
  local target="$2"
  local link="${VENDOR_DIR}/${name}"

  if [[ ! -e "${target}" ]]; then
    echo "Missing optional Godot asset source: ${target}" >&2
    return
  fi

  rm -rf "${link}"
  ln -s "${target}" "${link}"
}

link_first_existing() {
  local name="$1"
  shift
  local target
  for target in "$@"; do
    if [[ -e "${target}" ]]; then
      link_dir "${name}" "${target}"
      return
    fi
  done
  echo "Missing optional Godot asset source for ${name}" >&2
}

link_first_existing \
  scout \
  "${ROOT_DIR}/src/vendor/scout_description/meshes" \
  "${ROOT_DIR}/third_party/scout_ros2/scout_description/meshes" \
  "${ROOT_DIR}/third_party/scout_ros/scout_description/meshes"

link_first_existing \
  aubo_i5 \
  "${ROOT_DIR}/src/vendor/aubo_description/meshes/aubo_i5/visual" \
  "${ROOT_DIR}/third_party/aubo_description/meshes/aubo_i5/visual"

link_first_existing \
  ms42dc \
  "${ROOT_DIR}/src/arachne_description/meshes/gripper/ms42dc/split" \
  "${ROOT_DIR}/third_party/MS42DC_SPLIT"

link_first_existing \
  ag95 \
  "${ROOT_DIR}/src/vendor/dh_ag95_description/meshes/visual" \
  "${ROOT_DIR}/third_party/dh_ag95_gripper_ros2/dh_ag95_description/meshes/visual"

link_first_existing \
  props \
  "${ROOT_DIR}/third_party/LARA_AUBOi5_AG95/lara_description/meshes/parts"

convert_mesh() {
  local source="$1"
  local target="$2"

  if [[ ! -e "${source}" ]]; then
    echo "Missing optional Godot mesh conversion source: ${source}" >&2
    return
  fi

  if ! command -v assimp >/dev/null 2>&1; then
    echo "Skipping DAE -> GLB conversion because assimp is not installed: ${source}" >&2
    return
  fi

  mkdir -p "$(dirname "${target}")"
  if [[ -f "${target}" && "${target}" -nt "${source}" ]]; then
    return
  fi

  assimp export "${source}" "${target}" >/dev/null
}

convert_mesh "${VENDOR_DIR}/scout/base_link.dae" "${GENERATED_DIR}/scout/base_link.glb"
convert_mesh "${VENDOR_DIR}/scout/base_link_full.dae" "${GENERATED_DIR}/scout/base_link_full.glb"
convert_mesh "${VENDOR_DIR}/scout/wheel_type1.dae" "${GENERATED_DIR}/scout/wheel_type1.glb"
convert_mesh "${VENDOR_DIR}/scout/wheel_type2.dae" "${GENERATED_DIR}/scout/wheel_type2.glb"

for i in 0 1 2 3 4 5 6; do
  convert_mesh "${VENDOR_DIR}/aubo_i5/link${i}.DAE" "${GENERATED_DIR}/aubo_i5/link${i}.glb"
done

for part in base mid left_finger right_finger; do
  convert_mesh "${VENDOR_DIR}/ms42dc/ms42dc_${part}.stl" "${GENERATED_DIR}/ms42dc/ms42dc_${part}.glb"
done

echo "Godot mesh links are ready in ${VENDOR_DIR}"
echo "Godot generated mesh cache is ready in ${GENERATED_DIR}"
