#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/arachne_env.sh"
set -u

RECORDING_DIR="${ARACHNE_TEACH_RECORDING_DIR:-${ROOT_DIR}/recordings/teach}"
mkdir -p "${RECORDING_DIR}"

has_recording_dir=false
for arg in "$@"; do
  if [[ "${arg}" == recording_dir:=* || "${arg}" == --recording_dir:=* ]]; then
    has_recording_dir=true
    break
  fi
done

if [[ "${has_recording_dir}" == "true" ]]; then
  exec ros2 launch arachne_operator teach_panel.launch.py "$@"
fi

exec ros2 launch arachne_operator teach_panel.launch.py \
  recording_dir:="${RECORDING_DIR}" "$@"
