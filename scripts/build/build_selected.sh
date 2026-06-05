#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if [[ $# -lt 1 ]]; then
  echo "Usage: ./scripts/build/build_selected.sh <package> [<package> ...]" >&2
  exit 2
fi

set +u
export ARACHNE_ENV_NO_WORKSPACE=1
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
unset ARACHNE_ENV_NO_WORKSPACE
set -u

cd "${ROOT_DIR}"
arachne_remove_workspace_underlay "${ROOT_DIR}"

if [[ -z "${MAKEFLAGS:-}" && "$(uname -m)" == "aarch64" ]]; then
  export MAKEFLAGS="-j2"
fi

COLCON_ARGS=()
if [[ -n "${ARACHNE_COLCON_PARALLEL_WORKERS:-}" ]]; then
  COLCON_ARGS+=(--parallel-workers "${ARACHNE_COLCON_PARALLEL_WORKERS}")
elif [[ "$(uname -m)" == "aarch64" ]]; then
  COLCON_ARGS+=(--parallel-workers 2)
fi

colcon build "${COLCON_ARGS[@]}" --base-paths src --packages-select "$@" \
  --symlink-install \
  --cmake-args -DPython3_EXECUTABLE="${ARACHNE_SYSTEM_PYTHON}"
