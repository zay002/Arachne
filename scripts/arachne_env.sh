#!/usr/bin/env bash
# Source this file before building or running Arachne from a shell that may have
# conda/pyenv Python ahead of the Ubuntu Python used by ROS 2.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "This script must be sourced, not executed:" >&2
  echo "  source scripts/arachne_env.sh" >&2
  exit 2
fi

ARACHNE_ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ARACHNE_ROOT_DIR
ARACHNE_ENV_SOURCE_WORKSPACE=1
if [[ "${ARACHNE_ENV_NO_WORKSPACE:-0}" == "1" ]]; then
  ARACHNE_ENV_SOURCE_WORKSPACE=0
fi

export ARACHNE_SYSTEM_PYTHON="${ARACHNE_SYSTEM_PYTHON:-/usr/bin/python3}"
if [[ ! -x "${ARACHNE_SYSTEM_PYTHON}" ]]; then
  echo "ARACHNE_SYSTEM_PYTHON is not executable: ${ARACHNE_SYSTEM_PYTHON}" >&2
  return 1
fi

arachne_filter_colon_var() {
  local value="${1:-}"
  local filtered=()
  local entry
  IFS=':' read -r -a entries <<< "${value}"
  for entry in "${entries[@]}"; do
    [[ -z "${entry}" ]] && continue
    case "${entry}" in
      *miniconda*|*anaconda*|*.conda*|*/.pyenv/shims|*/.pyenv/versions/*)
        continue
        ;;
    esac
    filtered+=("${entry}")
  done
  local IFS=':'
  echo "${filtered[*]}"
}

ARACHNE_FILTERED_PATH="$(arachne_filter_colon_var "${PATH:-}")"
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
if [[ -n "${ARACHNE_FILTERED_PATH}" ]]; then
  export PATH="${PATH}:${ARACHNE_FILTERED_PATH}"
fi
unset ARACHNE_FILTERED_PATH

unset PYTHONHOME
if [[ -n "${PYTHONPATH:-}" ]]; then
  export PYTHONPATH="$(arachne_filter_colon_var "${PYTHONPATH}")"
fi
export PYTHONNOUSERSITE="${PYTHONNOUSERSITE:-1}"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    export ROS_DISTRO=jazzy
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    export ROS_DISTRO=humble
  fi
fi

if [[ -z "${ROS_DISTRO:-}" || ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS setup not found. Set ROS_DISTRO=jazzy or humble, then source this file again." >&2
  return 1
fi

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ "${ARACHNE_ENV_SOURCE_WORKSPACE}" == "1" && -f "${ARACHNE_ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ARACHNE_ROOT_DIR}/install/setup.bash"
fi

if [[ "${ARACHNE_ENV_QUIET:-0}" != "1" ]]; then
  echo "Arachne environment ready:"
  echo "  ROS_DISTRO=${ROS_DISTRO}"
  echo "  python3=$(command -v python3) ($("${ARACHNE_SYSTEM_PYTHON}" --version 2>&1))"
  echo "  ARACHNE_SYSTEM_PYTHON=${ARACHNE_SYSTEM_PYTHON}"
fi
unset ARACHNE_ENV_SOURCE_WORKSPACE
