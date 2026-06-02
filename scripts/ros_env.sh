#!/usr/bin/env bash
# Shared ROS environment helpers for Arachne shell entrypoints.

arachne_detect_ros_distro() {
  if [[ -n "${ROS_DISTRO:-}" ]]; then
    if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
      printf '%s\n' "${ROS_DISTRO}"
      return 0
    fi
    echo "ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
    return 1
  fi

  local ubuntu_codename=""
  if [[ -r /etc/os-release ]]; then
    ubuntu_codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-}")"
  fi

  local candidates=()
  case "${ubuntu_codename}" in
    jammy) candidates=(humble jazzy) ;;
    noble) candidates=(jazzy humble) ;;
    *) candidates=(humble jazzy) ;;
  esac

  local distro
  for distro in "${candidates[@]}"; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      printf '%s\n' "${distro}"
      return 0
    fi
  done

  echo "ROS setup not found. Set ROS_DISTRO=humble or jazzy, then retry." >&2
  return 1
}

arachne_require_ros_distro() {
  ROS_DISTRO="$(arachne_detect_ros_distro)" || return 1
  export ROS_DISTRO
}

arachne_source_bash_file() {
  local path="$1"
  local nounset_was_enabled=0
  local status=0

  case "$-" in
    *u*) nounset_was_enabled=1 ;;
  esac

  set +u
  # shellcheck disable=SC1090
  source "${path}" || status=$?

  if [[ "${nounset_was_enabled}" == "1" ]]; then
    set -u
  else
    set +u
  fi

  return "${status}"
}

arachne_source_ros_setup() {
  arachne_require_ros_distro || return 1
  arachne_source_bash_file "/opt/ros/${ROS_DISTRO}/setup.bash"
}

arachne_source_workspace_setup() {
  local root_dir="$1"
  local message="${2:-Workspace is not built yet. Run ./scripts/build_workspace.sh first.}"

  if [[ ! -f "${root_dir}/install/setup.bash" ]]; then
    echo "${message}" >&2
    return 1
  fi

  arachne_source_bash_file "${root_dir}/install/setup.bash"
}
