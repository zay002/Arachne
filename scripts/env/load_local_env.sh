#!/usr/bin/env bash
# Source local, untracked operator settings.

arachne_load_local_env() {
  local root_dir="${1:-}"
  if [[ -z "${root_dir}" ]]; then
    root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
  fi
  local env_file="${ARACHNE_LOCAL_ENV_FILE:-${root_dir}/.env.local}"
  if [[ ! -f "${env_file}" ]]; then
    return 0
  fi
  set -a
  # shellcheck disable=SC1090
  source "${env_file}"
  set +a
}

