#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
AUBO_ROBOT_IP="${AUBO_ROBOT_IP:-192.168.127.128}"

set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
set -u

exec "${ARACHNE_SYSTEM_PYTHON}" "${ROOT_DIR}/scripts/hardware/real_aubo_probe.py" --ip "${AUBO_ROBOT_IP}" "$@"
