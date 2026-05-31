#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AUBO_ROBOT_IP="${AUBO_ROBOT_IP:-192.168.127.128}"

exec "${ROOT_DIR}/scripts/real_aubo_probe.py" --ip "${AUBO_ROBOT_IP}" "$@"
