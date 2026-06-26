#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT_DIR}"

LOG_DIR="${ROOT_DIR}/log/desktop_teach_panel"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/teach_panel_$(date +%Y%m%d_%H%M%S).log"
ln -sfn "${LOG_FILE}" "${LOG_DIR}/latest.log"
exec > >(tee -a "${LOG_FILE}") 2>&1

echo "Starting Arachne Teach Panel"
echo "Log: ${LOG_FILE}"

set +u
if [[ -f "${ROOT_DIR}/scripts/env/arachne_env.sh" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/scripts/env/arachne_env.sh"
fi
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash"
fi
set -u

started_at="$(date +%s)"
set +e
ros2 launch arachne_operator teach_panel.launch.py "$@"
status=$?
set -e
elapsed=$(( "$(date +%s)" - started_at ))
if [[ "${status}" -ne 0 && "${elapsed}" -lt 20 ]]; then
  echo
  echo "Teach panel exited quickly with status ${status}."
  echo "Log: ${LOG_FILE}"
  read -r -p "Press Enter to close..." _ </dev/tty || sleep 30
fi
exit "${status}"
