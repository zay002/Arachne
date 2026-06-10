#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

REMOTE_HOST="${ARACHNE_REMOTE_HOST:-}"
REMOTE_USER="${ARACHNE_REMOTE_USER:-}"
REMOTE_DIR="${ARACHNE_REMOTE_DIR:-~/projects/arachne_remote_planner}"
REMOTE_PORT="${ARACHNE_REMOTE_PORT:-8765}"

if [[ -z "${REMOTE_HOST}" || -z "${REMOTE_USER}" ]]; then
  cat >&2 <<'EOF'
Set ARACHNE_REMOTE_HOST and ARACHNE_REMOTE_USER before deploying.
Do not commit real IP addresses, account names, passwords, or tokens.
EOF
  exit 2
fi

"${ROOT_DIR}/scripts/remote/sync_remote_planner.sh" push

cat <<EOF
Remote planner scripts deployed.

Start the server:

  ssh ${REMOTE_USER}@${REMOTE_HOST}
  cd ${REMOTE_DIR}
  python3 scripts/remote_planner_server.py --host 0.0.0.0 --port ${REMOTE_PORT} --log-dir logs

Probe from Jetson:

  python3 scripts/remote/remote_planner_client.py --url http://${REMOTE_HOST}:${REMOTE_PORT} --health
  python3 scripts/remote/remote_planner_client.py --url http://${REMOTE_HOST}:${REMOTE_PORT}

If the port is blocked, use an SSH tunnel:

  ssh -L ${REMOTE_PORT}:127.0.0.1:${REMOTE_PORT} ${REMOTE_USER}@${REMOTE_HOST}
  python3 scripts/remote/remote_planner_client.py --url http://127.0.0.1:${REMOTE_PORT} --health
EOF
