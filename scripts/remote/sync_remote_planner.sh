#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

REMOTE_HOST="${ARACHNE_REMOTE_HOST:-}"
REMOTE_USER="${ARACHNE_REMOTE_USER:-}"
REMOTE_DIR="${ARACHNE_REMOTE_DIR:-~/projects/arachne_remote_planner}"
REMOTE_PORT="${ARACHNE_REMOTE_PORT:-8765}"
ACTION="${1:-push}"

if [[ -z "${REMOTE_HOST}" || -z "${REMOTE_USER}" ]]; then
  cat >&2 <<'EOF'
Set ARACHNE_REMOTE_HOST and ARACHNE_REMOTE_USER before using this script.
Do not commit real IP addresses, account names, passwords, or tokens.
EOF
  exit 2
fi

remote="${REMOTE_USER}@${REMOTE_HOST}"

case "${ACTION}" in
  push)
    ssh "${remote}" "mkdir -p ${REMOTE_DIR}/scripts ${REMOTE_DIR}/logs"
    rsync -az --delete \
      --exclude '__pycache__/' \
      --exclude '*.pyc' \
      "${ROOT_DIR}/scripts/remote/" \
      "${remote}:${REMOTE_DIR}/scripts/"
    ;;
  pull)
    mkdir -p "${ROOT_DIR}/scripts/remote"
    rsync -az \
      --exclude '__pycache__/' \
      --exclude '*.pyc' \
      "${remote}:${REMOTE_DIR}/scripts/" \
      "${ROOT_DIR}/scripts/remote/"
    ;;
  restart)
    ssh "${remote}" "
      cd ${REMOTE_DIR}
      if [[ -f server.pid ]]; then
        kill \$(cat server.pid) >/dev/null 2>&1 || true
      fi
      ps -eo pid=,args= | awk '\$2 == \"python3\" && \$3 == \"scripts/remote_planner_server.py\" {print \$1}' | xargs -r kill >/dev/null 2>&1 || true
      nohup python3 scripts/remote_planner_server.py \
        --host 127.0.0.1 \
        --port ${REMOTE_PORT} \
        --log-dir logs \
        > server.log 2>&1 &
      server_pid=\$!
      echo \${server_pid} > server.pid
      sleep 0.5
      cat server.pid
    "
    ;;
  status)
    ssh "${remote}" "
      cd ${REMOTE_DIR}
      if [[ -f server.pid ]]; then
        ps -p \$(cat server.pid) -o pid,cmd || true
      fi
      python3 scripts/remote_planner_client.py --url http://127.0.0.1:${REMOTE_PORT} --health || true
    "
    ;;
  *)
    cat >&2 <<EOF
Usage: $0 push|pull|restart|status

Environment:
  ARACHNE_REMOTE_HOST=${REMOTE_HOST}
  ARACHNE_REMOTE_USER=${REMOTE_USER}
  ARACHNE_REMOTE_DIR=${REMOTE_DIR}
  ARACHNE_REMOTE_PORT=${REMOTE_PORT}
EOF
    exit 2
    ;;
esac
