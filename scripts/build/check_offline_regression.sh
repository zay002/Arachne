#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"
set +u
source "$ROOT_DIR/scripts/env/arachne_env.sh"
source "$ROOT_DIR/install/setup.bash" 2>/dev/null || true
set -u
exec ros2 run arachne_operator arachne check offline "$@"
