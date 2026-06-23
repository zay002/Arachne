#!/usr/bin/env bash
set -euo pipefail

export ARACHNE_TEACH_PANEL_CONFIRM=YES
export AUBO_PREP_TIMEOUT="${AUBO_PREP_TIMEOUT:-6.0}"
export ARACHNE_TEACH_BRINGUP_GRACE_SEC="${ARACHNE_TEACH_BRINGUP_GRACE_SEC:-30}"
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/start_real_teach_with_bringup.sh"
