#!/usr/bin/env bash
set -euo pipefail

export ARACHNE_TEACH_PANEL_CONFIRM=YES
export AUBO_PREP_TIMEOUT="${AUBO_PREP_TIMEOUT:-6.0}"
export ARACHNE_TEACH_BRINGUP_GRACE_SEC="${ARACHNE_TEACH_BRINGUP_GRACE_SEC:-30}"
# For 真实面板快速启动：默认跳过 Aubo 只读预检；若需预检请在环境中显式设为 false。
export SKIP_AUBO_CHECK="${SKIP_AUBO_CHECK:-true}"
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/start_real_teach_with_bringup.sh"
