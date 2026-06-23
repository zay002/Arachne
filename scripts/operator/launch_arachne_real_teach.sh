#!/usr/bin/env bash
set -euo pipefail

export ARACHNE_TEACH_PANEL_CONFIRM=YES
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/start_real_teach_with_bringup.sh"
