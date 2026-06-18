#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PANEL_ARGS=()

while (($#)); do
  case "$1" in
    -y|--yes|--quick|--skip-env-check)
      ;;
    --terminal)
      shift
      ;;
    --no-stop-existing)
      PANEL_ARGS+=("--no-stop-existing")
      ;;
    --)
      shift
      PANEL_ARGS+=("$@")
      break
      ;;
    *)
      PANEL_ARGS+=("$1")
      ;;
  esac
  shift
done

echo "real_grasp_console.sh is deprecated; starting scripts/operator/teach_panel.sh" >&2
exec "${ROOT_DIR}/scripts/operator/teach_panel.sh" "${PANEL_ARGS[@]}"
