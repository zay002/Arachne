#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

USE_SCOUT="${USE_SCOUT:-true}"
USE_MS42DC="${USE_MS42DC:-true}"
USE_AUBO="${USE_AUBO:-true}"
SCOUT_DRIVER="${SCOUT_DRIVER:-waveshare}"
AUBO_ROBOT_IP="${AUBO_ROBOT_IP:-192.168.127.128}"
SKIP_AUBO_CHECK="${SKIP_AUBO_CHECK:-false}"
HURRY_AUTO_ATTACH="${HURRY_AUTO_ATTACH:-true}"
AUBO_TEACH_FLAG_PATH="${AUBO_TEACH_FLAG_PATH:-/tmp/arachne_aubo_teach_mode}"
AUBO_KEEP_TEACH_FLAG="${AUBO_KEEP_TEACH_FLAG:-false}"
SHOW_ARGS=false
EXTRA_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./scripts/real_bringup.sh [options] [ros2 launch args...]

Starts the real Arachne hardware stack with the usual lab defaults.

Options:
  --no-scout          Do not start Scout 2.0.
  --no-ms42dc        Do not start the MS42DC gripper.
  --no-gripper       Alias for --no-ms42dc.
  --no-aubo          Do not start the Aubo i5 driver.
  --skip-aubo-check  Skip the read-only Aubo Running/SafetyMode check.
  -h, --help         Show this help.

Useful environment overrides:
  SCOUT_PORT=/dev/...       Override Scout Waveshare serial path.
  MS42DC_PORT=/dev/...      Override MS42DC serial path.
  AUBO_ROBOT_IP=...         Override Aubo controller IP.
  HURRY_AUTO_ATTACH=false   Disable automatic WSL2 usbipd attach attempts.
  AUBO_KEEP_TEACH_FLAG=true Keep an existing Aubo teach gate flag for debugging.
EOF
}

while (($#)); do
  case "$1" in
    --no-scout)
      USE_SCOUT=false
      ;;
    --no-ms42dc|--no-gripper)
      USE_MS42DC=false
      ;;
    --no-aubo)
      USE_AUBO=false
      ;;
    --skip-aubo-check)
      SKIP_AUBO_CHECK=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      if [[ "$1" == "--show-args" ]]; then
        SHOW_ARGS=true
        SKIP_AUBO_CHECK=true
      fi
      EXTRA_ARGS+=("$1")
      ;;
  esac
  shift
done

set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/arachne_env.sh"
set -u

fail_missing_port() {
  local label="$1"
  cat >&2 <<EOF
Could not find ${label}.

If you are in WSL2, attach USB devices first, then retry:
  hurry scan
  hurry attach <BUSID>

Current serial devices:
EOF
  ls -l /dev/serial/by-id/* /dev/ttyUSB* /dev/ttyACM* /dev/ttyCH* 2>/dev/null >&2 || true
  exit 1
}

is_wsl() {
  grep -qiE 'microsoft|wsl' /proc/version /proc/sys/kernel/osrelease 2>/dev/null
}

try_hurry_attach() {
  local label="$1"
  local scan_pattern="$2"

  if [[ "${HURRY_AUTO_ATTACH}" != "true" || "${SHOW_ARGS}" == "true" ]]; then
    return 1
  fi
  if ! is_wsl || ! command -v hurry >/dev/null 2>&1; then
    return 1
  fi

  echo "Trying hurry auto-attach for ${label}..." >&2
  local bus_id
  bus_id="$(
    hurry scan 2>/dev/null \
      | awk -v pat="${scan_pattern}" '$1 ~ /^usbipd:/ && $0 ~ pat {sub(/^usbipd:/, "", $1); print $1; exit}'
  )"
  if [[ -z "${bus_id}" ]]; then
    echo "hurry auto-attach: no matching Windows USB device for ${label}" >&2
    return 1
  fi

  hurry attach "${bus_id}" >&2 || return 1
  sleep 2
  return 0
}

resolve_port() {
  local env_value="$1"
  local label="$2"
  local attach_pattern="$3"
  shift 3

  if [[ -n "${env_value}" ]]; then
    if [[ -e "${env_value}" ]]; then
      printf '%s\n' "${env_value}"
      return 0
    fi
    try_hurry_attach "${label}" "${attach_pattern}" || true
    if [[ -e "${env_value}" ]]; then
      printf '%s\n' "${env_value}"
      return 0
    fi
    printf 'Configured %s does not exist: %s\n' "${label}" "${env_value}" >&2
    fail_missing_port "${label}"
  fi

  local pattern match
  for attempt in 1 2; do
    for pattern in "$@"; do
      shopt -s nullglob
      for match in ${pattern}; do
        if [[ -e "${match}" ]]; then
          shopt -u nullglob
          printf '%s\n' "${match}"
          return 0
        fi
      done
      shopt -u nullglob
    done
    if (( attempt == 1 )); then
      try_hurry_attach "${label}" "${attach_pattern}" || break
    fi
  done

  fail_missing_port "${label}"
}

if [[ ! -f "${ROOT_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built yet. Run ./scripts/build_workspace.sh first." >&2
  exit 1
fi

SCOUT_PORT_RESOLVED="${SCOUT_PORT:-}"
MS42DC_PORT_RESOLVED="${MS42DC_PORT:-}"

if [[ "${USE_SCOUT}" == "true" && "${SCOUT_DRIVER}" =~ ^(waveshare|serial|usb_can_a|usb-can-a)$ ]]; then
  if [[ "${SHOW_ARGS}" == "true" && -z "${SCOUT_PORT_RESOLVED}" ]]; then
    SCOUT_PORT_RESOLVED="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
  else
    SCOUT_PORT_RESOLVED="$(
      resolve_port "${SCOUT_PORT_RESOLVED}" "SCOUT_PORT" "USB-SERIAL CH340|CH340" \
        "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0" \
        "/dev/serial/by-id/*USB_Serial-if00-port0" \
        "/dev/ttyUSB*"
    )"
  fi
fi

if [[ "${USE_MS42DC}" == "true" ]]; then
  if [[ "${SHOW_ARGS}" == "true" && -z "${MS42DC_PORT_RESOLVED}" ]]; then
    MS42DC_PORT_RESOLVED="/dev/serial/by-id/usb-1a86_USB_Single_Serial_58EB003416-if00"
  else
    MS42DC_PORT_RESOLVED="$(
      resolve_port "${MS42DC_PORT_RESOLVED}" "MS42DC_PORT" "USB-Enhanced-SERIAL CH9102|CH9102|USB_Single_Serial" \
        "/dev/motor_serial" \
        "/dev/serial/by-id/usb-1a86_USB_Single_Serial_58EB003416-if00" \
        "/dev/serial/by-id/*USB_Single_Serial*" \
        "/dev/ttyACM*" \
        "/dev/ttyCH*"
    )"
  fi
fi

if [[ "${USE_AUBO}" == "true" && "${SKIP_AUBO_CHECK}" != "true" ]]; then
  "${ARACHNE_SYSTEM_PYTHON}" "${ROOT_DIR}/scripts/real_aubo_prepare.py" --ip "${AUBO_ROBOT_IP}"
fi

if [[ "${USE_AUBO}" == "true" && "${AUBO_KEEP_TEACH_FLAG}" != "true" ]]; then
  rm -f "${AUBO_TEACH_FLAG_PATH}"
fi

echo "Arachne real bringup:"
echo "  Scout: ${USE_SCOUT} ${SCOUT_PORT_RESOLVED:+(${SCOUT_PORT_RESOLVED})}"
echo "  MS42DC: ${USE_MS42DC} ${MS42DC_PORT_RESOLVED:+(${MS42DC_PORT_RESOLVED})}"
echo "  Aubo: ${USE_AUBO} (${AUBO_ROBOT_IP})"

exec ros2 launch arachne_hardware real_bringup.launch.py \
  use_scout:="${USE_SCOUT}" \
  scout_driver:="${SCOUT_DRIVER}" \
  scout_port:="${SCOUT_PORT_RESOLVED:-${SCOUT_PORT:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0}}" \
  use_ms42dc:="${USE_MS42DC}" \
  ms42dc_driver:=direct \
  ms42dc_port:="${MS42DC_PORT_RESOLVED:-${MS42DC_PORT:-/dev/motor_serial}}" \
  use_aubo:="${USE_AUBO}" \
  aubo_robot_ip:="${AUBO_ROBOT_IP}" \
  "${EXTRA_ARGS[@]}"
