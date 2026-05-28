#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STRICT=false

if [[ "${1:-}" == "--strict" ]]; then
  STRICT=true
fi

failures=0
warnings=0

ok() {
  printf '[OK] %s\n' "$1"
}

warn() {
  printf '[WARN] %s\n' "$1"
  warnings=$((warnings + 1))
}

fail_or_warn() {
  if [[ "${STRICT}" == "true" ]]; then
    printf '[FAIL] %s\n' "$1"
    failures=$((failures + 1))
  else
    warn "$1"
  fi
}

have_cmd() {
  command -v "$1" >/dev/null 2>&1
}

need_cmd() {
  if have_cmd "$1"; then
    ok "command available: $1"
  else
    fail_or_warn "missing command: $1"
  fi
}

is_wsl=false
if grep -qiE 'microsoft|wsl' /proc/version /proc/sys/kernel/osrelease 2>/dev/null; then
  is_wsl=true
fi

echo "Arachne real-hardware environment check"
echo "Workspace: ${ROOT_DIR}"
if [[ "${is_wsl}" == "true" ]]; then
  echo "Platform: WSL2"
else
  echo "Platform: native Linux"
fi
echo

echo "== ROS tools =="
need_cmd ros2
need_cmd colcon
need_cmd python3
if [[ -n "${ROS_DISTRO:-}" ]]; then
  ok "ROS_DISTRO=${ROS_DISTRO}"
else
  fail_or_warn "ROS_DISTRO is not set; run source /opt/ros/jazzy/setup.bash or /opt/ros/humble/setup.bash"
fi
echo

echo "== Vendor package links =="
for path in \
  "src/vendor/ugv_sdk" \
  "src/vendor/scout_base" \
  "src/vendor/scout_msgs" \
  "src/vendor/step_motor" \
  "src/vendor/serial" \
  "src/vendor/aubo_ros2_driver"
do
  if [[ -e "${ROOT_DIR}/${path}" ]]; then
    ok "${path}"
  else
    warn "${path} not found; run ./scripts/prepare_real_hardware_ros.sh"
  fi
done
echo

echo "== MS42DC serial =="
ms42dc_port="${MS42DC_PORT:-/dev/motor_serial}"
if [[ -e "${ms42dc_port}" ]]; then
  ok "MS42DC_PORT exists: ${ms42dc_port}"
else
  warn "MS42DC_PORT not found: ${ms42dc_port}"
fi

shopt -s nullglob
serial_candidates=(/dev/serial/by-id/* /dev/ttyUSB* /dev/ttyACM*)
if (( ${#serial_candidates[@]} > 0 )); then
  ok "serial candidates:"
  printf '  %s\n' "${serial_candidates[@]}"
else
  warn "no /dev/serial/by-id, /dev/ttyUSB, or /dev/ttyACM devices found"
fi
shopt -u nullglob

if [[ "${is_wsl}" == "true" ]]; then
  cat <<'EOF'
WSL2 serial note:
  USB serial devices must be attached from Windows before they appear in Linux.
  In an Administrator PowerShell:
    usbipd list
    usbipd bind --busid <BUSID>
    usbipd attach --wsl --busid <BUSID>
  Then in WSL2:
    lsusb
    ls /dev/ttyUSB* /dev/ttyACM*
EOF
fi
echo

echo "== Scout SocketCAN =="
can_iface="${SCOUT_CAN_IFACE:-can0}"
need_cmd ip
if ip link show "${can_iface}" >/dev/null 2>&1; then
  ok "CAN interface exists: ${can_iface}"
  if ip link show "${can_iface}" | grep -q "state UP"; then
    ok "${can_iface} is UP"
  else
    warn "${can_iface} exists but is not UP"
  fi
else
  warn "CAN interface not found: ${can_iface}"
fi

if have_cmd candump; then
  ok "can-utils available: candump"
else
  warn "can-utils not installed; install it with sudo apt install can-utils"
fi

if [[ "${is_wsl}" == "true" ]]; then
  cat <<'EOF'
WSL2 CAN note:
  Windows does not expose CAN devices to WSL2 automatically. Use a Linux-supported
  USB-CAN adapter through usbipd-win, then bring up SocketCAN inside WSL2:
    sudo modprobe can can_raw gs_usb
    sudo ip link set can0 up type can bitrate 500000
    candump can0
  If your WSL2 kernel lacks the required USB-CAN driver, run Scout control on
  native Linux or on an onboard Linux computer and bridge ROS over the network.
EOF
else
  cat <<'EOF'
Native Linux CAN note:
  For common gs_usb adapters:
    sudo modprobe gs_usb
    sudo ip link set can0 up type can bitrate 500000
    candump can0
EOF
fi
echo

echo "== Aubo TCP/IP =="
aubo_ip="${AUBO_ROBOT_IP:-192.168.127.128}"
aubo_port="${AUBO_PORT:-80}"
echo "Aubo target: ${aubo_ip}:${aubo_port}"
if timeout 1 bash -c "</dev/tcp/${aubo_ip}/${aubo_port}" >/dev/null 2>&1; then
  ok "Aubo TCP endpoint is reachable"
else
  warn "Aubo TCP endpoint is not reachable yet; verify robot IP, controller network, and firewall"
fi
echo

echo "Summary: ${warnings} warnings, ${failures} failures"
if (( failures > 0 )); then
  exit 1
fi
