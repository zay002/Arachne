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
if [[ "$(command -v python3 2>/dev/null || true)" != "/usr/bin/python3" ]]; then
  fail_or_warn "python3 resolves to $(command -v python3); run 'source scripts/arachne_env.sh' before ROS commands"
fi
if /usr/bin/python3 -c 'import rclpy' >/dev/null 2>&1; then
  ok "system Python can import rclpy"
else
  fail_or_warn "/usr/bin/python3 cannot import rclpy; source ROS or install the selected ROS distro"
fi
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
serial_candidates=(/dev/serial/by-id/* /dev/ttyUSB* /dev/ttyACM* /dev/ttyCH*)
if (( ${#serial_candidates[@]} > 0 )); then
  ok "serial candidates:"
  printf '  %s\n' "${serial_candidates[@]}"
else
  warn "no /dev/serial/by-id, /dev/ttyUSB, /dev/ttyACM, or /dev/ttyCH devices found"
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
    ls /dev/ttyUSB* /dev/ttyACM* /dev/ttyCH*
EOF
fi
echo

echo "== Scout CAN =="
scout_driver="${SCOUT_DRIVER:-waveshare}"
scout_port="${SCOUT_PORT:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0}"
can_iface="${SCOUT_CAN_IFACE:-can0}"

case "${scout_driver}" in
  waveshare|serial|usb_can_a|usb-can-a)
    echo "Scout driver: Waveshare USB-CAN-A serial"
    if [[ -e "${scout_port}" ]]; then
      ok "SCOUT_PORT exists: ${scout_port}"
    else
      fail_or_warn "SCOUT_PORT not found: ${scout_port}"
    fi
    echo "Expected CAN settings: 500000 bit/s, standard 11-bit frames, Motorola payload byte order"
    if have_cmd hurry; then
      ok "hurry available for USB-CAN-A diagnostics"
    else
      warn "hurry not found; install hurry-porter for quick USB-CAN-A diagnostics"
    fi
    if [[ "${is_wsl}" == "true" ]]; then
      cat <<'EOF'
WSL2 Scout note:
  Waveshare USB-CAN-A appears as a CH340 serial device, not can0.
  Attach it from Windows with usbipd-win, then use the stable /dev/serial/by-id path.
  A quick bus check is:
    hurry waveshare-can-a configure --port "$SCOUT_PORT" --can-bitrate 500000 --frame-type standard
    hurry waveshare-can-a recv --port "$SCOUT_PORT" --duration 2
EOF
    fi
    ;;
  official|socketcan|scout_base)
    echo "Scout driver: AgileX scout_base over SocketCAN"
    need_cmd ip
    if ip link show "${can_iface}" >/dev/null 2>&1; then
      ok "CAN interface exists: ${can_iface}"
      if ip link show "${can_iface}" | grep -q "state UP"; then
        ok "${can_iface} is UP"
      else
        warn "${can_iface} exists but is not UP"
      fi
    else
      fail_or_warn "CAN interface not found: ${can_iface}"
    fi
    if have_cmd candump; then
      ok "can-utils available: candump"
    else
      warn "can-utils not installed; install it with sudo apt install can-utils"
    fi
    if [[ "${is_wsl}" == "true" ]]; then
      cat <<'EOF'
WSL2 SocketCAN note:
  SocketCAN USB adapters need WSL2 kernel support for the matching driver.
  If the adapter is a Waveshare USB-CAN-A CH340 serial device, use SCOUT_DRIVER=waveshare instead.
EOF
    else
      cat <<'EOF'
Native Linux SocketCAN note:
  For common gs_usb adapters:
    sudo modprobe gs_usb
    sudo ip link set can0 up type can bitrate 500000
    candump can0
EOF
    fi
    ;;
  *)
    warn "unknown SCOUT_DRIVER=${scout_driver}; expected waveshare or official"
    ;;
esac
echo

echo "== Aubo TCP/IP =="
aubo_ip="${AUBO_ROBOT_IP:-192.168.127.128}"
aubo_port="${AUBO_PORT:-80}"
aubo_mac_hint="${AUBO_MAC_HINT:-CC:82:7F:A3:E6:2E}"
echo "Aubo target: ${aubo_ip}:${aubo_port}"
echo "Aubo MAC hint: ${aubo_mac_hint}"
if timeout 1 bash -c "</dev/tcp/${aubo_ip}/${aubo_port}" >/dev/null 2>&1; then
  ok "Aubo TCP endpoint is reachable"
else
  warn "Aubo TCP endpoint is not reachable yet; verify robot IP, controller network, and firewall"
fi
if have_cmd ip && ip neigh show "${aubo_ip}" 2>/dev/null | grep -qi "${aubo_mac_hint}"; then
  ok "Aubo MAC hint matches neighbor table"
else
  warn "Aubo MAC hint not seen in neighbor table yet; this is expected before first successful network contact"
fi
echo

echo "Summary: ${warnings} warnings, ${failures} failures"
if (( failures > 0 )); then
  exit 1
fi
