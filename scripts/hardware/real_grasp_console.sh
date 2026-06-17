#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/load_local_env.sh"
arachne_load_local_env "${ROOT_DIR}"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_real_defaults.sh"
set -u

AUBO_ROBOT_IP="${AUBO_ROBOT_IP:-192.168.127.128}"
AUBO_TYPE="${AUBO_TYPE:-aubo_i5}"
WAIT_TIMEOUT_SEC="${ARACHNE_CONSOLE_WAIT_TIMEOUT_SEC:-${WAIT_TIMEOUT_SEC:-180}}"
RECORDING_DIR="${ARACHNE_TEACH_RECORDING_DIR:-${ROOT_DIR}/recordings/teach}"
AUBO_PAYLOAD_MASS="${ARACHNE_AUBO_PAYLOAD_MASS}"
AUBO_PAYLOAD_COG="${ARACHNE_AUBO_PAYLOAD_COG}"
AUBO_PAYLOAD_AOM="${ARACHNE_AUBO_PAYLOAD_AOM}"
AUBO_PAYLOAD_INERTIA="${ARACHNE_AUBO_PAYLOAD_INERTIA}"
USE_REMOTE_PLANNER="${ARACHNE_CONSOLE_USE_REMOTE_PLANNER:-false}"
REMOTE_LOCAL_PORT="${ARACHNE_REMOTE_LOCAL_PORT:-8767}"
REMOTE_PLANNER_URL="${ARACHNE_CONSOLE_REMOTE_PLANNER_URL:-${ARACHNE_REMOTE_PLANNER_URL:-http://127.0.0.1:${REMOTE_LOCAL_PORT}}}"
REMOTE_PLANNER_TIMEOUT="${ARACHNE_CONSOLE_REMOTE_PLANNER_TIMEOUT:-${ARACHNE_REMOTE_PLANNER_TIMEOUT:-20}}"
DEFAULT_SERVER_EXTRA_ARGS="--planner-backend local --planning-key-waypoints approach,grasp,safe_mid,basket_over --vertical-approach --no-lock-grasp-orientation --tool-orientation-limit-deg 45 --grasp-orientation-yaw-offsets-deg 0,15,-15,30,-30 --grasp-orientation-tilt-offsets-deg 0,8,-8 --real-gripper-require-capture --real-sdk-semantic-targets-only --real-sdk-max-targets 6"
if [[ "${USE_REMOTE_PLANNER}" == "true" && -n "${REMOTE_PLANNER_URL}" ]]; then
  DEFAULT_SERVER_EXTRA_ARGS="--planner-backend remote --remote-planner-url ${REMOTE_PLANNER_URL} --remote-planner-timeout ${REMOTE_PLANNER_TIMEOUT}"
fi
SERVER_EXTRA_ARGS="${ARACHNE_CONSOLE_SERVER_EXTRA_ARGS:-${DEFAULT_SERVER_EXTRA_ARGS}}"
TEACH_WITH_CAMERA="${ARACHNE_CONSOLE_TEACH_WITH_CAMERA:-false}"
TEACH_WITH_RVIZ="${ARACHNE_CONSOLE_TEACH_WITH_RVIZ:-true}"
TEACH_WITH_VISUALIZATION="${ARACHNE_CONSOLE_TEACH_WITH_VISUALIZATION:-true}"
TEACH_WITH_LSLIDAR_DRIVER="${ARACHNE_CONSOLE_TEACH_WITH_LSLIDAR_DRIVER:-true}"
TEACH_EXTRA_ARGS="${ARACHNE_CONSOLE_TEACH_EXTRA_ARGS:-}"
WITH_VIEWER="${ARACHNE_CONSOLE_WITH_VIEWER:-false}"
AUTO_AUBO_START="${ARACHNE_CONSOLE_AUTO_AUBO_START:-false}"
AUTO_BASE_GRIPPER="${ARACHNE_CONSOLE_AUTO_BASE_GRIPPER:-true}"
AUTO_CAMERA="${ARACHNE_CONSOLE_AUTO_CAMERA:-false}"
AUTO_GRASP_SERVER="${ARACHNE_CONSOLE_AUTO_GRASP_SERVER:-false}"
AUTO_LIDAR_NAV="${ARACHNE_CONSOLE_AUTO_LIDAR_NAV:-false}"
REQUIRE_ODOM="${ARACHNE_CONSOLE_REQUIRE_ODOM:-false}"
REQUIRE_CAMERA_TOPICS="${ARACHNE_CONSOLE_REQUIRE_CAMERA_TOPICS:-true}"
VIEWER_IMAGE_TOPIC="${ARACHNE_CONSOLE_VIEWER_IMAGE_TOPIC:-/camera/color/image_raw}"
CAMERA_COLOR_WIDTH="${ARACHNE_CONSOLE_CAMERA_COLOR_WIDTH:-320}"
CAMERA_COLOR_HEIGHT="${ARACHNE_CONSOLE_CAMERA_COLOR_HEIGHT:-240}"
CAMERA_COLOR_FPS="${ARACHNE_CONSOLE_CAMERA_COLOR_FPS:-30.0}"
CAMERA_DEPTH_WIDTH="${ARACHNE_CONSOLE_CAMERA_DEPTH_WIDTH:-640}"
CAMERA_DEPTH_HEIGHT="${ARACHNE_CONSOLE_CAMERA_DEPTH_HEIGHT:-480}"
CAMERA_DEPTH_FPS="${ARACHNE_CONSOLE_CAMERA_DEPTH_FPS:-5.0}"
RUN_ENV_CHECK=true
STOP_EXISTING=true
CONFIRM=false
QUICK=false
TERMINAL_KIND="${ARACHNE_CONSOLE_TERMINAL:-background}"

usage() {
  cat <<EOF
Usage:
  ./scripts/hardware/real_grasp_console.sh --yes [options]

Open the real operator console. By default this starts only the guarded Aubo
driver, Scout/MS42DC bringup, RViz visualization, and the teach panel. Camera,
2D viewer, Localization/Nav, grasp server, and Aubo power/start can be controlled from
the teach panel Runtime Services and Aubo buttons.

Options:
  -y, --yes             Confirm real hardware startup.
  --skip-env-check     Skip scripts/hardware/check_real_hardware_env.sh --strict.
  --no-stop-existing   Do not stop stale Arachne real stack processes first.
  --no-viewer          Do not open image_view for the camera image topic.
  --quick              Skip strict env check and do not block server startup on /odom.
  --terminal KIND      auto, gnome-terminal, xfce4-terminal, xterm, or background.
  -- ARGS...           Extra launch args forwarded to teach_panel.launch.py.
  -h, --help           Show this help.

Environment:
  AUBO_ROBOT_IP=${AUBO_ROBOT_IP}
  AUBO_TYPE=${AUBO_TYPE}
  WAIT_TIMEOUT_SEC=${WAIT_TIMEOUT_SEC}
  ARACHNE_CONSOLE_WAIT_TIMEOUT_SEC=${WAIT_TIMEOUT_SEC}
  ARACHNE_TEACH_RECORDING_DIR=${RECORDING_DIR}
  ARACHNE_CONSOLE_SERVER_EXTRA_ARGS=${SERVER_EXTRA_ARGS}
  ARACHNE_CONSOLE_USE_REMOTE_PLANNER=${USE_REMOTE_PLANNER}
  ARACHNE_CONSOLE_REMOTE_PLANNER_URL=${REMOTE_PLANNER_URL:-}
  ARACHNE_CONSOLE_REMOTE_PLANNER_TIMEOUT=${REMOTE_PLANNER_TIMEOUT}
  ARACHNE_CONSOLE_TEACH_WITH_CAMERA=${TEACH_WITH_CAMERA}
  ARACHNE_CONSOLE_TEACH_WITH_RVIZ=${TEACH_WITH_RVIZ}
  ARACHNE_CONSOLE_TEACH_WITH_LSLIDAR_DRIVER=${TEACH_WITH_LSLIDAR_DRIVER}
  ARACHNE_CONSOLE_TEACH_EXTRA_ARGS=${TEACH_EXTRA_ARGS}
  ARACHNE_CONSOLE_WITH_VIEWER=${WITH_VIEWER}
  ARACHNE_CONSOLE_AUTO_AUBO_START=${AUTO_AUBO_START}
  ARACHNE_CONSOLE_AUTO_BASE_GRIPPER=${AUTO_BASE_GRIPPER}
  ARACHNE_CONSOLE_AUTO_CAMERA=${AUTO_CAMERA}
  ARACHNE_CONSOLE_AUTO_GRASP_SERVER=${AUTO_GRASP_SERVER}
  ARACHNE_CONSOLE_AUTO_LIDAR_NAV=${AUTO_LIDAR_NAV}
  ARACHNE_CONSOLE_REQUIRE_ODOM=${REQUIRE_ODOM}
  ARACHNE_CONSOLE_REQUIRE_CAMERA_TOPICS=${REQUIRE_CAMERA_TOPICS}
  ARACHNE_CONSOLE_VIEWER_IMAGE_TOPIC=${VIEWER_IMAGE_TOPIC}
  ARACHNE_CONSOLE_CAMERA_COLOR_WIDTH=${CAMERA_COLOR_WIDTH}
  ARACHNE_CONSOLE_CAMERA_COLOR_HEIGHT=${CAMERA_COLOR_HEIGHT}
  ARACHNE_CONSOLE_CAMERA_DEPTH_WIDTH=${CAMERA_DEPTH_WIDTH}
  ARACHNE_CONSOLE_CAMERA_DEPTH_HEIGHT=${CAMERA_DEPTH_HEIGHT}

After startup, use the teach panel buttons:
  Runtime Services    -> start/stop camera, 2D viewer, Localization/Nav, grasp server
  Aubo On / Start     -> power on / startup the real arm when needed
  Visual Grasp        -> start camera/viewer/server, preflight, then grasp
  Grasp Start         -> /arachne/grasp_task/start only
  G Stop / Grasp Stop    -> /arachne/grasp_task/stop
  Restore                -> /arachne/grasp_task/restore
EOF
}

while (($#)); do
  case "$1" in
    -y|--yes)
      CONFIRM=true
      ;;
    --skip-env-check)
      RUN_ENV_CHECK=false
      ;;
    --no-stop-existing)
      STOP_EXISTING=false
      ;;
    --no-viewer)
      WITH_VIEWER=false
      ;;
    --quick)
      QUICK=true
      RUN_ENV_CHECK=false
      REQUIRE_ODOM=false
      WAIT_TIMEOUT_SEC="${ARACHNE_CONSOLE_QUICK_WAIT_TIMEOUT_SEC:-${WAIT_TIMEOUT_SEC}}"
      ;;
    --terminal)
      shift
      TERMINAL_KIND="${1:-auto}"
      ;;
    --)
      shift
      TEACH_EXTRA_ARGS="$*"
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[ "${ARACHNE_CONFIRM_REAL_GRASP_CONSOLE:-}" == "YES" ]]; then
  CONFIRM=true
fi

if [[ "${CONFIRM}" != "true" ]]; then
  cat >&2 <<'EOF'
Refusing to start the real grasp console without confirmation.

This opens controls that can power/start the real Aubo arm and command the
Scout base, Aubo arm, MS42DC gripper, and grasp task server. Confirm the
workspace is clear and an emergency stop / power cut is within reach, then run:

  ./scripts/hardware/real_grasp_console.sh --yes
EOF
  exit 2
fi

set +u
export ARACHNE_ENV_QUIET=1
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
unset ARACHNE_ENV_QUIET
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash"
fi
set -u

if [[ ! -f "${ROOT_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built yet. Run ./scripts/build/build_workspace.sh first." >&2
  exit 1
fi

timeout 3 ros2 daemon stop >/dev/null 2>&1 || true

LOG_DIR="${ROOT_DIR}/log/real_grasp_console/$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}" "${RECORDING_DIR}"
ln -sfn "${LOG_DIR}" "${ROOT_DIR}/log/real_grasp_console/latest"

q() {
  printf "%q" "$1"
}

run_logged() {
  local label="$1"
  local log_file="$2"
  shift 2
  echo "Running ${label}; log: ${log_file}"
  set +e
  "$@" 2>&1 | tee "${log_file}"
  local status=${PIPESTATUS[0]}
  set -e
  if [[ "${status}" != "0" ]]; then
    echo "${label} failed with exit code ${status}; see ${log_file}" >&2
    exit "${status}"
  fi
}

terminal_available() {
  case "$1" in
    gnome-terminal) command -v gnome-terminal >/dev/null 2>&1 ;;
    xfce4-terminal) command -v xfce4-terminal >/dev/null 2>&1 ;;
    xterm) command -v xterm >/dev/null 2>&1 ;;
    background) return 0 ;;
    *) return 1 ;;
  esac
}

choose_terminal() {
  if [[ "${TERMINAL_KIND}" != "auto" ]]; then
    if terminal_available "${TERMINAL_KIND}"; then
      echo "${TERMINAL_KIND}"
      return 0
    fi
    echo "Requested terminal is unavailable: ${TERMINAL_KIND}" >&2
    exit 2
  fi
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "background"
    return 0
  fi
  for candidate in gnome-terminal xfce4-terminal xterm; do
    if terminal_available "${candidate}"; then
      echo "${candidate}"
      return 0
    fi
  done
  echo "background"
}

TERMINAL_KIND="$(choose_terminal)"
BACKGROUND_PIDS=()

open_terminal() {
  local title="$1"
  local script="$2"
  local log_file="$3"
  local launcher_script="${LOG_DIR}/launcher_$(basename "${script}")"
  cat >"${launcher_script}" <<EOF
#!/usr/bin/env bash
set +e
echo "[${title}] starting"
echo "runner: ${script}"
echo "log: ${log_file}"
echo
bash $(q "${script}") 2>&1 | tee $(q "${log_file}")
status=\${PIPESTATUS[0]}
echo
echo "[${title}] exited with status \${status}"
echo "log: ${log_file}"
if [[ "\${ARACHNE_TERMINAL_HOLD:-true}" == "true" ]]; then
  exec bash
fi
exit "\${status}"
EOF
  chmod +x "${launcher_script}"
  case "${TERMINAL_KIND}" in
    gnome-terminal)
      gnome-terminal --title="${title}" -- bash "${launcher_script}"
      ;;
    xfce4-terminal)
      xfce4-terminal --title="${title}" --command="bash $(q "${launcher_script}")"
      ;;
    xterm)
      xterm -T "${title}" -e bash "${launcher_script}" &
      ;;
    background)
      if command -v setsid >/dev/null 2>&1; then
        ARACHNE_TERMINAL_HOLD=false setsid bash "${launcher_script}" </dev/null >/dev/null 2>&1 &
      else
        ARACHNE_TERMINAL_HOLD=false nohup bash "${launcher_script}" </dev/null >/dev/null 2>&1 &
      fi
      BACKGROUND_PIDS+=("$!")
      echo "Started ${title} in background; log: ${log_file}"
      ;;
  esac
}

write_runner() {
  local name="$1"
  local body="$2"
  local script="${LOG_DIR}/${name}.sh"
  cat >"${script}" <<EOF
#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR=$(q "${ROOT_DIR}")
AUBO_ROBOT_IP=$(q "${AUBO_ROBOT_IP}")
AUBO_TYPE=$(q "${AUBO_TYPE}")
WAIT_TIMEOUT_SEC=$(q "${WAIT_TIMEOUT_SEC}")
RECORDING_DIR=$(q "${RECORDING_DIR}")
AUBO_PAYLOAD_MASS=$(q "${AUBO_PAYLOAD_MASS}")
AUBO_PAYLOAD_COG=$(q "${AUBO_PAYLOAD_COG}")
AUBO_PAYLOAD_AOM=$(q "${AUBO_PAYLOAD_AOM}")
AUBO_PAYLOAD_INERTIA=$(q "${AUBO_PAYLOAD_INERTIA}")
SERVER_EXTRA_ARGS=$(q "${SERVER_EXTRA_ARGS}")
TEACH_WITH_CAMERA=$(q "${TEACH_WITH_CAMERA}")
TEACH_WITH_RVIZ=$(q "${TEACH_WITH_RVIZ}")
TEACH_WITH_VISUALIZATION=$(q "${TEACH_WITH_VISUALIZATION}")
TEACH_WITH_LSLIDAR_DRIVER=$(q "${TEACH_WITH_LSLIDAR_DRIVER}")
TEACH_EXTRA_ARGS=$(q "${TEACH_EXTRA_ARGS}")
REQUIRE_ODOM=$(q "${REQUIRE_ODOM}")
REQUIRE_CAMERA_TOPICS=$(q "${REQUIRE_CAMERA_TOPICS}")
VIEWER_IMAGE_TOPIC=$(q "${VIEWER_IMAGE_TOPIC}")
CAMERA_COLOR_WIDTH=$(q "${CAMERA_COLOR_WIDTH}")
CAMERA_COLOR_HEIGHT=$(q "${CAMERA_COLOR_HEIGHT}")
CAMERA_COLOR_FPS=$(q "${CAMERA_COLOR_FPS}")
CAMERA_DEPTH_WIDTH=$(q "${CAMERA_DEPTH_WIDTH}")
CAMERA_DEPTH_HEIGHT=$(q "${CAMERA_DEPTH_HEIGHT}")
CAMERA_DEPTH_FPS=$(q "${CAMERA_DEPTH_FPS}")
cd "\${ROOT_DIR}"
set +u
source "\${ROOT_DIR}/scripts/env/arachne_env.sh"
source "\${ROOT_DIR}/install/setup.bash"
source "\${ROOT_DIR}/scripts/env/arachne_real_defaults.sh"
set -u
refresh_ros2_cli_daemon() {
  timeout 3 ros2 daemon stop >/dev/null 2>&1 || true
}
refresh_ros2_cli_daemon
wait_for_topic() {
  local topic="\$1"
  local label="\$2"
  local deadline=\$((SECONDS + WAIT_TIMEOUT_SEC))
  local info
  while (( WAIT_TIMEOUT_SEC <= 0 || SECONDS < deadline )); do
    info="\$(timeout 3 ros2 topic info --no-daemon "\${topic}" 2>/dev/null || true)"
    if grep -Eq '^Publisher count: [1-9][0-9]*$' <<<"\${info}"; then
      echo "ready: \${label} (\${topic})"
      return 0
    fi
    sleep 1
  done
  echo "Timed out after \${WAIT_TIMEOUT_SEC}s waiting for \${label} (\${topic})." >&2
  exit 1
}
wait_for_service() {
  local service="\$1"
  local label="\$2"
  local deadline=\$((SECONDS + WAIT_TIMEOUT_SEC))
  while (( WAIT_TIMEOUT_SEC <= 0 || SECONDS < deadline )); do
    if timeout 3 ros2 service list --no-daemon 2>/dev/null | grep -qx "\${service}"; then
      echo "ready: \${label} (\${service})"
      return 0
    fi
    sleep 1
  done
  echo "Timed out after \${WAIT_TIMEOUT_SEC}s waiting for \${label} (\${service})." >&2
  exit 1
}
wait_for_controller_active() {
  local controller="\$1"
  local label="\$2"
  local deadline=\$((SECONDS + WAIT_TIMEOUT_SEC))
  local output
  local clean_output
  while (( WAIT_TIMEOUT_SEC <= 0 || SECONDS < deadline )); do
    refresh_ros2_cli_daemon
    output="\$(timeout 3 ros2 control list_controllers 2>/dev/null || true)"
    clean_output="\$(python3 -c 'import re,sys; print(re.sub("\\\\x1b\\\\[[0-9;]*m", "", sys.stdin.read()), end="")' <<<"\${output}")"
    if awk '{print \$1, \$NF}' <<<"\${clean_output}" | grep -qx "\${controller} active"; then
      echo "ready: \${label} (\${controller})"
      return 0
    fi
    sleep 1
  done
  echo "Timed out after \${WAIT_TIMEOUT_SEC}s waiting for \${label} (\${controller})." >&2
  refresh_ros2_cli_daemon
  timeout 3 ros2 control list_controllers 2>/dev/null || true
  exit 1
}
EOF
  printf "%s\n" "${body}" >>"${script}"
  chmod +x "${script}"
  printf "%s\n" "${script}"
}

cleanup() {
  if ((${#BACKGROUND_PIDS[@]})); then
    for pid in "${BACKGROUND_PIDS[@]}"; do
      kill -INT "${pid}" 2>/dev/null || true
    done
  fi
}
trap cleanup INT TERM

echo "Arachne real grasp console"
echo "  workspace: ${ROOT_DIR}"
echo "  terminal: ${TERMINAL_KIND}"
echo "  logs: ${LOG_DIR}"
echo "  Aubo: ${AUBO_ROBOT_IP} / ${AUBO_TYPE}"
echo "  server extra args: ${SERVER_EXTRA_ARGS}"
echo "  remote planner: ${REMOTE_PLANNER_URL:-disabled}"
echo "  quick mode: ${QUICK}"
echo "  teach extra args: ${TEACH_EXTRA_ARGS:-none}"
echo "  auto Aubo startup: ${AUTO_AUBO_START}"
echo "  auto base+gripper: ${AUTO_BASE_GRIPPER}"
echo "  auto camera: ${AUTO_CAMERA}"
echo "  auto Localization/Nav: ${AUTO_LIDAR_NAV}"
echo "  auto grasp server: ${AUTO_GRASP_SERVER}"
echo "  auto 2D viewer: ${WITH_VIEWER}"
echo "  require odom before server: ${REQUIRE_ODOM}"
echo "  require camera topics in preflight: ${REQUIRE_CAMERA_TOPICS}"

cd "${ROOT_DIR}"

if [[ "${STOP_EXISTING}" == "true" ]]; then
  run_logged "stop existing real stack" \
    "${LOG_DIR}/00_stop_existing_real_stack.log" \
    "${ROOT_DIR}/scripts/hardware/stop_real_stack.sh"
fi

if [[ "${RUN_ENV_CHECK}" == "true" ]]; then
  run_logged "real-hardware environment check" \
    "${LOG_DIR}/01_check_real_hardware_env.log" \
    "${ROOT_DIR}/scripts/hardware/check_real_hardware_env.sh" --strict
fi

aubo_driver_script="$(write_runner "10_aubo_driver" '
echo "Starting Aubo ROS2 driver in guarded prestart mode..."
exec env ARACHNE_CONFIRM_AUBO_DRIVER=YES \
  ARACHNE_AUBO_ALLOW_PRESTART=YES \
  AUBO_ROBOT_IP="${AUBO_ROBOT_IP}" \
  AUBO_TYPE="${AUBO_TYPE}" \
  "${ROOT_DIR}/scripts/hardware/real_aubo_bringup.sh"
')"

aubo_start_script="$(write_runner "20_aubo_remote_start" '
echo "Waiting briefly for Aubo driver..."
sleep 4
echo "Configuring Aubo payload..."
env ARACHNE_CONFIRM_AUBO_PAYLOAD=YES \
  AUBO_ROBOT_IP="${AUBO_ROBOT_IP}" \
  "${ARACHNE_SYSTEM_PYTHON}" "${ROOT_DIR}/scripts/hardware/real_aubo_payload.py" \
  --mass "${AUBO_PAYLOAD_MASS}" \
  --cog "${AUBO_PAYLOAD_COG}" \
  --aom "${AUBO_PAYLOAD_AOM}" \
  --inertia "${AUBO_PAYLOAD_INERTIA}"
echo "Running guarded Aubo remote startup..."
env ARACHNE_CONFIRM_AUBO_REMOTE_START=YES \
  AUBO_ROBOT_IP="${AUBO_ROBOT_IP}" \
  "${ROOT_DIR}/scripts/hardware/real_aubo_remote_start.sh"
echo "Checking Aubo running/safety state..."
"${ARACHNE_SYSTEM_PYTHON}" "${ROOT_DIR}/scripts/hardware/real_aubo_prepare.py" --ip "${AUBO_ROBOT_IP}"
wait_for_topic "/joint_states" "Aubo joint states"
wait_for_controller_active "forward_command_controller_velocity" "Aubo velocity controller"
echo "Aubo remote startup ready."
')"

base_script="$(write_runner "30_base_gripper_bringup" '
echo "Starting Scout + MS42DC bringup..."
exec "${ROOT_DIR}/scripts/hardware/real_bringup.sh" --no-aubo
')"

camera_script="$(write_runner "35_gemini_camera" '
echo "Starting Gemini335 camera..."
exec ros2 launch arachne_sensors gemini335.launch.py \
  publish_pointcloud:=false \
  with_color_view:=false \
  with_depth_view:=false \
  with_tf:=true \
  camera_parent_frame:=ee_camera_link \
  "color_width:=${CAMERA_COLOR_WIDTH}" \
  "color_height:=${CAMERA_COLOR_HEIGHT}" \
  "color_fps:=${CAMERA_COLOR_FPS}" \
  "depth_width:=${CAMERA_DEPTH_WIDTH}" \
  "depth_height:=${CAMERA_DEPTH_HEIGHT}" \
  "depth_fps:=${CAMERA_DEPTH_FPS}"
')"

server_script="$(write_runner "40_grasp_task_server" '
wait_for_topic "/joint_states" "Aubo joint states"
if [[ "${REQUIRE_ODOM}" == "true" ]]; then
  wait_for_topic "/odom" "Scout odometry"
else
  echo "Skipping blocking wait for /odom (ARACHNE_CONSOLE_REQUIRE_ODOM=false)."
fi
wait_for_topic "/arachne/hardware/gripper_status" "MS42DC status"
wait_for_topic "/camera/color/image_raw" "Gemini335 color image"
wait_for_topic "/camera/depth/image_raw" "Gemini335 depth image"
echo "Starting grasp task server..."
exec env \
  ARACHNE_GRASP_START_CAMERA=false \
  ARACHNE_GRASP_CAMERA_COLOR_WIDTH="${CAMERA_COLOR_WIDTH}" \
  ARACHNE_GRASP_CAMERA_COLOR_HEIGHT="${CAMERA_COLOR_HEIGHT}" \
  ARACHNE_GRASP_CAMERA_COLOR_FPS="${CAMERA_COLOR_FPS}" \
  ARACHNE_GRASP_CAMERA_DEPTH_WIDTH="${CAMERA_DEPTH_WIDTH}" \
  ARACHNE_GRASP_CAMERA_DEPTH_HEIGHT="${CAMERA_DEPTH_HEIGHT}" \
  ARACHNE_GRASP_CAMERA_DEPTH_FPS="${CAMERA_DEPTH_FPS}" \
  "${ROOT_DIR}/scripts/vision/grasp_task_server.sh" \
  execute_real:=true \
  confirm_execute_real:=true \
  with_rviz:=false \
  preview_on_start:=false \
  planning_recovery_base_enabled:=false \
  require_odom:="${REQUIRE_ODOM}" \
  require_camera_topics:="${REQUIRE_CAMERA_TOPICS}" \
  require_aubo_status:=false \
  require_gripper_status:=false \
  max_grasp_attempts:=3 \
  retry_on_gripper_miss:=true \
  extra_args:="${SERVER_EXTRA_ARGS}"
')"

teach_script="$(write_runner "50_teach_panel" '
echo "Starting teach panel..."
exec ros2 launch arachne_operator teach_panel.launch.py \
  with_camera:="${TEACH_WITH_CAMERA}" \
  with_visualization:="${TEACH_WITH_VISUALIZATION}" \
  visualization_with_rviz:="${TEACH_WITH_RVIZ}" \
  visualization_with_lslidar_driver:="${TEACH_WITH_LSLIDAR_DRIVER}" \
  recording_dir:="${RECORDING_DIR}" \
  workspace_root:="${ROOT_DIR}" \
  runtime_log_root:="log/teach_panel" \
  camera_command:="ros2 launch arachne_sensors gemini335.launch.py publish_pointcloud:=false with_color_view:=false with_depth_view:=false with_tf:=true camera_parent_frame:=ee_camera_link projection_flip_x:=true projection_flip_y:=true color_width:=${CAMERA_COLOR_WIDTH} color_height:=${CAMERA_COLOR_HEIGHT} color_fps:=${CAMERA_COLOR_FPS} depth_width:=${CAMERA_DEPTH_WIDTH} depth_height:=${CAMERA_DEPTH_HEIGHT} depth_fps:=${CAMERA_DEPTH_FPS}" \
  camera_view_command:="\${ARACHNE_SYSTEM_PYTHON:-python3} scripts/vision/raw_image_viewer.py --topic ${VIEWER_IMAGE_TOPIC} --window \"Arachne Raw Camera\" --max-fps 15" \
  slam_command:="ARACHNE_NAV_WITH_LSLIDAR_DRIVER=false scripts/hardware/real_lidar_nav.sh" \
  grasp_server_command:="scripts/vision/grasp_task_server.sh execute_real:=true confirm_execute_real:=true with_rviz:=false preview_on_start:=false planning_recovery_base_enabled:=false require_odom:=${REQUIRE_ODOM} require_camera_topics:=${REQUIRE_CAMERA_TOPICS} require_aubo_status:=false require_gripper_status:=false max_grasp_attempts:=3 retry_on_gripper_miss:=true extra_args:=\"${SERVER_EXTRA_ARGS}\"" \
  ${TEACH_EXTRA_ARGS}
')"

nav_script="$(write_runner "45_lidar_nav" '
echo "Starting lidar localization/Nav..."
exec env ARACHNE_NAV_WITH_LSLIDAR_DRIVER=false "${ROOT_DIR}/scripts/hardware/real_lidar_nav.sh"
')"

viewer_script="$(write_runner "60_grasp_viewer" '
echo "Opening raw 2D camera view: ${VIEWER_IMAGE_TOPIC}"
wait_for_topic "${VIEWER_IMAGE_TOPIC}" "raw camera image"
exec "${ARACHNE_SYSTEM_PYTHON}" "${ROOT_DIR}/scripts/vision/raw_image_viewer.py" \
  --topic "${VIEWER_IMAGE_TOPIC}" \
  --window "Arachne Raw Camera" \
  --max-fps 15
')"

open_terminal "Arachne Teach Panel" "${teach_script}" "${LOG_DIR}/50_teach_panel.log"
open_terminal "Arachne Aubo Driver" "${aubo_driver_script}" "${LOG_DIR}/10_aubo_driver.log"
if [[ "${AUTO_AUBO_START}" == "true" ]]; then
  open_terminal "Arachne Aubo Remote Start" "${aubo_start_script}" "${LOG_DIR}/20_aubo_remote_start.log"
fi
if [[ "${AUTO_BASE_GRIPPER}" == "true" ]]; then
  open_terminal "Arachne Base + Gripper" "${base_script}" "${LOG_DIR}/30_base_gripper_bringup.log"
fi
if [[ "${AUTO_CAMERA}" == "true" ]]; then
  open_terminal "Arachne Gemini Camera" "${camera_script}" "${LOG_DIR}/35_gemini_camera.log"
fi
if [[ "${AUTO_LIDAR_NAV}" == "true" ]]; then
  open_terminal "Arachne Lidar Localization/Nav" "${nav_script}" "${LOG_DIR}/45_lidar_nav.log"
fi
if [[ "${AUTO_GRASP_SERVER}" == "true" ]]; then
  open_terminal "Arachne Grasp Server" "${server_script}" "${LOG_DIR}/40_grasp_task_server.log"
fi
if [[ "${WITH_VIEWER}" == "true" ]]; then
  open_terminal "Arachne 2D Grasp View" "${viewer_script}" "${LOG_DIR}/60_grasp_viewer.log"
fi

cat <<EOF

Console launched.
Logs and generated runner scripts:
  ${LOG_DIR}

Teach panel grasp controls:
  Runtime Services     -> start/stop camera, 2D viewer, Localization/Nav, grasp server
  Aubo On / Aubo Start -> power on / startup the real arm
  Visual Grasp         -> start camera/viewer/server, preflight, then grasp
  Grasp Start          -> start one grasp task only
  G Stop / Grasp Stop    -> stop current grasp task
  Restore                -> restore recorded base recovery motion

To stop all real-stack processes later:
  ./scripts/hardware/stop_real_stack.sh
EOF
