#!/usr/bin/env bash
set -euo pipefail

UBUNTU_CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME}")"
ARACHNE_SETUP_WITH_GAZEBO="${ARACHNE_SETUP_WITH_GAZEBO:-false}"
ARACHNE_SETUP_ROS_DESKTOP="${ARACHNE_SETUP_ROS_DESKTOP:-true}"

case "${ARACHNE_SETUP_WITH_GAZEBO}" in
  auto|true|false) ;;
  *)
    echo "ARACHNE_SETUP_WITH_GAZEBO must be auto, true, or false. Current value: ${ARACHNE_SETUP_WITH_GAZEBO}" >&2
    exit 1
    ;;
esac
case "${ARACHNE_SETUP_ROS_DESKTOP}" in
  true|false) ;;
  *)
    echo "ARACHNE_SETUP_ROS_DESKTOP must be true or false. Current value: ${ARACHNE_SETUP_ROS_DESKTOP}" >&2
    exit 1
    ;;
esac

if [[ -z "${ROS_DISTRO:-}" ]]; then
  case "${UBUNTU_CODENAME}" in
    jammy) ROS_DISTRO="humble" ;;
    noble) ROS_DISTRO="jazzy" ;;
    *)
      echo "Set ROS_DISTRO=humble or ROS_DISTRO=jazzy for ${UBUNTU_CODENAME}." >&2
      exit 1
      ;;
  esac
fi

if [[ "${ROS_DISTRO}" != "humble" && "${ROS_DISTRO}" != "jazzy" ]]; then
  echo "ROS_DISTRO must be humble or jazzy. Current value: ${ROS_DISTRO}" >&2
  exit 1
fi

if ! sudo -n true 2>/dev/null; then
  echo "This installer needs sudo to install apt packages." >&2
  echo "Run it in a terminal where sudo can prompt for your password, or pre-authenticate with: sudo -v" >&2
  sudo -v
fi

sudo apt-get update
sudo apt-get install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe -y

if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
fi

sudo apt-get update
ROS_PACKAGES=(
  "ros-${ROS_DISTRO}-rviz2"
  "ros-${ROS_DISTRO}-xacro"
  "ros-${ROS_DISTRO}-joint-state-publisher"
  "ros-${ROS_DISTRO}-joint-state-publisher-gui"
  "ros-${ROS_DISTRO}-robot-state-publisher"
  "ros-${ROS_DISTRO}-tf2-tools"
  "ros-${ROS_DISTRO}-joy"
  "ros-${ROS_DISTRO}-teleop-twist-joy"
  "ros-${ROS_DISTRO}-controller-manager"
  "ros-${ROS_DISTRO}-joint-state-broadcaster"
  "ros-${ROS_DISTRO}-joint-trajectory-controller"
  "ros-${ROS_DISTRO}-diff-drive-controller"
  "ros-${ROS_DISTRO}-forward-command-controller"
  "ros-${ROS_DISTRO}-position-controllers"
  "ros-${ROS_DISTRO}-moveit"
  "ros-${ROS_DISTRO}-nav2-bringup"
)
if [[ "${ARACHNE_SETUP_ROS_DESKTOP}" == "true" ]]; then
  ROS_PACKAGES=("ros-${ROS_DISTRO}-desktop" "${ROS_PACKAGES[@]}")
fi

apt_package_available() {
  local package="$1"
  apt-cache show "${package}" >/dev/null 2>&1
}

add_ros_package_if_available() {
  local package="$1"
  if apt_package_available "${package}"; then
    ROS_PACKAGES+=("${package}")
  fi
}

add_ros_package_if_available "ros-${ROS_DISTRO}-ros2-control"
add_ros_package_if_available "ros-${ROS_DISTRO}-ros2-controllers"

if [[ "${ARACHNE_SETUP_WITH_GAZEBO}" != "false" ]]; then
  GAZEBO_PACKAGES=(
    "ros-${ROS_DISTRO}-ros-gz-sim"
    "ros-${ROS_DISTRO}-ros-gz-bridge"
    "ros-${ROS_DISTRO}-gz-msgs-vendor"
    "ros-${ROS_DISTRO}-gz-transport-vendor"
  )
  add_ros_package_if_available "ros-${ROS_DISTRO}-ros-gz"
  add_ros_package_if_available "ros-${ROS_DISTRO}-gz-ros2-control"

  MISSING_GAZEBO_PACKAGES=()
  for package in "${GAZEBO_PACKAGES[@]}"; do
    if apt_package_available "${package}"; then
      ROS_PACKAGES+=("${package}")
    else
      MISSING_GAZEBO_PACKAGES+=("${package}")
    fi
  done

  if [[ ${#MISSING_GAZEBO_PACKAGES[@]} -gt 0 ]]; then
    if [[ "${ARACHNE_SETUP_WITH_GAZEBO}" == "true" ]]; then
      echo "Missing required Gazebo packages:" >&2
      printf '  %s\n' "${MISSING_GAZEBO_PACKAGES[@]}" >&2
      exit 1
    fi
    echo "Skipping unavailable optional Gazebo packages:"
    printf '  %s\n' "${MISSING_GAZEBO_PACKAGES[@]}"
  fi
fi

sudo apt-get install -y \
  "${ROS_PACKAGES[@]}" \
  python3-colcon-common-extensions \
  python3-tk \
  assimp-utils \
  can-utils \
  libasio-dev \
  usbutils \
  ros-dev-tools

echo "source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "Then run:"
echo "  source scripts/arachne_env.sh"
echo "  ./scripts/build_workspace.sh"
