#!/usr/bin/env bash
set -euo pipefail

UBUNTU_CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME}")"

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
sudo apt-get install -y \
  "ros-${ROS_DISTRO}-desktop" \
  "ros-${ROS_DISTRO}-xacro" \
  "ros-${ROS_DISTRO}-joint-state-publisher" \
  "ros-${ROS_DISTRO}-joint-state-publisher-gui" \
  "ros-${ROS_DISTRO}-robot-state-publisher" \
  "ros-${ROS_DISTRO}-tf2-tools" \
  python3-colcon-common-extensions \
  python3-tk \
  ros-dev-tools

echo "source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "Then run: colcon build --symlink-install"
