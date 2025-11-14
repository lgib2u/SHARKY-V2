#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=bootstrap_env.sh
source "${SCRIPT_DIR}/bootstrap_env.sh"

echo "Ubuntu 25.10: first-boot prep (local)..."

echo "Updating OS and installing prerequisites..."
remote_sudo "apt update && apt full-upgrade -y"
remote_sudo "apt install -y libcamera-apps libcamera-dev \
  build-essential cmake git wget curl gnupg lsb-release software-properties-common \
  python3-pip python3-venv python3-empy python3-numpy \
  python3-rosdep python3-vcstool python3-colcon-common-extensions \
  libasio-dev libtinyxml2-dev libyaml-cpp-dev libssl-dev \
  libjpeg-dev libpng-dev network-manager"

echo "Adding ROS 2 apt repository (rolling)..."
remote_sudo "mkdir -p /usr/share/keyrings"
remote_sudo "curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"
remote_sudo "sh -lc '. /etc/os-release && echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \${UBUNTU_CODENAME} main\" > /etc/apt/sources.list.d/ros2.list'"
remote_sudo "apt update"

echo "Installing ROS 2 rolling (ros-base)..."
remote_sudo "apt install -y ros-${ROS_DISTRO}-ros-base"

echo "Initializing rosdep..."
remote_sudo "rosdep init || true"
remote "rosdep update || true"

echo "Setting ROS environment defaults..."
remote "grep -q 'ROS_DOMAIN_ID' ~/.bashrc || echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}' >> ~/.bashrc"
remote "grep -q 'RMW_IMPLEMENTATION' ~/.bashrc || echo 'export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}' >> ~/.bashrc"
remote "grep -q '/opt/ros/${ROS_DISTRO}/setup.bash' ~/.bashrc || echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc"

echo "Done. Reboot is recommended."


