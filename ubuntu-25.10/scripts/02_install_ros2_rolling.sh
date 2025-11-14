#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=bootstrap_env.sh
source "${SCRIPT_DIR}/bootstrap_env.sh"

echo "Ubuntu 25.10: install/build camera_ros on ROS 2 ${ROS_DISTRO}..."

echo "Preparing workspace and fetching sources..."
remote "mkdir -p ~/ros2_ws/src"
remote "cd ~/ros2_ws/src && [ -d camera_ros ] || git clone https://github.com/christianrauch/camera_ros.git"

echo "Installing dependencies via rosdep..."
remote_sudo "apt update"
remote_sudo "rosdep update || true"
remote_sudo "rosdep install --from-paths /home/${USER}/ros2_ws/src --ignore-src -y --rosdistro ${ROS_DISTRO} \
  --skip-keys 'fastcdr ignition-cmake2 ignition-math6 urdfdom_headers rti-connext-dds-6.0.1 python3-sip-dev python3-catkin-pkg-modules' || true"

echo "Building camera_ros (this may take a while)..."
remote "cd ~/ros2_ws && colcon build \
  --merge-install --symlink-install \
  --packages-select camera_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --event-handlers console_direct+"

echo "Ensuring workspace is sourced on login..."
remote "grep -q 'ros2_ws/install/setup.bash' ~/.bashrc || echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc"

echo "camera_ros installation complete."


