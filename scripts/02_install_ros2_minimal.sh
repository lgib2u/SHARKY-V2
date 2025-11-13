#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=bootstrap_env.sh
source "${SCRIPT_DIR}/bootstrap_env.sh"

echo "Installing ROS 2 minimal locally..."

echo "Creating ROS 2 workspace and fetching sources (Humble)..."
remote "mkdir -p ~/ros2_ws/src && cd ~/ros2_ws && [ -f ros2.repos ] || wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos"
remote "cd ~/ros2_ws && vcs import src < ros2.repos"

echo "Installing dependencies via rosdep..."
remote_sudo "apt update"
remote_sudo "rosdep update || true"
# Ensure SIP is available even if python3-sip-dev is missing on Debian/RPi OS
remote_sudo "apt install -y python3-sip || true"
remote_sudo "python3 -m pip install sip --break-system-packages || true"
remote_sudo "rosdep install --from-paths /home/${USER}/ros2_ws/src --ignore-src -y --rosdistro humble --skip-keys 'fastcdr ignition-cmake2 ignition-math6 urdfdom_headers rti-connext-dds-6.0.1 python3-sip-dev'"

echo "Building minimal ROS 2 core (this may take quite a while on Zero 2 W)..."
remote "cd ~/ros2_ws && colcon build \
  --merge-install --symlink-install \
  --packages-up-to rclcpp sensor_msgs image_transport \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --event-handlers console_direct+"

echo "Fetching and building camera_ros (libcamera driver)..."
remote "cd ~/ros2_ws/src && [ -d camera_ros ] || git clone https://github.com/christianrauch/camera_ros.git"
echo "Installing camera_ros dependencies via rosdep..."
remote_sudo "rosdep install --from-paths /home/${USER}/ros2_ws/src/camera_ros --ignore-src -y --rosdistro humble --skip-keys 'fastcdr ignition-cmake2 ignition-math6 urdfdom_headers rti-connext-dds-6.0.1 python3-sip-dev'"
remote "cd ~/ros2_ws && colcon build \
  --merge-install --symlink-install \
  --packages-select camera_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --event-handlers console_direct+"

echo "Ensuring workspace is sourced on login..."
remote "grep -q 'ros2_ws/install/setup.bash' ~/.bashrc || echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc"

echo "ROS 2 minimal + libcamera_ros install completed."


