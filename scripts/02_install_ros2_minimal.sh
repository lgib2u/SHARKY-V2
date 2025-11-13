#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=bootstrap_env.sh
source "${SCRIPT_DIR}/bootstrap_env.sh"

require_cmd ssh
require_cmd scp

echo "Target: ${USER}@${HOST}"
wait_ssh

echo "Creating ROS 2 workspace and fetching sources (Humble)..."
remote "mkdir -p ~/ros2_ws/src && cd ~/ros2_ws && [ -f ros2.repos ] || wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos"
remote "cd ~/ros2_ws && vcs import src < ros2.repos"

echo "Installing dependencies via rosdep..."
remote_sudo "apt update"
remote_sudo "rosdep update || true"
remote_sudo "rosdep install --from-paths /home/${USER}/ros2_ws/src --ignore-src -y --rosdistro humble"

echo "Building minimal ROS 2 core (this may take quite a while on Zero 2 W)..."
remote "cd ~/ros2_ws && colcon build \
  --merge-install --symlink-install \
  --packages-up-to rclcpp sensor_msgs image_transport \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --event-handlers console_direct+"

echo "Fetching and building camera_ros (libcamera driver)..."
remote "cd ~/ros2_ws/src && [ -d camera_ros ] || git clone https://github.com/christianrauch/camera_ros.git"
remote "cd ~/ros2_ws && colcon build \
  --merge-install --symlink-install \
  --packages-select camera_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --event-handlers console_direct+"

echo "Ensuring workspace is sourced on login..."
remote "grep -q 'ros2_ws/install/setup.bash' ~/.bashrc || echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc"

echo "ROS 2 minimal + libcamera_ros install completed."


