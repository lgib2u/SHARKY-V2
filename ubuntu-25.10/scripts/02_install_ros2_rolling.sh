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
# Ensure tooling exists (apt where available; fallback to pip)
remote_sudo "apt install -y python3-pip python3-venv || true"
remote_sudo "python3 -m ensurepip --upgrade || true"
remote "python3 -m pip --version || true"
remote_sudo "apt install -y python3-rosdep python3-vcstool python3-colcon-common-extensions || true"
remote_sudo "python3 -m pip install -U pip setuptools wheel --break-system-packages || true"
remote_sudo "command -v rosdep >/dev/null 2>&1 || python3 -m pip install rosdep --break-system-packages"
remote_sudo "command -v vcs >/dev/null 2>&1 || python3 -m pip install vcstool --break-system-packages"
remote_sudo "command -v colcon >/dev/null 2>&1 || python3 -m pip install colcon-common-extensions --break-system-packages"
remote_sudo "rosdep init || true"
remote "export PATH=/usr/local/bin:/home/${USER}/.local/bin:\$PATH; rosdep update || true"
remote "export PATH=/usr/local/bin:/home/${USER}/.local/bin:\$PATH; sudo rosdep install --from-paths /home/${USER}/ros2_ws/src --ignore-src -y --rosdistro ${ROS_DISTRO} --skip-keys 'fastcdr ignition-cmake2 ignition-math6 urdfdom_headers rti-connext-dds-6.0.1 python3-sip-dev python3-catkin-pkg-modules' || true"

echo "Building camera_ros (this may take a while)..."
remote "bash -lc 'source /opt/ros/${ROS_DISTRO}/setup.bash 2>/dev/null || true; export PATH=/usr/local/bin:/home/${USER}/.local/bin:\$PATH; cd ~/ros2_ws && colcon build \
  --merge-install --symlink-install \
  --packages-select camera_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --event-handlers console_direct+'"

echo "Ensuring workspace is sourced on login..."
remote "grep -q 'ros2_ws/install/setup.bash' ~/.bashrc || echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc"

echo "camera_ros installation complete."


