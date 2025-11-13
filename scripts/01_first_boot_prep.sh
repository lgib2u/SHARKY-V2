#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=bootstrap_env.sh
source "${SCRIPT_DIR}/bootstrap_env.sh"

echo "Running first-boot prep locally..."

echo "Updating OS and installing prerequisites..."
remote_sudo "apt update && apt full-upgrade -y"
remote_sudo "apt install -y libcamera-apps dphys-swapfile \
  libcamera-dev \
  build-essential cmake git wget curl gnupg lsb-release \
  python3-pip python3-venv \
  python3-empy python3-numpy \
  libasio-dev libtinyxml2-dev libyaml-cpp-dev libssl-dev \
  libjpeg-dev libpng-dev network-manager"

echo "Ensuring Python tooling (pip, colcon, vcs, rosdep) is available..."
# Debian Bookworm on Raspberry Pi OS may not provide these via apt; use pip fallback
remote_sudo "python3 -m pip install -U pip setuptools wheel --break-system-packages || true"
remote_sudo "command -v colcon >/dev/null 2>&1 || python3 -m pip install colcon-common-extensions --break-system-packages"
remote_sudo "command -v vcs >/dev/null 2>&1 || python3 -m pip install vcstool --break-system-packages"
remote_sudo "command -v rosdep >/dev/null 2>&1 || python3 -m pip install rosdep --break-system-packages"

echo "Configuring swap to 2048 MB (temporary for builds)..."
remote_sudo "if grep -q '^CONF_SWAPSIZE=' /etc/dphys-swapfile; then \
  sed -i 's/^CONF_SWAPSIZE=.*/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile; \
else \
  echo 'CONF_SWAPSIZE=2048' >> /etc/dphys-swapfile; \
fi; systemctl restart dphys-swapfile"

echo "Setting ROS environment defaults..."
remote "grep -q 'ROS_DOMAIN_ID' ~/.bashrc || echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}' >> ~/.bashrc"
remote "grep -q 'RMW_IMPLEMENTATION' ~/.bashrc || echo 'export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}' >> ~/.bashrc"

echo "Initializing rosdep (idempotent)..."
remote_sudo "rosdep init || true"
remote "rosdep update || true"

echo "First-boot prep completed."


