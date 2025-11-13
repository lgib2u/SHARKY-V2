#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=bootstrap_env.sh
source "${SCRIPT_DIR}/bootstrap_env.sh"

require_cmd ssh
require_cmd scp

echo "Target: ${USER}@${HOST}"
wait_ssh

echo "Updating OS and installing prerequisites..."
remote_sudo "apt update && apt full-upgrade -y"
remote_sudo "apt install -y libcamera-apps dphys-swapfile \
  build-essential cmake git wget curl gnupg lsb-release \
  python3-pip python3-venv python3-colcon-common-extensions python3-vcstool \
  python3-rosdep python3-empy python3-numpy \
  libasio-dev libtinyxml2-dev libyaml-cpp-dev libssl-dev \
  libjpeg-dev libpng-dev network-manager"

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


