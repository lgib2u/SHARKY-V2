#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=bootstrap_env.sh
source "${SCRIPT_DIR}/bootstrap_env.sh"

echo "Enabling camera publisher service locally..."

SERVICE_TMP="/tmp/libcamera_ros.service"
SERVICE_PATH="/etc/systemd/system/libcamera_ros.service"

echo "Uploading systemd service to run libcamera_ros on boot..."
remote "cat > ${SERVICE_TMP} <<EOF
[Unit]
Description=ROS 2 libcamera publisher
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${USER}
WorkingDirectory=/home/${USER}
Environment=ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
Environment=RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
ExecStart=/bin/bash -lc 'source /home/${USER}/ros2_ws/install/setup.bash && exec ros2 run camera_ros camera_node --ros-args -p camera.frame_id:=camera_frame'
Restart=on-failure
RestartSec=3
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF"

remote_sudo "mv ${SERVICE_TMP} ${SERVICE_PATH} && chown root:root ${SERVICE_PATH} && chmod 644 ${SERVICE_PATH}"
remote_sudo "systemctl daemon-reload && systemctl enable --now libcamera_ros.service"

echo "Service enabled. Use:"
echo "  journalctl -u libcamera_ros -f"
echo "to follow logs on the Pi."


