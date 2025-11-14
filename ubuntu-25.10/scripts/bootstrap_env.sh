#!/usr/bin/env bash
set -euo pipefail

# Defaults (override by exporting env vars before running scripts)
HOST="${HOST:-localhost}"
USER="${USER:-${USER:-ubuntu}}"

# ROS defaults
ROS_DISTRO="${ROS_DISTRO:-rolling}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# Wi-Fi AP defaults (use 2.4GHz-safe values; avoid spaces in SSID for simplicity)
AP_SSID="${AP_SSID:-SHARKY}"
AP_PSK="${AP_PSK:-bread164}"

wait_ssh() {
  # No-op in local-only mode
  return 0
}

remote() {
  # Execute locally using bash -lc to match login shell semantics
  bash -lc "$*"
}

remote_sudo() {
  # Run command as root locally
  sudo bash -lc "$*"
}

copy_to() {
  # copy_to <local_path> <remote_path>
  cp "$1" "$2"
}


