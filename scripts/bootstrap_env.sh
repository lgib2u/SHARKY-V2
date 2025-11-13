#!/usr/bin/env bash
set -euo pipefail

# Defaults (override by exporting env vars before running scripts)
HOST="${HOST:-sharky.local}"
USER="${USER:-lewis}"
PASS="${PASS:-Luna21}"

# ROS defaults
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# Wi-Fi AP defaults (use 2.4GHz-safe values; avoid spaces in SSID for simplicity)
AP_SSID="${AP_SSID:-SHARKY}"
AP_PSK="${AP_PSK:-Luna21}"

SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Missing required command: $1" >&2
    exit 1
  }
}

wait_ssh() {
  local tries=60
  echo "Waiting for SSH on ${USER}@${HOST} ..."
  until ssh -o ConnectTimeout=2 ${SSH_OPTS} "${USER}@${HOST}" true 2>/dev/null; do
    tries=$((tries-1))
    if [ "${tries}" -le 0 ]; then
      echo "SSH not reachable at ${HOST}" >&2
      exit 1
    fi
    sleep 2
  done
}

remote() {
  ssh ${SSH_OPTS} "${USER}@${HOST}" "$@"
}

remote_sudo() {
  # Run a command as root via sudo -S on the remote
  # Keep commands simple to avoid quoting pitfalls.
  ssh ${SSH_OPTS} "${USER}@${HOST}" "echo \"${PASS}\" | sudo -S bash -lc \"$*\""
}

copy_to() {
  # copy_to <local_path> <remote_path>
  scp ${SSH_OPTS} "$1" "${USER}@${HOST}:$2"
}


