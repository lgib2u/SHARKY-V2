#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=bootstrap_env.sh
source "${SCRIPT_DIR}/bootstrap_env.sh"

echo "Ubuntu 25.10: configuring Wi‑Fi hotspot (AP) locally..."

echo "Ensuring NetworkManager is installed and active..."
remote_sudo "apt update && apt install -y network-manager"
remote_sudo "systemctl enable --now NetworkManager"

echo "Configuring Wi‑Fi hotspot via NetworkManager..."
remote_sudo "nmcli connection show '${AP_SSID}' >/dev/null 2>&1 || nmcli connection add type wifi ifname wlan0 con-name '${AP_SSID}' autoconnect yes ssid '${AP_SSID}'"
remote_sudo "nmcli connection modify '${AP_SSID}' 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared wifi-sec.key-mgmt wpa-psk wifi-sec.psk '${AP_PSK}'"
remote_sudo "nmcli connection up '${AP_SSID}' || nmcli device wifi hotspot ifname wlan0 ssid '${AP_SSID}' password '${AP_PSK}'"

echo "Hotspot configured:"
echo "  SSID: ${AP_SSID}"
echo "  PSK : ${AP_PSK}"
echo "Note: Clients will receive NATed internet (ipv4.method=shared) when the Pi has upstream connectivity."


