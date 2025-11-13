#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=bootstrap_env.sh
source "${SCRIPT_DIR}/bootstrap_env.sh"

require_cmd ssh

echo "Target: ${USER}@${HOST}"
wait_ssh

REMOTE_SCRIPT="/usr/local/sbin/sharky_boot_net.sh"
SERVICE_PATH="/etc/systemd/system/sharky_boot_net.service"

echo "Uploading boot-time network fallback script..."
remote "cat > /tmp/sharky_boot_net.sh <<'EOSH'
#!/usr/bin/env bash
set -euo pipefail

AP_SSID="${AP_SSID:-SHARKY}"
AP_PSK="${AP_PSK:-Luna21}"
WAIT_SECS="${WAIT_SECS:-60}"

LED_DIR=""
if [ -d /sys/class/leds/led0 ]; then
  LED_DIR="/sys/class/leds/led0"
elif [ -d /sys/class/leds/ACT ]; then
  LED_DIR="/sys/class/leds/ACT"
fi

led_trigger() {
  local trig="$1"
  [ -n "${LED_DIR}" ] && echo "${trig}" > "${LED_DIR}/trigger" 2>/dev/null || true
}

led_timer() {
  local on_ms="$1"; local off_ms="$2"
  if [ -n "${LED_DIR}" ]; then
    echo "timer" > "${LED_DIR}/trigger" 2>/dev/null || true
    echo "${on_ms}" > "${LED_DIR}/delay_on" 2>/dev/null || true
    echo "${off_ms}" > "${LED_DIR}/delay_off" 2>/dev/null || true
  fi
}

echo "Sharky boot net watchdog: waiting ${WAIT_SECS}s for SSH login..."
led_timer 100 100   # fast blink while waiting

# Ensure NetworkManager exists
if ! command -v nmcli >/dev/null 2>&1; then
  echo "NetworkManager not present; installing..."
  apt update && apt install -y network-manager
  systemctl enable --now NetworkManager
fi

# Wait window
sleep "${WAIT_SECS}"

echo "Checking for SSH accepted logins this boot..."
if journalctl -b -u ssh --no-pager | grep -qi 'Accepted '; then
  echo "SSH login detected; staying in client mode."
  led_trigger default-on
  exit 0
fi

echo "No SSH login detected; enabling AP '${AP_SSID}' ..."

# Create/modify AP connection
nmcli connection show "${AP_SSID}" >/dev/null 2>&1 || nmcli connection add type wifi ifname wlan0 con-name "${AP_SSID}" autoconnect yes ssid "${AP_SSID}"
nmcli connection modify "${AP_SSID}" 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared wifi-sec.key-mgmt wpa-psk wifi-sec.psk "${AP_PSK}"
nmcli connection up "${AP_SSID}" || nmcli device wifi hotspot ifname wlan0 ssid "${AP_SSID}" password "${AP_PSK}"

# Indicate AP mode with slower blink
led_timer 500 500
echo "AP enabled. SSID=${AP_SSID}"
exit 0
EOSH"
remote_sudo "mv /tmp/sharky_boot_net.sh ${REMOTE_SCRIPT} && chmod 755 ${REMOTE_SCRIPT}"

echo "Creating systemd service for boot watchdog..."
remote "cat > /tmp/sharky_boot_net.service <<'EOF'
[Unit]
Description=Sharky boot network watchdog (switch to AP if no SSH)
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
User=root
Environment=AP_SSID=${AP_SSID}
Environment=AP_PSK=${AP_PSK}
Environment=WAIT_SECS=60
ExecStart=${REMOTE_SCRIPT}

[Install]
WantedBy=multi-user.target
EOF"
remote_sudo "mv /tmp/sharky_boot_net.service ${SERVICE_PATH} && chown root:root ${SERVICE_PATH} && chmod 644 ${SERVICE_PATH}"
remote_sudo "systemctl daemon-reload && systemctl enable sharky_boot_net.service"

echo "Boot-time AP fallback with LED signaling configured."


