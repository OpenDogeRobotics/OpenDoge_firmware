#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPTS_DIR="${ROOT_DIR}/scripts"

SYSTEMD_USER_DIR="${HOME}/.config/systemd/user"
SYSTEMD_SYSTEM_DIR="/etc/systemd/system"

CAN_SERVICE="opendoge-can.service"
XBOXDRV_SERVICE="opendoge-xboxdrv.service"
IMU_SERVICE="opendoge-imu.service"
JOYSTICK_SERVICE="opendoge-joystick.service"

echo "=== OpenDoge Service Installer ==="
echo ""

# ═══════════════════════════════════════════════════════════════════
# Step 1: CAN interfaces (system oneshot)
# ═══════════════════════════════════════════════════════════════════
echo "[1/5] Installing CAN interface setup (system oneshot) ..."

if command -v sudo &>/dev/null; then
    sudo cp "${SCRIPTS_DIR}/${CAN_SERVICE}" "${SYSTEMD_SYSTEM_DIR}/${CAN_SERVICE}"
    sudo systemctl daemon-reload
    sudo systemctl enable "${CAN_SERVICE}"
    echo "      Installed → ${SYSTEMD_SYSTEM_DIR}/${CAN_SERVICE}"
    echo "      Starting ..."
    sudo systemctl restart "${CAN_SERVICE}" || echo "      [WARN] CAN setup failed (USB-CAN adapters not plugged in?)"
else
    echo "      [SKIP] sudo not available"
    exit 1
fi

# ═══════════════════════════════════════════════════════════════════
# Step 2: xboxdrv (system service)
# ═══════════════════════════════════════════════════════════════════
echo ""
echo "[2/5] Installing xboxdrv driver (system service) ..."

if ! command -v xboxdrv &>/dev/null; then
    echo "      [WARN] xboxdrv not found at /usr/bin/xboxdrv"
    echo "      Install with: sudo apt install xboxdrv"
fi

sudo cp "${SCRIPTS_DIR}/${XBOXDRV_SERVICE}" "${SYSTEMD_SYSTEM_DIR}/${XBOXDRV_SERVICE}"
sudo systemctl daemon-reload
sudo systemctl enable "${XBOXDRV_SERVICE}"
echo "      Installed → ${SYSTEMD_SYSTEM_DIR}/${XBOXDRV_SERVICE}"
echo "      Starting ..."
sudo systemctl restart "${XBOXDRV_SERVICE}" || echo "      [WARN] xboxdrv failed to start (dongle not plugged in?)"

# ═══════════════════════════════════════════════════════════════════
# Step 3: Wait for js0 (needed by joystick bridge)
# ═══════════════════════════════════════════════════════════════════
echo ""
echo "[3/5] Waiting for /dev/input/js0 (up to 15s) ..."
for i in $(seq 1 30); do
    if [ -e /dev/input/js0 ]; then
        echo "      /dev/input/js0 ready ($(cat /sys/class/input/js0/device/name 2>/dev/null))"
        break
    fi
    sleep 0.5
done
if [ ! -e /dev/input/js0 ]; then
    echo "      [WARN] /dev/input/js0 did not appear — check:"
    echo "        sudo systemctl status opendoge-xboxdrv"
    echo "        lsusb | grep 413d"
fi

# ═══════════════════════════════════════════════════════════════════
# Step 4: IMU bridge (user service)
# ═══════════════════════════════════════════════════════════════════
echo ""
echo "[4/5] Installing DM-IMU-L1 bridge (user service) ..."
mkdir -p "${SYSTEMD_USER_DIR}"
cp "${SCRIPTS_DIR}/${IMU_SERVICE}" "${SYSTEMD_USER_DIR}/${IMU_SERVICE}"
systemctl --user daemon-reload
systemctl --user enable "${IMU_SERVICE}"
echo "      Installed → ${SYSTEMD_USER_DIR}/${IMU_SERVICE}"
echo "      Starting ..."
systemctl --user restart "${IMU_SERVICE}" || echo "      [WARN] IMU bridge failed to start (IMU not plugged in?)"

# ═══════════════════════════════════════════════════════════════════
# Step 5: Joystick bridge (user service)
# ═══════════════════════════════════════════════════════════════════
echo ""
echo "[5/5] Installing joystick bridge (user service) ..."
cp "${SCRIPTS_DIR}/${JOYSTICK_SERVICE}" "${SYSTEMD_USER_DIR}/${JOYSTICK_SERVICE}"
systemctl --user daemon-reload
systemctl --user enable "${JOYSTICK_SERVICE}"
systemctl --user restart "${JOYSTICK_SERVICE}" || echo "      [WARN] joystick bridge failed to start (no js0?)"
echo "      Installed → ${SYSTEMD_USER_DIR}/${JOYSTICK_SERVICE}"

# ═══════════════════════════════════════════════════════════════════
# Enable lingering so user services start at boot
# ═══════════════════════════════════════════════════════════════════
if command -v loginctl &>/dev/null; then
    loginctl enable-linger "$(whoami)" 2>/dev/null || true
    echo ""
    echo "      Lingering enabled for $(whoami)"
fi

# ═══════════════════════════════════════════════════════════════════
# Summary
# ═══════════════════════════════════════════════════════════════════
echo ""
echo "=== Installation Complete ==="
echo ""
echo "System services (start at boot, require sudo to manage):"
echo "  systemctl status opendoge-can              # CAN interfaces (can0-3)"
echo "  systemctl status opendoge-xboxdrv          # Xbox controller driver"
echo ""
echo "User services (start at boot via lingering):"
echo "  systemctl --user status opendoge-imu       # DM-IMU-L1 bridge"
echo "  systemctl --user status opendoge-joystick  # Joystick bridge"
echo ""
echo "Logs:"
echo "  journalctl -u opendoge-can -f              # CAN setup"
echo "  journalctl -u opendoge-xboxdrv -f          # xboxdrv"
echo "  journalctl --user -u opendoge-imu -f       # IMU bridge"
echo "  journalctl --user -u opendoge-joystick -f  # Joystick bridge"
echo ""
echo "Verify:"
echo "  cat /tmp/opendoge_imu.state                # IMU data"
echo "  watch -n 0.2 cat /tmp/opendoge_command.state  # Joystick commands"
echo "  ip -details link show can0                 # CAN status"
echo ""
echo "Disable:"
echo "  sudo systemctl disable opendoge-can opendoge-xboxdrv"
echo "  systemctl --user disable opendoge-imu opendoge-joystick"
