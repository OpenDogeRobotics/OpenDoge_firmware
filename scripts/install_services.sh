#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPTS_DIR="${ROOT_DIR}/scripts"

SYSTEMD_USER_DIR="${HOME}/.config/systemd/user"
SYSTEMD_SYSTEM_DIR="/etc/systemd/system"

XBOXDRV_SERVICE="opendoge-xboxdrv.service"
BRIDGE_SERVICE="opendoge-joystick.service"

echo "=== OpenDoge Service Installer ==="
echo ""

# ── 1. xboxdrv (system service, requires sudo) ───────────────────
echo "[1/3] Installing xboxdrv driver (system service) ..."
echo "      This needs sudo to install to ${SYSTEMD_SYSTEM_DIR}"

if ! command -v xboxdrv &>/dev/null; then
    echo "      [WARN] xboxdrv not found at /usr/bin/xboxdrv"
    echo "      Install with: sudo apt install xboxdrv"
fi

if command -v sudo &>/dev/null; then
    sudo cp "${SCRIPTS_DIR}/${XBOXDRV_SERVICE}" "${SYSTEMD_SYSTEM_DIR}/${XBOXDRV_SERVICE}"
    sudo systemctl daemon-reload
    sudo systemctl enable "${XBOXDRV_SERVICE}"
    echo "      Installed → ${SYSTEMD_SYSTEM_DIR}/${XBOXDRV_SERVICE}"
    echo "      Starting ..."
    sudo systemctl restart "${XBOXDRV_SERVICE}" || echo "      [WARN] xboxdrv failed to start (dongle not plugged in?)"
else
    echo "      [SKIP] sudo not available"
    exit 1
fi

# ── 2. Wait for js0 ──────────────────────────────────────────────
echo ""
echo "[2/3] Waiting for /dev/input/js0 (up to 15s) ..."
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

# ── 3. Joystick bridge (user service) ────────────────────────────
echo ""
echo "[3/3] Installing joystick bridge (user service) ..."
mkdir -p "${SYSTEMD_USER_DIR}"
cp "${SCRIPTS_DIR}/${BRIDGE_SERVICE}" "${SYSTEMD_USER_DIR}/${BRIDGE_SERVICE}"
systemctl --user daemon-reload
systemctl --user enable "${BRIDGE_SERVICE}"
systemctl --user restart "${BRIDGE_SERVICE}" || echo "      [WARN] bridge failed to start (no js0?)"
echo "      Installed → ${SYSTEMD_USER_DIR}/${BRIDGE_SERVICE}"

# ── Enable lingering so user services start at boot ──────────────
if command -v loginctl &>/dev/null; then
    loginctl enable-linger "$(whoami)" 2>/dev/null || true
    echo "      Lingering enabled for $(whoami)"
fi

# ── Summary ──────────────────────────────────────────────────────
echo ""
echo "=== Installation Complete ==="
echo ""
echo "Service status:"
echo "  systemctl status opendoge-xboxdrv           # xboxdrv (system)"
echo "  systemctl --user status opendoge-joystick   # bridge (user)"
echo ""
echo "Logs:"
echo "  journalctl -u opendoge-xboxdrv -f           # xboxdrv"
echo "  journalctl --user -u opendoge-joystick -f   # bridge"
echo ""
echo "Disable:"
echo "  sudo systemctl disable opendoge-xboxdrv"
echo "  systemctl --user disable opendoge-joystick"
echo ""
echo "Verify with: watch -n 0.2 cat /tmp/opendoge_command.state"
