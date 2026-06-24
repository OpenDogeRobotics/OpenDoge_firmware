#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPTS_DIR="${ROOT_DIR}/scripts"

SYSTEMD_USER_DIR="${HOME}/.config/systemd/user"
SYSTEMD_SYSTEM_DIR="/etc/systemd/system"

# ── User service (no sudo needed) ─────────────────────────────
BRIDGE_SERVICE="opendoge-joystick.service"
BRIDGE_SRC="${SCRIPTS_DIR}/${BRIDGE_SERVICE}"

echo "=== OpenDoge Service Installer ==="
echo ""

# ── 1. Joystick bridge (user service) ─────────────────────────
echo "[1/2] Installing joystick bridge (user service) ..."
mkdir -p "${SYSTEMD_USER_DIR}"
cp "${BRIDGE_SRC}" "${SYSTEMD_USER_DIR}/${BRIDGE_SERVICE}"
systemctl --user daemon-reload
systemctl --user enable "${BRIDGE_SERVICE}"
systemctl --user start "${BRIDGE_SERVICE}"
echo "       Installed → ${SYSTEMD_USER_DIR}/${BRIDGE_SERVICE}"

# ── 2. xboxdrv (system service, requires sudo) ─────────────────
XBOXDRV_SERVICE="opendoge-xboxdrv.service"
XBOXDRV_SRC="${SCRIPTS_DIR}/${XBOXDRV_SERVICE}"

echo ""
echo "[2/2] Installing xboxdrv driver (system service — needs sudo) ..."
if command -v sudo &>/dev/null && sudo -n true 2>/dev/null; then
    sudo cp "${XBOXDRV_SRC}" "${SYSTEMD_SYSTEM_DIR}/${XBOXDRV_SERVICE}"
    sudo systemctl daemon-reload
    sudo systemctl enable "${XBOXDRV_SERVICE}"
    sudo systemctl start "${XBOXDRV_SERVICE}"
    echo "       Installed → ${SYSTEMD_SYSTEM_DIR}/${XBOXDRV_SERVICE}"
else
    echo "       [SKIP] No passwordless sudo — install manually:"
    echo "         sudo cp ${XBOXDRV_SRC} ${SYSTEMD_SYSTEM_DIR}/"
    echo "         sudo systemctl enable --now ${XBOXDRV_SERVICE}"
    echo ""
    echo "       Or start xboxdrv manually before the bridge:"
    echo "         sudo xboxdrv --device-by-id 413d:2104 --type xbox360 --detach-kernel-driver --silent"
fi

# ── Enable lingering so user services start at boot ────────────
if command -v loginctl &>/dev/null; then
    loginctl enable-linger "$(whoami)" 2>/dev/null || true
fi

echo ""
echo "=== Done ==="
echo "Commands:"
echo "  status:    systemctl --user status opendoge-joystick"
echo "  logs:      journalctl --user -u opendoge-joystick -f"
echo "  stop:      systemctl --user stop opendoge-joystick"
echo "  restart:   systemctl --user restart opendoge-joystick"
echo "  disable:   systemctl --user disable opendoge-joystick"
