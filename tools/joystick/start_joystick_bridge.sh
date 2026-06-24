#!/usr/bin/env bash
set -euo pipefail

DEVICE_ID="${XBOX_USB_ID:-413d:2104}"
JS_DEV="/dev/input/js0"
BRIDGE_SCRIPT="$(dirname "$(readlink -f "$0")")/xbox_command_bridge.py"

echo "=== OpenDoge Joystick Bridge Launcher ==="

# ── 1. Check / start xboxdrv ──────────────────────────────────
if ! pgrep -x xboxdrv > /dev/null; then
    echo "[xboxdrv] not running — starting..."
    sudo xboxdrv \
        --device-by-id "$DEVICE_ID" \
        --type xbox360 \
        --detach-kernel-driver \
        > /dev/null 2>&1 &
    echo "[xboxdrv] waiting for initialization..."
    sleep 4
else
    echo "[xboxdrv] already running"
    sleep 1
fi

# ── 2. Wait for js0 to appear ─────────────────────────────────
if [ ! -e "$JS_DEV" ]; then
    echo "[wait] waiting for $JS_DEV ..."
    for i in $(seq 1 30); do
        if [ -e "$JS_DEV" ]; then
            break
        fi
        sleep 0.5
    done
fi

if [ ! -e "$JS_DEV" ]; then
    echo "ERROR: $JS_DEV did not appear after 15s — check xboxdrv status."
    exit 1
fi

echo "[ready] $JS_DEV -> $(cat /sys/class/input/js0/device/name 2>/dev/null)"

# ── 3. Run the bridge (pass all arguments through) ─────────────
exec python3 "$BRIDGE_SCRIPT" "$@"
