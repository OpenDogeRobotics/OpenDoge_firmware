#!/usr/bin/env python3
"""Diagnostic: start xboxdrv, monitor js0 for events."""

import os, struct, subprocess, sys, time

JS_DEV = "/dev/input/js0"
EVENT_SIZE = 8

# 1. Kill old xboxdrv
subprocess.run(["sudo", "killall", "xboxdrv"], capture_output=True)
time.sleep(1)

# 2. Start xboxdrv in background
print("[diag] Starting xboxdrv...")
proc = subprocess.Popen(
    ["sudo", "xboxdrv", "--device-by-id", "413d:2104",
     "--type", "xbox360", "--detach-kernel-driver"],
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
)
time.sleep(4)

# 3. Check js0 exists
if not os.path.exists(JS_DEV):
    print(f"ERROR: {JS_DEV} not found")
    proc.terminate()
    sys.exit(1)

print(f"[diag] {JS_DEV} ready: {open(f'/sys/class/input/js0/device/name').read().strip()}")

# 4. Open js0
fd = os.open(JS_DEV, os.O_RDONLY | os.O_NONBLOCK)
init_axes = {}
init_btns = {}
live_events = []

# 5. Drain init events first
print("[diag] Draining init events...")
for _ in range(200):
    try:
        data = os.read(fd, EVENT_SIZE)
        ts, val, etype, num = struct.unpack("IhBB", data)
        base = etype & ~0x80
        if etype & 0x80:
            if base == 0x02:
                init_axes[num] = val
            elif base == 0x01:
                init_btns[num] = val
    except BlockingIOError:
        break

print(f"[diag] Init axes: {len(init_axes)} -> " +
      ", ".join(f"a{num}={val}" for num, val in sorted(init_axes.items())))
print(f"[diag] Init btns: {len(init_btns)}")

# 6. Poll for 8 seconds - user moves sticks!
print("[diag] >>> NOW MOVE LEFT STICK + PRESS A for 8 seconds <<<")
start = time.time()
while time.time() - start < 8:
    try:
        data = os.read(fd, EVENT_SIZE)
        ts, val, etype, num = struct.unpack("IhBB", data)
        if not (etype & 0x80):  # non-init only
            base = etype & ~0x80
            kind = "AXIS" if base == 0x02 else "BTN"
            live_events.append(f"{kind} num={num} val={val:+6d}")
            print(f"  LIVE: {kind} num={num} val={val:+6d}")
    except BlockingIOError:
        time.sleep(0.002)

# 7. Report
os.close(fd)
print(f"\n[diag] === RESULT ===")
print(f"[diag] Init events: axes={len(init_axes)}, btns={len(init_btns)}")
print(f"[diag] Live events in 8s: {len(live_events)}")
if live_events:
    for e in live_events[:10]:
        print(f"  {e}")
    if len(live_events) > 10:
        print(f"  ... and {len(live_events) - 10} more")
else:
    print("[diag] *** NO LIVE EVENTS — xboxdrv is NOT sending data to js0 ***")
    print("[diag] *** This is caused by the Zikway clone dongle + xboxdrv bug ***")
    print("[diag] *** Try: sudo apt install xboxdrv-stable || use xpad kernel module ***")

proc.terminate()
