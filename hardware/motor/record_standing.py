#!/usr/bin/env python3
"""
OpenDoge 站立姿态实测标定 — 实时读取关节 mechPos。

用法:
  python3 hardware/motor/record_standing.py                  # 全部 12 关节
  python3 hardware/motor/record_standing.py --leg FL         # 只看左前腿
  python3 hardware/motor/record_standing.py --leg FR --hz 5  # 右前腿, 5 Hz

操作:
  机器人趴地、已标零 (mechPos≈0)。逐条腿掰到站立姿态，
  观察 LIVE 列读数。当关节掰到接近 TARGET 位置时，
  终端会显示 "✅ 到位!"。

  按 Enter 锁定当前显示关节的位置，按 Ctrl-C 退出并输出标定参数。
  换腿时重新运行脚本指定 --leg 即可，log 会合并到同一文件。
"""

import argparse
import json
import os
import select
import socket
import struct
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

# ── 复用 el05_motor_menu CAN 协议 ──
REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
from hardware.motor.el05_motor_menu import (
    El05Bus, COMM_READ_PARAM, IDX_MECH_POS, DEFAULT_MOTORS, motor_label,
)

LOG_DIR = Path(__file__).resolve().parent / "calibration_logs"

JOINTS = [
    # (name, can, motor_id, default_pos)
    ("FL_hip_joint",   "can0", 1,   0.0),
    ("FL_thigh_joint", "can0", 2,   0.5),
    ("FL_calf_joint",  "can0", 3,  -1.3),
    ("FR_hip_joint",   "can1", 4,   0.0),
    ("FR_thigh_joint", "can1", 5,   0.5),
    ("FR_calf_joint",  "can1", 6,  -1.3),
    ("RL_hip_joint",   "can2", 7,   0.0),
    ("RL_thigh_joint", "can2", 8,   0.7),
    ("RL_calf_joint",  "can2", 9,  -1.3),
    ("RR_hip_joint",   "can3", 10,  0.0),
    ("RR_thigh_joint", "can3", 11,  0.7),
    ("RR_calf_joint",  "can3", 12, -1.3),
]

# Motor-to-joint reduction ratios (calf joints have 1.5:1 gear reduction)
REDUCTION = [
    1.0, 1.0, 1.5,   # FL: hip, thigh, calf
    1.0, 1.0, 1.5,   # FR: hip, thigh, calf
    1.0, 1.0, 1.5,   # RL: hip, thigh, calf
    1.0, 1.0, 1.5,   # RR: hip, thigh, calf
]

# MuJoCo 仿真趴伏角 (fallback)
MUJOCO_PRONE = {
    "FL_hip_joint": 0.0600, "FL_thigh_joint": 0.7615, "FL_calf_joint": -2.2623,
    "FR_hip_joint": -0.0600, "FR_thigh_joint": 0.7615, "FR_calf_joint": -2.2624,
    "RL_hip_joint": 0.2610, "RL_thigh_joint": 1.1358, "RL_calf_joint": -2.6275,
    "RR_hip_joint": -0.2610, "RR_thigh_joint": 1.1358, "RR_calf_joint": -2.6275,
}


def compute_offset(mechpos_at_standing: float, default_pos: float, reduction: float = 1.0) -> tuple[int, float]:
    """
    logicalPos = direction * (mechPos / reduction - offset)
    At standing: direction * (mechpos_at_standing / reduction - offset) = default_pos
    => offset = mechpos_at_standing / reduction - direction * default_pos

    Default direction = +1. Returns (direction, offset).
    """
    return 1, mechpos_at_standing / reduction - default_pos


def read_all_fast(buses: dict[str, El05Bus]) -> list[Optional[float]]:
    """Parallel read: send requests to motors on open buses, collect responses."""
    results: list[Optional[float]] = [None] * 12

    # Phase 1: send all requests (only to buses that are open)
    active = [(i, ch, mid) for i, (_name, ch, mid, _dp) in enumerate(JOINTS) if ch in buses]
    for i, ch, mid in active:
        bus = buses[ch]
        bus.drain()
        data = [IDX_MECH_POS & 0xFF, (IDX_MECH_POS >> 8) & 0xFF, 0, 0, 0, 0, 0, 0]
        bus.send(COMM_READ_PARAM, mid, data)

    # Phase 2: collect responses, per-bus parallel
    deadline = time.monotonic() + 0.08
    pending = {i for i, _ch, _mid in active}

    while pending and time.monotonic() < deadline:
        for i in list(pending):
            _name, ch, _mid, _dp = JOINTS[i]
            bus = buses[ch]
            frame = bus.recv(timeout=0.0)
            if frame is None:
                continue
            can_id, payload = frame
            comm_type = ((can_id & 0x1FFFFFFF) >> 24) & 0x1F
            if comm_type != COMM_READ_PARAM or len(payload) < 8:
                continue
            resp_index = payload[0] | (payload[1] << 8)
            if resp_index != IDX_MECH_POS:
                continue
            try:
                results[i] = struct.unpack("<f", bytes(payload[4:8]))[0]
            except struct.error:
                pass
            pending.discard(i)

    return results


def save_log(captures: list[Optional[float]], positions: list[Optional[float]],
             notes: str = "") -> Path:
    """Save calibration log to hardware/motor/calibration_logs/."""
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    log_path = LOG_DIR / f"standing_calib_{timestamp}.log"

    lines = []
    lines.append(f"# OpenDoge Standing Pose Calibration Log")
    lines.append(f"# Timestamp: {datetime.now().isoformat()}")
    lines.append(f"# zero_sta=1 (-π~π), prone-position zero reference")
    if notes:
        for n in notes.strip().split('\n'):
            lines.append(f"# Notes: {n}")
    lines.append("")
    lines.append(f"{'Joint':<20s} {'motorID':>7s} {'defaultPos':>10s} "
                 f"{'liveMechPos':>12s} {'capturedMechPos':>14s} "
                 f"{'direction':>9s} {'offset':>10s} {'source':>12s}")
    lines.append("-" * 100)

    for i, (name, _ch, mid, dp) in enumerate(JOINTS):
        live = positions[i]
        cap = captures[i]

        live_str = f"{live:+.4f}" if live is not None else "N/A"
        cap_str = f"{cap:+.4f}" if cap is not None else "N/A"

        if cap is not None:
            direction, offset = compute_offset(cap, dp, REDUCTION[i])
            source = "实测锁定"
        else:
            prone = MUJOCO_PRONE.get(name, 0.0)
            direction = 1
            offset = -prone
            source = "MuJoCo仿真"

        lines.append(f"  {name:<18s} {mid:>7d} {dp:>+10.4f} {live_str:>12s} "
                     f"{cap_str:>14s} {direction:>+9d} {offset:>10.4f} {source:>12s}")

    lines.append("")
    lines.append("# Deploy config snippet (copy to opendoge_deploy.conf):")
    lines.append("")
    for i, (name, _ch, _mid, dp) in enumerate(JOINTS):
        cap = captures[i]
        if cap is not None:
            direction, offset = compute_offset(cap, dp, REDUCTION[i])
        else:
            prone = MUJOCO_PRONE.get(name, 0.0)
            direction = 1
            offset = -prone
        lines.append(f"joint.{name}.direction={direction}")
        lines.append(f"joint.{name}.offset={offset:.4f}")

    lines.append("")
    content = "\n".join(lines)
    log_path.write_text(content, encoding="utf-8")

    # Also save machine-readable JSON state
    state_path = LOG_DIR / "captures.json"
    state_path.write_text(json.dumps(captures, indent=2))

    return log_path


LEG_JOINTS = {
    "FL": [0, 1, 2],
    "FR": [3, 4, 5],
    "RL": [6, 7, 8],
    "RR": [9, 10, 11],
}


def main():
    parser = argparse.ArgumentParser(description="OpenDoge 站立姿态实测标定")
    parser.add_argument("--leg", choices=["FL", "FR", "RL", "RR"], default=None,
                        help="只处理指定腿 (FL/FR/RL/RR), 默认全部")
    parser.add_argument("--hz", type=float, default=10.0,
                        help="刷新率 (Hz), 默认 10")
    args = parser.parse_args()

    # Filter joints
    if args.leg:
        active_indices = LEG_JOINTS[args.leg.upper()]
        active_channels = {JOINTS[i][1] for i in active_indices}
        print(f"单腿模式: {args.leg.upper()} ({len(active_indices)} 关节, {active_channels})")
    else:
        active_indices = list(range(12))
        active_channels = {"can0", "can1", "can2", "can3"}

    target_hz = args.hz

    # Open only needed buses
    buses: dict[str, El05Bus] = {}
    for ch in sorted(active_channels):
        bus = El05Bus(ch, 0xFD)
        try:
            bus.open()
            bus.drain()
            buses[ch] = bus
        except OSError as e:
            print(f"无法打开 {ch}: {e}")
            return 1

    print("\033[2J\033[H", end="")  # Clear screen
    print("OpenDoge 站立姿态实测标定")
    print("=" * 78)
    print("机器人趴地、已标零 → 逐个关节掰到站立姿态 → 按 Enter 锁定读数")
    print("=" * 78)

    # Load previous captures from JSON state file (cross-session continuity)
    state_path = LOG_DIR / "captures.json"
    captures: list[Optional[float]] = [None] * 12
    if state_path.exists():
        try:
            raw = json.loads(state_path.read_text())
            for i, v in enumerate(raw):
                if v is not None and i < 12:
                    captures[i] = float(v)
            loaded = sum(1 for c in captures if c is not None)
            if loaded:
                print(f"📂 从上次加载了 {loaded}/12 个锁定值")
        except (json.JSONDecodeError, ValueError):
            pass

    iteration = 0
    period = 1.0 / target_hz
    last_t = time.monotonic()
    actual_hz = 0.0

    # Time-series CSV — records every single reading during the session
    csv_path = LOG_DIR / f"timeseries_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    csv_file = open(csv_path, 'w')
    csv_header = "t," + ",".join(JOINTS[i][0] for i in active_indices)
    csv_file.write(csv_header + "\n")
    session_start = time.monotonic()

    try:
        while True:
            t_start = time.monotonic()
            iteration += 1
            positions = read_all_fast(buses)

            # Write time-series row
            t_elapsed = t_start - session_start
            vals = [f"{positions[i]:.6f}" if positions[i] is not None else "" for i in active_indices]
            csv_file.write(f"{t_elapsed:.3f}," + ",".join(vals) + "\n")

            # Header every 20 iterations
            if iteration % 20 == 1:
                leg_tag = f" [{args.leg}]" if args.leg else ""
                print(f"\n{'#':>2s} {'关节':<18s} {'TARGET':>8s} {'LIVE':>10s} "
                      f"{'DELTA':>9s} {'锁定值':>10s} 状态{leg_tag}  [{actual_hz:.0f} Hz]")
                print("-" * 78)

            for i in active_indices:
                name, _ch, _mid, dp = JOINTS[i]
                live = positions[i]
                cap = captures[i]

                live_str = f"{live:+10.4f}" if live is not None else "    无响应 "
                delta_str = "     -    "
                cap_str = "     -    "
                status = ""

                if live is not None:
                    delta = dp - live
                    delta_str = f"{delta:+9.4f}"
                    if abs(delta) < 0.05:
                        status = "✅ 到位!"
                    elif abs(delta) < 0.15:
                        status = "接近"

                if cap is not None:
                    direction, offset = compute_offset(cap, dp, REDUCTION[i])
                    cap_str = f"{cap:+10.4f}"
                    status = f"🔒 dir={direction:+d} off={offset:+.4f}"

                marker = f"[{i+1:2d}]" if cap is not None else f" {i+1:2d} "
                print(f"{marker} {name:<18s} {dp:>+8.4f} {live_str} {delta_str} {cap_str} {status}")

            # Non-blocking stdin check (no raw mode needed)
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                line = sys.stdin.readline().strip().lower()
                if line == 'q':
                    break
                elif line == '':  # Enter key → capture all active joints
                    for i in active_indices:
                        if positions[i] is not None:
                            captures[i] = positions[i]
                    n = len(active_indices)
                    # Save immediately for crash-safety
                    LOG_DIR.mkdir(parents=True, exist_ok=True)
                    (LOG_DIR / "captures.json").write_text(json.dumps(captures, indent=2))
                    print(f"\n📸 已锁定 {n} 个关节并保存! 按 Enter 重新锁定, 按 q 退出\n")
                elif line.isdigit():
                    idx = int(line) - 1
                    if idx in active_indices:
                        if captures[idx] is not None:
                            captures[idx] = None
                            print(f"  解锁 #{idx+1} {JOINTS[idx][0]}")
                        elif positions[idx] is not None:
                            captures[idx] = positions[idx]
                            print(f"  锁定 #{idx+1} {JOINTS[idx][0]} = {positions[idx]:+.4f}")

            # Maintain target rate
            elapsed = time.monotonic() - t_start
            time.sleep(max(0.0, period - elapsed))
            dt = time.monotonic() - last_t
            if dt > 0.5:
                actual_hz = 1.0 / dt if dt > 0 else 0
                last_t = time.monotonic()

    except KeyboardInterrupt:
        print("\n\n用户中断")

    finally:
        csv_file.close()
        for bus in buses.values():
            bus.close()

    # ── Save log ──
    log_path = save_log(captures, positions)
    print(f"\n📄 Log: {log_path}")
    print(f"📊 Time-series: {csv_path} ({csv_path.stat().st_size} bytes)")

    # ── 输出结果 ──
    print("\n" + "=" * 78)
    print("标定结果")
    print("=" * 78)

    all_locked = all(c is not None for c in captures)
    locked_count = sum(1 for c in captures if c is not None)

    print(f"\n实测锁定: {locked_count}/12 关节")
    if not all_locked:
        print("未锁定关节使用 MuJoCo 仿真趴伏角作为 fallback。\n")

    for i, (name, _ch, _mid, dp) in enumerate(JOINTS):
        cap = captures[i]
        if cap is not None:
            direction, offset = compute_offset(cap, dp, REDUCTION[i])
            source = "实测锁定"
        else:
            prone = MUJOCO_PRONE.get(name, 0.0)
            direction = 1
            offset = -prone
            source = "MuJoCo仿真(未锁定)"

        # Also compute logical position at prone (mechPos=0) for verification
        prone_logical = direction * (0.0 - offset)

        print(f"  joint.{name}.direction={direction}")
        print(f"  joint.{name}.offset={offset:.4f}    # {source}, "
              f"prone_logical={prone_logical:+.4f}, standing_cmd={offset + direction*dp:+.4f}")

    print(f"\n将以上 offset 值更新到 deploy/configs/opendoge_deploy.conf。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
