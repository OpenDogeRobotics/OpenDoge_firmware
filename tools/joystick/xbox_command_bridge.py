#!/usr/bin/env python3
"""Write OpenDoge command.state from an Xbox-compatible joystick."""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time

from opendoge_joystick import (
    LinuxJoystick,
    XboxCommandMapper,
    atomic_write,
    first_joystick_device,
    format_command_file,
)


STOP = False


def handle_signal(_signum, _frame) -> None:
    global STOP
    STOP = True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Bridge /dev/input/js* Xbox controller input to OpenDoge command.state."
    )
    parser.add_argument("--device", default=None, help="Joystick device, default: first /dev/input/js*")
    parser.add_argument("--output", default="/tmp/opendoge_command.state", help="Command file to write")
    parser.add_argument("--rate-hz", type=float, default=50.0, help="Command output update rate")
    parser.add_argument("--deadzone", type=float, default=0.08, help="Joystick axis deadzone")
    parser.add_argument("--max-vx", type=float, default=0.6, help="Max forward speed command in m/s")
    parser.add_argument("--max-vy", type=float, default=0.4, help="Max lateral speed command in m/s")
    parser.add_argument("--max-yaw-rate", type=float, default=1.0, help="Max yaw rate command in rad/s")
    parser.add_argument("--axis-vy", type=int, default=0, help="Axis number for lateral command")
    parser.add_argument("--axis-vx", type=int, default=1, help="Axis number for forward command")
    parser.add_argument("--axis-yaw", type=int, default=3, help="Axis number for yaw command")
    parser.add_argument(
        "--require-rb",
        action="store_true",
        help="Require holding RB for active output; releasing RB returns command active=false",
    )
    parser.add_argument("--quiet", action="store_true", help="Do not print periodic status")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    device = args.device or first_joystick_device()
    if not device:
        print("No joystick found under /dev/input/js*", file=sys.stderr)
        return 1

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    mapper = XboxCommandMapper(
        max_vx=args.max_vx,
        max_vy=args.max_vy,
        max_yaw_rate=args.max_yaw_rate,
        deadzone=args.deadzone,
        axis_vy=args.axis_vy,
        axis_vx=args.axis_vx,
        axis_yaw=args.axis_yaw,
        require_rb=args.require_rb,
    )

    period_s = 1.0 / max(args.rate_hz, 1.0)
    next_status_s = 0.0
    last_command = None

    with LinuxJoystick(device) as joystick:
        name = joystick.name() or "unknown joystick"
        print(f"Joystick: {device} ({name})")
        print(f"Writing:  {args.output}")
        print("Controls: left stick=vx/vy, right stick x=yaw, A=active, B=ready, X/BACK=estop, Y=clear estop")
        if args.require_rb:
            print("Deadman:  RB must be held for active output")

        while not STOP:
            loop_start_s = time.monotonic()
            mapper.update(joystick.read_events())
            command = mapper.command()
            atomic_write(args.output, format_command_file(command))
            last_command = command

            now_s = time.monotonic()
            if not args.quiet and now_s >= next_status_s:
                print(
                    f"active={int(command.active)} estop={int(command.estop)} "
                    f"vx={command.vx:+.3f} vy={command.vy:+.3f} yaw={command.yaw_rate:+.3f}"
                )
                next_status_s = now_s + 1.0

            sleep_s = period_s - (time.monotonic() - loop_start_s)
            if sleep_s > 0.0:
                time.sleep(sleep_s)

    if last_command is not None:
        last_command.vx = 0.0
        last_command.vy = 0.0
        last_command.yaw_rate = 0.0
        last_command.active = False
        atomic_write(args.output, format_command_file(last_command))
    print("Joystick bridge stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
