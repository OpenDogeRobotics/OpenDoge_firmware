#!/usr/bin/env python3
"""Interactive EL05/RobStride motor test menu over SocketCAN.

This tool targets the OpenDoge production signal path:

    host -> SocketCAN(can0/can1/...) -> USB2CAN forwarding board -> EL05 bus

It intentionally contains no LK/Lingkong support. EL05 is a RobStride/RS
motor and uses the RobStride private 29-bit extended CAN protocol here.
"""

from __future__ import annotations

import argparse
import math
import select
import socket
import struct
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional


CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF

COMM_CONTROL = 0x01
COMM_STATUS = 0x02
COMM_ENABLE = 0x03
COMM_STOP = 0x04
COMM_SET_ZERO = 0x06
COMM_READ_PARAM = 0x11
COMM_WRITE_PARAM = 0x12

IDX_RUN_MODE = 0x7005
IDX_MECH_POS = 0x7019
IDX_MECH_VEL = 0x701B
RUN_MODE_MOTION = 0

P_MIN = -12.57
P_MAX = 12.57
V_MIN = -50.0
V_MAX = 50.0
T_MIN = -6.0
T_MAX = 6.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0

DEFAULT_MOTORS = [
    ("fl_hip_joint", 1),
    ("fl_thigh_joint", 2),
    ("fl_knee_joint", 3),
    ("fr_hip_joint", 4),
    ("fr_thigh_joint", 5),
    ("fr_knee_joint", 6),
    ("hl_hip_joint", 7),
    ("hl_thigh_joint", 8),
    ("hl_knee_joint", 9),
    ("hr_hip_joint", 10),
    ("hr_thigh_joint", 11),
    ("hr_knee_joint", 12),
]


@dataclass
class MotorStatus:
    motor_id: int
    position: float
    velocity: float
    torque: float
    temperature: float
    fault: int
    mode: int


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def float_to_uint(value: float, low: float, high: float, bits: int = 16) -> int:
    value = clamp(value, low, high)
    span = high - low
    return int((value - low) * ((1 << bits) - 1) / span)


def uint_to_float(value: int, low: float, high: float, bits: int = 16) -> float:
    span = high - low
    return float(value) * span / ((1 << bits) - 1) + low


def build_ext_id(comm_type: int, data2: int, target_id: int) -> int:
    return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_ext_id(can_id: int) -> tuple[int, int, int]:
    raw = can_id & CAN_EFF_MASK
    return (raw >> 24) & 0x1F, (raw >> 8) & 0xFFFF, raw & 0xFF


def pack_frame(can_id: int, data: bytes | bytearray | list[int]) -> bytes:
    payload = bytes(data[:8]).ljust(8, b"\x00")
    return struct.pack("=IB3x8s", can_id | CAN_EFF_FLAG, len(payload), payload)


def unpack_frame(packet: bytes) -> tuple[int, bytes]:
    can_id, can_dlc, payload = struct.unpack("=IB3x8s", packet)
    return can_id, payload[:can_dlc]


def parse_status(can_id: int, data: bytes) -> Optional[MotorStatus]:
    comm_type, data2, _target = parse_ext_id(can_id)
    if comm_type != COMM_STATUS or len(data) < 8:
        return None

    motor_id = data2 & 0xFF
    fault = (data2 >> 8) & 0x3F
    mode = (data2 >> 14) & 0x03

    pos_u = (data[0] << 8) | data[1]
    vel_u = (data[2] << 8) | data[3]
    trq_u = (data[4] << 8) | data[5]
    tmp_u = (data[6] << 8) | data[7]

    return MotorStatus(
        motor_id=motor_id,
        position=uint_to_float(pos_u, P_MIN, P_MAX),
        velocity=uint_to_float(vel_u, V_MIN, V_MAX),
        torque=uint_to_float(trq_u, T_MIN, T_MAX),
        temperature=tmp_u * 0.1,
        fault=fault,
        mode=mode,
    )


class El05Bus:
    def __init__(self, channel: str, master_id: int):
        self.channel = channel
        self.master_id = master_id & 0xFF
        self.sock: Optional[socket.socket] = None

    def open(self) -> None:
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.setblocking(False)
        self.sock.bind((self.channel,))

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def send(self, comm_type: int, motor_id: int, data: Iterable[int] = (), data2: Optional[int] = None) -> None:
        if self.sock is None:
            raise RuntimeError("CAN socket is not open")
        if data2 is None:
            data2 = self.master_id
        can_id = build_ext_id(comm_type, data2, motor_id)
        self.sock.send(pack_frame(can_id, list(data)))

    def recv(self, timeout: float = 0.0) -> Optional[tuple[int, bytes]]:
        if self.sock is None:
            raise RuntimeError("CAN socket is not open")
        ready, _, _ = select.select([self.sock], [], [], timeout)
        if not ready:
            return None
        packet = self.sock.recv(16)
        if len(packet) < 16:
            return None
        return unpack_frame(packet)

    def drain(self) -> None:
        while self.recv(0.0) is not None:
            pass

    def wait_status(self, motor_id: int, timeout: float = 0.5) -> Optional[MotorStatus]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.recv(max(0.0, deadline - time.monotonic()))
            if frame is None:
                continue
            status = parse_status(*frame)
            if status is not None and status.motor_id == motor_id:
                return status
        return None

    def read_param_float(self, motor_id: int, index: int, timeout: float = 0.5) -> Optional[float]:
        data = [index & 0xFF, (index >> 8) & 0xFF, 0, 0, 0, 0, 0, 0]
        self.send(COMM_READ_PARAM, motor_id, data)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.recv(max(0.0, deadline - time.monotonic()))
            if frame is None:
                continue
            can_id, payload = frame
            comm_type, data2, _target = parse_ext_id(can_id)
            resp_motor_id = data2 & 0xFF
            if comm_type != COMM_READ_PARAM or resp_motor_id != motor_id or len(payload) < 8:
                continue
            resp_index = payload[0] | (payload[1] << 8)
            if resp_index != index:
                continue
            return struct.unpack("<f", payload[4:8])[0]
        return None

    def enable(self, motor_id: int) -> Optional[MotorStatus]:
        self.send(COMM_ENABLE, motor_id, [0] * 8)
        return self.wait_status(motor_id)

    def stop(self, motor_id: int, clear_fault: bool = False) -> Optional[MotorStatus]:
        data = [0] * 8
        data[0] = 1 if clear_fault else 0
        self.send(COMM_STOP, motor_id, data)
        return self.wait_status(motor_id, timeout=0.3)

    def set_zero(self, motor_id: int) -> Optional[MotorStatus]:
        data = [0] * 8
        data[0] = 1
        self.send(COMM_SET_ZERO, motor_id, data)
        return self.wait_status(motor_id, timeout=0.5)

    def write_run_mode_motion(self, motor_id: int) -> Optional[MotorStatus]:
        data = [
            IDX_RUN_MODE & 0xFF,
            (IDX_RUN_MODE >> 8) & 0xFF,
            0,
            0,
            RUN_MODE_MOTION,
            0,
            0,
            0,
        ]
        self.send(COMM_WRITE_PARAM, motor_id, data)
        return self.wait_status(motor_id, timeout=0.5)

    def control_motion(self, motor_id: int, q: float, dq: float, tau: float, kp: float, kd: float) -> None:
        tau_u = float_to_uint(tau, T_MIN, T_MAX)
        q_u = float_to_uint(q, P_MIN, P_MAX)
        dq_u = float_to_uint(dq, V_MIN, V_MAX)
        kp_u = float_to_uint(kp, KP_MIN, KP_MAX)
        kd_u = float_to_uint(kd, KD_MIN, KD_MAX)
        data = [
            (q_u >> 8) & 0xFF,
            q_u & 0xFF,
            (dq_u >> 8) & 0xFF,
            dq_u & 0xFF,
            (kp_u >> 8) & 0xFF,
            kp_u & 0xFF,
            (kd_u >> 8) & 0xFF,
            kd_u & 0xFF,
        ]
        self.send(COMM_CONTROL, motor_id, data, data2=tau_u)


def motor_label(motors: list[tuple[str, int]], motor_id: int) -> str:
    for name, mid in motors:
        if mid == motor_id:
            return name
    return f"id_{motor_id}"


def print_status(status: Optional[MotorStatus], name: str = "") -> None:
    if status is None:
        print("  no response")
        return
    prefix = f"{name} " if name else ""
    print(
        f"  {prefix}id={status.motor_id:02d} "
        f"q={status.position:+.4f} rad "
        f"dq={status.velocity:+.3f} rad/s "
        f"tau={status.torque:+.3f} Nm "
        f"temp={status.temperature:.1f} C "
        f"mode={status.mode} fault=0x{status.fault:02X}"
    )


def parse_ids(text: str, motors: list[tuple[str, int]]) -> Optional[list[int]]:
    text = text.strip()
    if not text:
        return None
    if text.lower() in {"q", "quit", "back"}:
        return []
    if text.lower() in {"a", "all"}:
        return [mid for _name, mid in motors]

    by_name = {name: mid for name, mid in motors}
    result: list[int] = []
    for token in text.replace(",", " ").split():
        if token in by_name:
            result.append(by_name[token])
            continue
        try:
            value = int(token, 0)
        except ValueError:
            print(f"  unknown motor token: {token}")
            return None
        if value < 1 or value > 127:
            print(f"  invalid motor id: {value}")
            return None
        result.append(value)
    return result


def prompt_ids(motors: list[tuple[str, int]]) -> list[int]:
    print("\nMotors:")
    for idx, (name, motor_id) in enumerate(motors):
        print(f"  {idx:02d}: id={motor_id:02d} {name}")
    print("  input examples: 1 | 1,2,3 | fl_hip_joint | a | q")
    while True:
        selected = parse_ids(input("Select motor(s): "), motors)
        if selected is not None:
            return selected


def confirm(prompt: str) -> bool:
    return input(f"{prompt} Type 'y' to continue: ").strip().lower() == "y"


def read_float(prompt: str, default: float, low: float, high: float) -> float:
    text = input(f"{prompt} [{default}]: ").strip()
    if not text:
        value = default
    else:
        try:
            value = float(text)
        except ValueError:
            print(f"  invalid value, using default {default}")
            value = default
    return clamp(value, low, high)


def cmd_read_position(bus: El05Bus, motors: list[tuple[str, int]]) -> None:
    ids = prompt_ids(motors)
    for motor_id in ids:
        bus.drain()
        pos = bus.read_param_float(motor_id, IDX_MECH_POS)
        vel = bus.read_param_float(motor_id, IDX_MECH_VEL)
        name = motor_label(motors, motor_id)
        if pos is None:
            print(f"  {name} id={motor_id:02d} no position response")
        else:
            vel_text = "n/a" if vel is None else f"{vel:+.3f} rad/s"
            print(f"  {name} id={motor_id:02d} mechPos={pos:+.4f} rad mechVel={vel_text}")


def cmd_monitor(bus: El05Bus, motors: list[tuple[str, int]]) -> None:
    ids = prompt_ids(motors)
    if not ids:
        return
    print("Listening for feedback frames only. Press Enter to stop.")
    count = 0
    selected = set(ids)
    while True:
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if ready:
            sys.stdin.readline()
            break
        frame = bus.recv(timeout=0.2)
        if frame is None:
            continue
        status = parse_status(*frame)
        if status is not None and status.motor_id in selected:
            print(f"[{count:04d}]", end="")
            print_status(status, motor_label(motors, status.motor_id))
            count += 1


def cmd_simple(bus: El05Bus, motors: list[tuple[str, int]], action: str) -> None:
    ids = prompt_ids(motors)
    if not ids:
        return
    if action == "zero" and not confirm("Setting mechanical zero changes the motor reference."):
        return
    if action in {"stop_all", "clear"} and not confirm("This will affect selected motor(s)."):
        return

    for motor_id in ids:
        bus.drain()
        if action == "enable":
            status = bus.enable(motor_id)
        elif action == "stop":
            status = bus.stop(motor_id, clear_fault=False)
        elif action == "clear":
            status = bus.stop(motor_id, clear_fault=True)
        elif action == "zero":
            bus.stop(motor_id)
            time.sleep(0.05)
            status = bus.set_zero(motor_id)
        elif action == "mode_motion":
            bus.stop(motor_id)
            time.sleep(0.05)
            status = bus.write_run_mode_motion(motor_id)
        else:
            raise ValueError(action)
        print_status(status, motor_label(motors, motor_id))


def cmd_jog(bus: El05Bus, motors: list[tuple[str, int]]) -> None:
    ids = prompt_ids(motors)
    if len(ids) != 1:
        print("  jog supports exactly one motor")
        return
    motor_id = ids[0]
    amp = read_float("Amplitude rad", 0.05, 0.0, 0.30)
    freq = read_float("Frequency Hz", 0.25, 0.01, 2.0)
    duration = read_float("Duration sec", 4.0, 0.1, 20.0)
    kp = read_float("Kp", 5.0, KP_MIN, KP_MAX)
    kd = read_float("Kd", 0.2, KD_MIN, KD_MAX)

    if not confirm("Jog will enable the motor and send motion-control frames."):
        return

    bus.drain()
    bus.stop(motor_id)
    time.sleep(0.05)
    bus.write_run_mode_motion(motor_id)
    time.sleep(0.05)
    status = bus.enable(motor_id)
    print_status(status, motor_label(motors, motor_id))
    if status is None:
        print("  abort: no initial status")
        return

    center = status.position
    hz = 100.0
    period = 1.0 / hz
    t0 = time.monotonic()
    next_t = t0
    try:
        while time.monotonic() - t0 < duration:
            t = time.monotonic() - t0
            target = center + amp * math.sin(2.0 * math.pi * freq * t)
            bus.control_motion(motor_id, target, 0.0, 0.0, kp, kd)
            if int(t * 10) != int((t - period) * 10):
                status = bus.wait_status(motor_id, timeout=0.0)
                if status is not None:
                    print_status(status, motor_label(motors, motor_id))
            next_t += period
            time.sleep(max(0.0, next_t - time.monotonic()))
    except KeyboardInterrupt:
        print("\n  interrupted")
    finally:
        bus.control_motion(motor_id, center, 0.0, 0.0, kp, kd)
        time.sleep(0.05)
        bus.stop(motor_id)
        print("  stopped")


def print_menu(channel: str, master_id: int) -> None:
    print(
        f"\nOpenDoge EL05 interactive menu | channel={channel} master=0x{master_id:02X}\n"
        "  1: list motors\n"
        "  2: read mech position/velocity params\n"
        "  3: listen feedback frames\n"
        "  4: enable selected\n"
        "  5: stop selected\n"
        "  6: clear fault selected\n"
        "  7: set motion mode selected (run_mode=0)\n"
        "  8: small jog one motor\n"
        "  9: set mechanical zero selected\n"
        "  q: quit"
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", default="can0", help="SocketCAN interface, e.g. can0")
    parser.add_argument("--master-id", default="0xfd", help="Host/master CAN id, default 0xfd")
    parser.add_argument(
        "--ids",
        default=",".join(str(mid) for _name, mid in DEFAULT_MOTORS),
        help="Comma-separated motor ids in OpenDoge joint order.",
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    master_id = int(str(args.master_id), 0)
    ids = [int(x, 0) for x in args.ids.replace(",", " ").split()]
    motors = [(DEFAULT_MOTORS[i][0] if i < len(DEFAULT_MOTORS) else f"motor_{i+1}", mid) for i, mid in enumerate(ids)]

    bus = El05Bus(args.channel, master_id)
    try:
        bus.open()
    except OSError as exc:
        print(f"Failed to open {args.channel}: {exc}")
        print("Check: sudo ip link set can0 type can bitrate 1000000 && sudo ip link set up can0")
        return 1

    try:
        while True:
            print_menu(args.channel, master_id)
            choice = input("Select: ").strip().lower()
            if choice == "1":
                for name, motor_id in motors:
                    print(f"  id={motor_id:02d} {name}")
            elif choice == "2":
                cmd_read_position(bus, motors)
            elif choice == "3":
                cmd_monitor(bus, motors)
            elif choice == "4":
                cmd_simple(bus, motors, "enable")
            elif choice == "5":
                cmd_simple(bus, motors, "stop")
            elif choice == "6":
                cmd_simple(bus, motors, "clear")
            elif choice == "7":
                cmd_simple(bus, motors, "mode_motion")
            elif choice == "8":
                cmd_jog(bus, motors)
            elif choice == "9":
                cmd_simple(bus, motors, "zero")
            elif choice in {"q", "quit", "exit"}:
                break
            else:
                print("  unknown selection")
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
