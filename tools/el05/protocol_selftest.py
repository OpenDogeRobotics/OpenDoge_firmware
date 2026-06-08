#!/usr/bin/env python3
"""No-hardware protocol checks for OpenDoge EL05 frame packing."""

from __future__ import annotations

import struct


CAN_EFF_FLAG = 0x80000000
COMM_CONTROL = 0x01
COMM_WRITE_PARAM = 0x12
IDX_RUN_MODE = 0x7005
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


def build_ext_id(comm_type: int, data2: int, target_id: int) -> int:
    return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (target_id & 0xFF)


def float_to_uint(value: float, low: float, high: float) -> int:
    value = max(low, min(high, value))
    return round((value - low) * 0xFFFF / (high - low))


def pack_motion(q: float, dq: float, tau: float, kp: float, kd: float) -> tuple[int, bytes]:
    tau_u = float_to_uint(tau, T_MIN, T_MAX)
    q_u = float_to_uint(q, P_MIN, P_MAX)
    dq_u = float_to_uint(dq, V_MIN, V_MAX)
    kp_u = float_to_uint(kp, KP_MIN, KP_MAX)
    kd_u = float_to_uint(kd, KD_MIN, KD_MAX)
    payload = bytes(
        [
            (q_u >> 8) & 0xFF,
            q_u & 0xFF,
            (dq_u >> 8) & 0xFF,
            dq_u & 0xFF,
            (kp_u >> 8) & 0xFF,
            kp_u & 0xFF,
            (kd_u >> 8) & 0xFF,
            kd_u & 0xFF,
        ]
    )
    return tau_u, payload


def assert_equal(name: str, actual: object, expected: object) -> None:
    if actual != expected:
        raise AssertionError(f"{name}: expected {expected!r}, got {actual!r}")


def main() -> int:
    master_id = 0xFD
    motor_id = 1

    run_mode_id = build_ext_id(COMM_WRITE_PARAM, master_id, motor_id) | CAN_EFF_FLAG
    run_mode_payload = bytes([IDX_RUN_MODE & 0xFF, (IDX_RUN_MODE >> 8) & 0xFF, 0, 0, RUN_MODE_MOTION, 0, 0, 0])
    assert_equal("run_mode_id", run_mode_id, 0x9200FD01)
    assert_equal("run_mode_payload", run_mode_payload.hex(" "), "05 70 00 00 00 00 00 00")

    tau_u, motion_payload = pack_motion(q=0.0, dq=0.0, tau=0.0, kp=12.0, kd=0.5)
    motion_id = build_ext_id(COMM_CONTROL, tau_u, motor_id) | CAN_EFF_FLAG
    assert_equal("motion_id", motion_id, 0x81800001)
    assert_equal("motion_payload_len", len(motion_payload), 8)

    socketcan_packet = struct.pack("=IB3x8s", motion_id, 8, motion_payload)
    assert_equal("socketcan_packet_len", len(socketcan_packet), 16)

    print("EL05 protocol selftest passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
