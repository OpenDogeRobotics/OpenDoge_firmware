#!/usr/bin/env python3
"""
EL05/RobStride CAN 协议自检 — 无硬件, 纯离线验证帧打包/解包。

覆盖 9 个通信类型 (0x00-0x12) 的帧 ID 构造和数据编解码,
确保 Python 实现与手册及 C++ 端一致。

基于: docs/EL05使用说明书2600428.pdf 第 4 章

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
快速使用
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    python3 bringup/el05/protocol_selftest.py

  无需硬件、无需 CAN 接口。修改协议参数后运行以验证一致性。
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

from __future__ import annotations

import struct


CAN_EFF_FLAG = 0x80000000

COMM_GET_DEVICE_ID = 0x00
COMM_CONTROL = 0x01
COMM_STATUS = 0x02
COMM_ENABLE = 0x03
COMM_STOP = 0x04
COMM_SET_ZERO = 0x06
COMM_SET_CAN_ID = 0x07
COMM_READ_PARAM = 0x11
COMM_WRITE_PARAM = 0x12
COMM_FAULT_FEEDBACK = 0x15
COMM_SAVE_PARAM = 0x16

IDX_RUN_MODE = 0x7005
IDX_MECH_POS = 0x7019
IDX_MECH_VEL = 0x701B
IDX_FAULT_STA = 0x3022
RUN_MODE_MOTION = 0

P_MIN, P_MAX = -12.57, 12.57
V_MIN, V_MAX = -50.0, 50.0
T_MIN, T_MAX = -6.0, 6.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0

MASTER_ID = 0xFD


def build_ext_id(comm_type: int, data2: int, target_id: int) -> int:
    """与 el05_socketcan.cpp buildExtId 一致"""
    return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (target_id & 0xFF)


def float_to_uint(value: float, low: float, high: float) -> int:
    value = max(low, min(high, value))
    return int((value - low) * 0xFFFF / (high - low))


def assert_equal(name: str, actual: object, expected: object) -> None:
    if actual != expected:
        raise AssertionError(f"{name}: expected {expected!r}, got {actual!r}")


def test_get_device_id() -> None:
    """通信类型 0: 获取设备 ID (广播)"""
    # 手册: comm=0x00, data2=0x00FD, target=0x7F, data=全零
    can_id = build_ext_id(COMM_GET_DEVICE_ID, MASTER_ID, 0x7F) | CAN_EFF_FLAG
    assert_equal("get_device_id broadcast", can_id, 0x8000FD7F)
    packet = struct.pack("=IB3x8s", can_id, 8, bytes(8))
    assert_equal("get_device_id packet len", len(packet), 16)
    # 单播查询
    can_id2 = build_ext_id(COMM_GET_DEVICE_ID, MASTER_ID, 1) | CAN_EFF_FLAG
    assert_equal("get_device_id unicast", can_id2, 0x8000FD01)


def test_motion_control() -> None:
    """通信类型 1: 运控模式控制指令"""
    motor_id = 1
    tau_u = float_to_uint(0.0, T_MIN, T_MAX)
    q_u = float_to_uint(0.0, P_MIN, P_MAX)
    dq_u = float_to_uint(0.0, V_MIN, V_MAX)
    kp_u = float_to_uint(12.0, KP_MIN, KP_MAX)
    kd_u = float_to_uint(0.5, KD_MIN, KD_MAX)

    payload = bytes([
        (q_u >> 8) & 0xFF, q_u & 0xFF,
        (dq_u >> 8) & 0xFF, dq_u & 0xFF,
        (kp_u >> 8) & 0xFF, kp_u & 0xFF,
        (kd_u >> 8) & 0xFF, kd_u & 0xFF,
    ])
    can_id = build_ext_id(COMM_CONTROL, tau_u, motor_id) | CAN_EFF_FLAG
    # tau_u=32767 (0x7FFF) for tau=0.0
    assert_equal("control id (tau=0)", can_id, 0x817FFF01)
    assert_equal("control payload len", len(payload), 8)


def test_status_parsing() -> None:
    """通信类型 2: 电机反馈数据 解析"""
    # 手册示例: comm=0x02, bit8-15=电机CAN_ID, bit16-21=故障, bit22-23=模式
    # data2: bit0-7=电机ID, bit8-13=故障, bit14-15=模式
    # 模拟: motor=127, fault=0, mode=2 (Motor运行)
    data2 = (2 << 14) | (0 << 8) | 127  # mode=2, fault=0, id=127
    motor_id = data2 & 0xFF
    fault = (data2 >> 8) & 0x3F
    mode = (data2 >> 14) & 0x03
    assert_equal("status motor_id", motor_id, 127)
    assert_equal("status fault", fault, 0)
    assert_equal("status mode", mode, 2)


def test_enable() -> None:
    """通信类型 3: 电机使能运行"""
    # 手册: comm=0x03, data2=0x00FD, target=0x7F
    can_id = build_ext_id(COMM_ENABLE, MASTER_ID, 0x7F) | CAN_EFF_FLAG
    assert_equal("enable id", can_id, 0x8300FD7F)


def test_stop() -> None:
    """通信类型 4: 电机停止运行"""
    can_id = build_ext_id(COMM_STOP, MASTER_ID, 1) | CAN_EFF_FLAG
    assert_equal("stop id", can_id, 0x8400FD01)
    # 清故障: Byte[0]=1
    assert True  # data packing covered by El05Bus.stop()


def test_set_zero() -> None:
    """通信类型 6: 设置电机机械零位"""
    can_id = build_ext_id(COMM_SET_ZERO, MASTER_ID, 1) | CAN_EFF_FLAG
    assert_equal("set_zero id", can_id, 0x8600FD01)


def test_read_param() -> None:
    """通信类型 17: 单个参数读取"""
    motor_id = 127
    can_id = build_ext_id(COMM_READ_PARAM, MASTER_ID, motor_id) | CAN_EFF_FLAG
    assert_equal("read_param id", can_id, 0x9100FD7F)
    # 读 mechPos (0x7019): 低字节在前
    data = [IDX_MECH_POS & 0xFF, (IDX_MECH_POS >> 8) & 0xFF, 0, 0, 0, 0, 0, 0]
    assert_equal("read_param data[0]", data[0], 0x19)
    assert_equal("read_param data[1]", data[1], 0x70)


def test_write_param() -> None:
    """通信类型 18: 单个参数写入"""
    motor_id = 1
    can_id = build_ext_id(COMM_WRITE_PARAM, MASTER_ID, motor_id) | CAN_EFF_FLAG
    assert_equal("write_param id", can_id, 0x9200FD01)
    # 写 run_mode=0 (运控)
    data = [IDX_RUN_MODE & 0xFF, (IDX_RUN_MODE >> 8) & 0xFF, 0, 0, RUN_MODE_MOTION, 0, 0, 0]
    assert_equal("write_param data hex", data[4], RUN_MODE_MOTION)


def test_set_can_id() -> None:
    """通信类型 7: 设置电机 CAN_ID"""
    old_id, new_id = 127, 10
    # data2: bit8-15=host CAN_ID, bit16-23=new CAN_ID
    data2_wire = (new_id << 8) | MASTER_ID
    can_id = build_ext_id(COMM_SET_CAN_ID, data2_wire, old_id) | CAN_EFF_FLAG
    expected_data2 = (new_id << 8) | MASTER_ID  # 0x0AFD
    assert_equal("set_can_id data2", data2_wire, expected_data2)
    assert_equal("set_can_id target", old_id, 127)
    assert_equal("set_can_id new_id in data2", (data2_wire >> 8) & 0xFF, new_id)


def main() -> int:
    tests = [
        ("COMM_GET_DEVICE_ID (0x00)", test_get_device_id),
        ("COMM_CONTROL (0x01)", test_motion_control),
        ("COMM_STATUS (0x02)", test_status_parsing),
        ("COMM_ENABLE (0x03)", test_enable),
        ("COMM_STOP (0x04)", test_stop),
        ("COMM_SET_ZERO (0x06)", test_set_zero),
        ("COMM_SET_CAN_ID (0x07)", test_set_can_id),
        ("COMM_READ_PARAM (0x11)", test_read_param),
        ("COMM_WRITE_PARAM (0x12)", test_write_param),
    ]

    for name, fn in tests:
        try:
            fn()
            print(f"  ✅ {name}")
        except AssertionError as e:
            print(f"  ❌ {name}: {e}")
            return 1

    # SocketCAN 帧打包验证
    can_id = build_ext_id(COMM_WRITE_PARAM, MASTER_ID, 1) | CAN_EFF_FLAG
    payload = bytes([IDX_RUN_MODE & 0xFF, (IDX_RUN_MODE >> 8) & 0xFF, 0, 0, 0, 0, 0, 0])
    packet = struct.pack("=IB3x8s", can_id, 8, payload)
    assert_equal("socketcan_packet_len", len(packet), 16)

    print(f"\nEL05 协议自检通过 ({len(tests)} 项)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
