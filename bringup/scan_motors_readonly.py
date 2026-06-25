#!/usr/bin/env python3
"""
只读电机扫描工具 — 不发送任何控制指令 (不使能、不置零、不写参数)。

对每条 CAN 总线遍历全部 12 个电机 ID，使用 COMM_READ_PARAM (0x11)
查询机械位置 (0x7019)、速度 (0x701B) 和故障状态 (0x3022)。

参考: mi_motor_demo_TB.py (协议层), el05_motor_menu.py (参数索引)

用法:
    python3 scan_motors_readonly.py [--channel can0] [--ids 1,2,3] [--timeout 0.3]
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from typing import Optional

try:
    import can
except ImportError:
    print("需要 python-can: pip install python-can")
    sys.exit(1)


# ============================================================
# EL05 / RobStride 协议常量 (与 mi_motor_demo_TB.py 一致)
# ============================================================
COMM_READ_PARAM = 0x11
COMM_STATUS = 0x02

IDX_MECH_POS = 0x7019
IDX_MECH_VEL = 0x701B
IDX_FAULT_STA = 0x3022

P_MIN, P_MAX = -12.57, 12.57
V_MIN, V_MAX = -50.0, 50.0

MASTER_ID = 0xFD

# 电机编号 → 关节名
MOTOR_NAMES = {
    1: "FL_hip", 2: "FL_thigh", 3: "FL_calf",
    4: "FR_hip", 5: "FR_thigh", 6: "FR_calf",
    7: "RL_hip", 8: "RL_thigh", 9: "RL_calf",
    10: "RR_hip", 11: "RR_thigh", 12: "RR_calf",
}

# CAN 通道 → 腿名
CHANNEL_LEG = {"can0": "FL(左前)", "can1": "FR(右前)", "can2": "RL(左后)", "can3": "RR(右后)"}


def build_ext_id(comm_type: int, data2: int, target_id: int) -> int:
    """与 el05_socketcan.cpp buildExtId 一致"""
    return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_ext_id(can_id: int) -> tuple[int, int, int]:
    raw = can_id & 0x1FFFFFFF
    return (raw >> 24) & 0x1F, (raw >> 8) & 0xFFFF, raw & 0xFF


def uint_to_float(value: int, low: float, high: float, bits: int = 16) -> float:
    span = high - low
    return float(value) * span / ((1 << bits) - 1) + low


def parse_status_frame(can_id: int, data: bytes) -> Optional[dict]:
    """解析 COMM_STATUS (0x02) 反馈帧"""
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
    return {
        "motor_id": motor_id,
        "position": uint_to_float(pos_u, P_MIN, P_MAX),
        "velocity": uint_to_float(vel_u, V_MIN, V_MAX),
        "torque": uint_to_float(trq_u, -6.0, 6.0),
        "temperature": tmp_u * 0.1,
        "fault": fault,
        "mode": mode,
        "source": "status_frame",
    }


def parse_read_param_response(can_id: int, data: bytes, expected_index: int) -> Optional[dict]:
    """解析 COMM_READ_PARAM (0x11) 回复"""
    comm_type, data2, _target = parse_ext_id(can_id)
    if comm_type != COMM_READ_PARAM or len(data) < 8:
        return None
    motor_id = data2 & 0xFF
    resp_index = data[0] | (data[1] << 8)
    if resp_index != expected_index:
        return None
    raw_bytes = bytes(data[4:8])
    return {
        "motor_id": motor_id,
        "index": resp_index,
        "raw_bytes": raw_bytes,
        "float_value": struct.unpack("<f", raw_bytes)[0],
        "uint32_value": int.from_bytes(raw_bytes, "little", signed=False),
    }


def scan_motor(bus: can.BusABC, motor_id: int, timeout: float) -> dict:
    """
    只读查询一个电机 — 发送 COMM_READ_PARAM 读位置、速度、故障码。
    不发送任何控制指令。
    """
    result = {
        "motor_id": motor_id,
        "position": None,
        "velocity": None,
        "fault_sta": None,
        "fault_bits": None,
        "errors": [],
    }

    # --- 读机械位置 (0x7019) ---
    index = IDX_MECH_POS
    data_bytes = [index & 0xFF, (index >> 8) & 0xFF, 0, 0, 0, 0, 0, 0]
    can_id = build_ext_id(COMM_READ_PARAM, MASTER_ID, motor_id)
    msg = can.Message(arbitration_id=can_id, data=data_bytes, is_extended_id=True)

    try:
        bus.send(msg)
    except Exception as e:
        result["errors"].append(f"send err: {e}")
        return result

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            reply = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        except Exception:
            continue
        if reply is None:
            break
        parsed = parse_read_param_response(reply.arbitration_id, reply.data, IDX_MECH_POS)
        if parsed and parsed["motor_id"] == motor_id:
            result["position"] = parsed["float_value"]
            break

    # --- 读速度 (0x701B) ---
    index = IDX_MECH_VEL
    data_bytes = [index & 0xFF, (index >> 8) & 0xFF, 0, 0, 0, 0, 0, 0]
    can_id = build_ext_id(COMM_READ_PARAM, MASTER_ID, motor_id)
    msg = can.Message(arbitration_id=can_id, data=data_bytes, is_extended_id=True)

    try:
        bus.send(msg)
    except Exception as e:
        result["errors"].append(f"vel send err: {e}")
        return result

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            reply = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        except Exception:
            continue
        if reply is None:
            break
        parsed = parse_read_param_response(reply.arbitration_id, reply.data, IDX_MECH_VEL)
        if parsed and parsed["motor_id"] == motor_id:
            result["velocity"] = parsed["float_value"]
            break

    # --- 读故障状态 (0x3022) ---
    index = IDX_FAULT_STA
    data_bytes = [index & 0xFF, (index >> 8) & 0xFF, 0, 0, 0, 0, 0, 0]
    can_id = build_ext_id(COMM_READ_PARAM, MASTER_ID, motor_id)
    msg = can.Message(arbitration_id=can_id, data=data_bytes, is_extended_id=True)

    try:
        bus.send(msg)
    except Exception as e:
        result["errors"].append(f"fault send err: {e}")
        return result

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            reply = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        except Exception:
            continue
        if reply is None:
            break
        parsed = parse_read_param_response(reply.arbitration_id, reply.data, IDX_FAULT_STA)
        if parsed and parsed["motor_id"] == motor_id:
            result["fault_sta"] = parsed["uint32_value"]
            result["fault_bits"] = [b for b in range(32) if parsed["uint32_value"] & (1 << b)]
            break

    return result


def listen_status(bus: can.BusABC, duration: float, target_ids: set) -> list[dict]:
    """被动监听 COMM_STATUS 反馈帧 (电机运行时会周期性发送)"""
    results = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        try:
            reply = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        except Exception:
            continue
        if reply is None:
            break
        status = parse_status_frame(reply.arbitration_id, reply.data)
        if status and status["motor_id"] in target_ids:
            results.append(status)
    return results


def print_result(channel: str, r: dict) -> None:
    name = MOTOR_NAMES.get(r["motor_id"], f"id_{r['motor_id']}")
    pos_str = f"{r['position']:+.4f} rad" if r["position"] is not None else "N/A"
    vel_str = f"{r['velocity']:+.3f} rad/s" if r["velocity"] is not None else "N/A"
    fault_str = f"0x{r['fault_sta']:08X}" if r["fault_sta"] is not None else "N/A"
    bits_str = f" [{','.join(f'b{b}' for b in r['fault_bits'])}]" if r.get("fault_bits") else ""

    if r["position"] is None and r["velocity"] is None and r["fault_sta"] is None:
        # 无响应
        print(f"  [{channel}] {name:>10s}  id={r['motor_id']:02d}  ⛔ 无响应")
    else:
        status = "✅" if not r["errors"] else "⚠️"
        print(f"  [{channel}] {name:>10s}  id={r['motor_id']:02d}  {status}  pos={pos_str}  vel={vel_str}")
        if r["fault_sta"] and r["fault_sta"] != 0:
            print(f"           ⚡ FAULT: {fault_str}{bits_str}")
        elif r["fault_sta"] == 0:
            print(f"           故障: 无")


def main():
    parser = argparse.ArgumentParser(description="只读电机扫描 — 不发送任何控制指令")
    parser.add_argument("--channels", default="can0,can1,can2,can3",
                        help="CAN 通道列表, 逗号分隔 (默认 can0,can1,can2,can3)")
    parser.add_argument("--ids", default="1,2,3,4,5,6,7,8,9,10,11,12",
                        help="电机 ID 列表, 逗号分隔")
    parser.add_argument("--timeout", type=float, default=0.3,
                        help="每个查询超时 (秒)")
    parser.add_argument("--listen", type=float, default=0,
                        help="额外被动监听 N 秒 (捕获电机主动上报的状态帧)")
    parser.add_argument("--single", action="store_true",
                        help="单次: 只查第一个可用通道, 找到电机就停")
    args = parser.parse_args()

    channels = [c.strip() for c in args.channels.split(",") if c.strip()]
    motor_ids = [int(x.strip()) for x in args.ids.split(",") if x.strip()]

    print("=" * 72)
    print("OpenDoge 只读电机扫描 — 只查询参数, 不发送控制指令")
    print(f"通道: {channels}  电机: {motor_ids}  超时: {args.timeout}s")
    print("=" * 72)

    found_any = False
    all_results = []

    for channel in channels:
        leg_name = CHANNEL_LEG.get(channel, "?")
        print(f"\n--- {channel} ({leg_name}) ---")

        try:
            bus = can.interface.Bus(bustype="socketcan", channel=channel, bitrate=1000000)
        except Exception as e:
            print(f"  ⛔ 无法打开 {channel}: {e}")
            continue

        try:
            channel_has_motor = False

            # 扫描该通道上所有请求的电机 ID
            ids_on_channel = [
                mid for mid in motor_ids
                if channel == "can0" and mid in (1, 2, 3)
                or channel == "can1" and mid in (4, 5, 6)
                or channel == "can2" and mid in (7, 8, 9)
                or channel == "can3" and mid in (10, 11, 12)
            ]

            for motor_id in ids_on_channel:
                result = scan_motor(bus, motor_id, args.timeout)
                all_results.append(result)
                print_result(channel, result)
                if result["position"] is not None:
                    channel_has_motor = True
                    found_any = True

            # 可选: 被动监听
            if args.listen > 0 and channel_has_motor:
                print(f"\n  ... 监听 {args.listen}s (被动接收状态帧) ...")
                statuses = listen_status(bus, args.listen, set(ids_on_channel))
                for s in statuses:
                    name = MOTOR_NAMES.get(s["motor_id"], f"id_{s['motor_id']}")
                    print(f"  📡 {name} id={s['motor_id']:02d}  "
                          f"pos={s['position']:+.4f}  vel={s['velocity']:+.3f}  "
                          f"tau={s['torque']:+.3f}  temp={s['temperature']:.1f}°C  "
                          f"mode={s['mode']}  fault=0x{s['fault']:02X}")

            if args.single and channel_has_motor:
                break

        finally:
            bus.shutdown()

    # 汇总
    print("\n" + "=" * 72)
    responding = [r for r in all_results if r["position"] is not None]
    no_response = [r for r in all_results if r["position"] is None]
    faults = [r for r in all_results if r.get("fault_sta") and r["fault_sta"] != 0]

    print(f"扫描完成: {len(responding)}/{len(all_results)} 电机响应")
    if no_response:
        names = [MOTOR_NAMES.get(r['motor_id'], str(r['motor_id'])) for r in no_response]
        print(f"无响应: {', '.join(names)}")
    if faults:
        print(f"带故障: {len(faults)} 个")
        for r in faults:
            name = MOTOR_NAMES.get(r["motor_id"], str(r["motor_id"]))
            bits = ",".join(f"b{b}" for b in r.get("fault_bits", []))
            print(f"  {name}: 0x{r['fault_sta']:08X} [{bits}]")
    if not found_any:
        print("\n💡 提示: 如果没有电机响应, 请检查:")
        print("   1. 电机是否上电 (24V)")
        print("   2. CAN 总线接线是否正确")
        print("   3. 终端电阻是否在位 (120Ω)")
        print("   4. 电机 ID 是否与预期一致")

    return 0 if found_any else 1


if __name__ == "__main__":
    raise SystemExit(main())
