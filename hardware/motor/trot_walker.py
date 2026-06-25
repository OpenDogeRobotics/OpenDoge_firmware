#!/usr/bin/env python3
"""
OpenDoge Trot Walker — 手柄 + IMU 位置控制 trot 步态。

功能:
  1. 从趴伏姿态平稳起立 (三段式斜坡, smoothstep 插值)
  2. 手柄遥控 trot 行走 (对角步态, 正弦关节轨迹)
  3. IMU 倾斜安全监控 + 急停 + 指令超时检测

用法:
  # 空跑测试 (不发 CAN, 仅打印目标值)
  python3 hardware/motor/trot_walker.py --dry-run

  # 仅起立 (慢速, 用于验证关节方向)
  python3 hardware/motor/trot_walker.py --stand-up-only --standup-duration 8.0

  # 完整 trot 行走 (需手柄 active + position_control)
  python3 hardware/motor/trot_walker.py

  # 低增益首次测试
  python3 hardware/motor/trot_walker.py --kp 5.0 --kd 0.1 --safe-kd 0.5

依赖: 手柄桥接 (opendoge-joystick.service) + IMU 桥接 (opendoge-imu.service)
      必须已运行，输出文件为 /tmp/opendoge_command.state 和 /tmp/opendoge_imu.state
"""

from __future__ import annotations

import argparse
import math
import os
import select
import signal
import socket
import struct
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# ── 复用同目录下 el05_motor_menu.py 的 CAN 协议实现 ──
_script_dir = str(Path(__file__).resolve().parent)
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

from el05_motor_menu import (  # noqa: E402
    CHANNEL_MOTOR_IDS,
    COMM_CONTROL,
    COMM_ENABLE,
    COMM_SAVE_PARAM,
    COMM_STOP,
    COMM_WRITE_PARAM,
    DEFAULT_MOTORS,
    IDX_RUN_MODE,
    IDX_MECH_POS,
    IDX_MECH_VEL,
    KD_MAX,
    KP_MAX,
    P_MAX,
    P_MIN,
    RUN_MODE_MOTION,
    T_MAX,
    T_MIN,
    V_MAX,
    V_MIN,
    El05Bus,
    MotorStatus,
    clamp,
    float_to_uint,
    parse_ext_id,
    parse_status,
)

# ──────────────────────────── 常量 ────────────────────────────

# 默认站立姿态 (逻辑角 / URDF 弧度, 必须匹配训练 keyframe)
DEFAULT_POS = (
    0.0, 0.5, -1.3,   # FL: hip, thigh, calf
    0.0, 0.5, -1.3,   # FR: hip, thigh, calf
    0.0, 0.7, -1.3,   # RL: hip, thigh, calf
    0.0, 0.7, -1.3,   # RR: hip, thigh, calf
)

# 关节名 (与 DEFAULT_MOTORS 同序)
JOINT_NAMES = [name for name, _mid in DEFAULT_MOTORS]

# 每通道对应的关节名 (按 hip/thigh/calf 顺序)
CHANNEL_JOINTS = {
    "can0": ["FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"],
    "can1": ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"],
    "can2": ["RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"],
    "can3": ["RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"],
}

CAN_CHANNELS = ["can0", "can1", "can2", "can3"]


# ──────────────────── 数据结构 ────────────────────

@dataclass
class JointCalibration:
    """单关节标定参数, 从 opendoge_deploy.conf 解析。"""
    name: str
    direction: float = 1.0       # 方向符号 (+1 或 -1)
    offset: float = 0.0          # 零位偏移 (rad)
    reduction: float = 1.0       # 电机→关节减速比 (>1 表示电机转得更快)
    lower: float = -12.57        # 逻辑角下限 (rad)
    upper: float = 12.57         # 逻辑角上限 (rad)
    max_position_step: float = 0.015  # 每拍最大位置步长 (rad)
    max_kp: float = 50.0
    max_kd: float = 5.0


@dataclass
class WalkerConfig:
    """运行时配置。"""
    kp: float = 20.0
    kd: float = 0.3
    safe_kd: float = 2.0
    action_scale: float = 0.25
    standup_duration: float = 4.0
    rate_hz: float = 200.0
    pc_startup_ramp_s: float = 2.0
    torque_threshold: float = 3.0
    tracking_error_threshold: float = 0.5
    fall_gz_threshold: float = 0.3
    command_timeout_s: float = 0.5
    over_temperature_c: float = 80.0
    temp_warn_c: float = 65.0
    joints: list[JointCalibration] = field(default_factory=list)


@dataclass
class OperatorCommand:
    """手柄指令 (对应 /tmp/opendoge_command.state)。"""
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    active: bool = False
    estop: bool = False
    position_control: bool = False
    rl_inference: bool = False
    clear_fault: bool = False
    low_gain_mode: bool = False


@dataclass
class ImuSample:
    """IMU 数据 (对应 /tmp/opendoge_imu.state)。"""
    wx: float = 0.0
    wy: float = 0.0
    wz: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    valid: bool = False


@dataclass
class MotorState:
    """单电机反馈状态。"""
    position: float = 0.0      # mechPos (rad)
    velocity: float = 0.0      # mechVel (rad/s)
    torque: float = 0.0        # Nm
    temperature: float = 0.0   # °C
    fault: int = 0
    mode: int = 0
    received: bool = False


# ──────────────────── 坐标转换 ────────────────────

def logical_position(motor_pos: float, cal: JointCalibration) -> float:
    """电机编码器位置 → 逻辑关节角 (URDF)。"""
    return cal.direction * (motor_pos / cal.reduction - cal.offset)


def motor_position(logical_pos: float, cal: JointCalibration) -> float:
    """逻辑关节角 (URDF) → 电机编码器位置。"""
    return (cal.offset + cal.direction * logical_pos) * cal.reduction


def logical_velocity(motor_vel: float, cal: JointCalibration) -> float:
    """电机速度 → 逻辑关节速度。"""
    return cal.direction * motor_vel / cal.reduction


def smoothstep(t: float) -> float:
    """Smoothstep 插值: 3t² - 2t³, 保证端点零速度。t 自动 clamp 到 [0,1]。"""
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def rate_limit(desired: float, previous: float, max_step: float) -> float:
    """限制每拍变化量。"""
    return previous + max(-max_step, min(max_step, desired - previous))


# ──────────────────── 配置文件解析 ────────────────────

def parse_opendoge_config(path: str) -> WalkerConfig:
    """解析 opendoge_deploy.conf, 提取标定参数和安全阈值。"""
    config = WalkerConfig()

    if not path or not os.path.exists(path):
        print(f"[WARN] 配置文件不存在: {path}, 使用默认值")
        return config

    values: dict[str, str] = {}
    with open(path) as f:
        for line in f:
            line = line.split("#")[0].strip()
            if not line or "=" not in line:
                continue
            k, v = line.split("=", 1)
            values[k.strip()] = v.strip()

    def _get(k: str, default: float) -> float:
        v = values.get(k, "")
        return float(v) if v else default

    config.kp = _get("kp", config.kp)
    config.kd = _get("kd", config.kd)
    config.safe_kd = _get("safe_kd", config.safe_kd)
    config.action_scale = _get("action_scale", config.action_scale)
    config.pc_startup_ramp_s = _get("pc_startup_ramp_s", config.pc_startup_ramp_s)
    config.torque_threshold = _get("torque_threshold", config.torque_threshold)
    config.tracking_error_threshold = _get("tracking_error_threshold", config.tracking_error_threshold)
    config.fall_gz_threshold = _get("fall_gravity_z_threshold", config.fall_gz_threshold)
    config.command_timeout_s = _get("command_timeout_s", config.command_timeout_s)
    config.over_temperature_c = _get("over_temperature_c", config.over_temperature_c)
    config.temp_warn_c = _get("temp_warn_c", config.temp_warn_c)

    # 解析每个关节的标定参数
    config.joints = []
    for name in JOINT_NAMES:
        prefix = f"joint.{name}."
        cal = JointCalibration(
            name=name,
            direction=_get(prefix + "direction", 1.0),
            offset=_get(prefix + "offset", 0.0),
            reduction=_get(prefix + "reduction", 1.0),
            lower=_get(prefix + "lower", -12.57),
            upper=_get(prefix + "upper", 12.57),
            max_position_step=_get(prefix + "max_position_step", 0.015),
            max_kp=_get(prefix + "max_kp", 50.0),
            max_kd=_get(prefix + "max_kd", 5.0),
        )
        cal.direction = -1.0 if cal.direction < 0 else 1.0
        if cal.lower > cal.upper:
            cal.lower, cal.upper = cal.upper, cal.lower
        config.joints.append(cal)

    print(f"[OK] 已加载配置: {path}")
    print(f"     PD: kp={config.kp}, kd={config.kd}, safe_kd={config.safe_kd}")
    return config


# ──────────────────── 文件 IPC 读取 ────────────────────

def read_command_file(path: str) -> OperatorCommand:
    """读取 /tmp/opendoge_command.state。解析失败返回安全默认值。"""
    cmd = OperatorCommand()
    if not path or not os.path.exists(path):
        return cmd
    try:
        values: dict[str, str] = {}
        with open(path) as f:
            for line in f:
                line = line.split("#")[0].strip()
                if not line or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                values[k.strip()] = v.strip().lower()
        cmd.vx = float(values.get("vx", "0"))
        cmd.vy = float(values.get("vy", "0"))
        cmd.yaw_rate = float(values.get("yaw_rate", "0"))
        cmd.active = values.get("active", "false") in ("true", "1", "yes", "on")
        cmd.estop = values.get("estop", "false") in ("true", "1", "yes", "on")
        cmd.position_control = values.get("position_control", "false") in ("true", "1", "yes", "on")
        cmd.rl_inference = values.get("rl_inference", "false") in ("true", "1", "yes", "on")
        cmd.clear_fault = values.get("clear_fault", "false") in ("true", "1", "yes", "on")
        cmd.low_gain_mode = values.get("low_gain_mode", "false") in ("true", "1", "yes", "on")
    except Exception:
        pass
    return cmd


def read_imu_file(path: str) -> ImuSample:
    """读取 /tmp/opendoge_imu.state。解析失败返回 invalid。"""
    imu = ImuSample()
    if not path or not os.path.exists(path):
        return imu
    try:
        values: dict[str, str] = {}
        with open(path) as f:
            for line in f:
                line = line.split("#")[0].strip()
                if not line or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                values[k.strip()] = v.strip()
        imu.wx = float(values.get("wx", "0"))
        imu.wy = float(values.get("wy", "0"))
        imu.wz = float(values.get("wz", "0"))
        imu.gx = float(values.get("gx", "0"))
        imu.gy = float(values.get("gy", "0"))
        imu.gz = float(values.get("gz", "0"))
        imu.valid = True
    except Exception:
        pass
    return imu


def command_file_mtime(path: str) -> float:
    """返回指令文件的修改时间, 不存在返回 0。"""
    try:
        return os.stat(path).st_mtime
    except OSError:
        return 0.0


# ──────────────────── TrotWalker 主类 ────────────────────

class TrotWalker:
    """手柄 + IMU 位置控制 trot 步态控制器。"""

    def __init__(
        self,
        config: WalkerConfig,
        command_file: str = "/tmp/opendoge_command.state",
        imu_file: str = "/tmp/opendoge_imu.state",
        master_id: int = 0xFD,
        dry_run: bool = False,
        quiet: bool = False,
        stand_up_only: bool = False,
    ):
        self._cfg = config
        self._command_file = command_file
        self._imu_file = imu_file
        self._master_id = master_id
        self._dry_run = dry_run
        self._quiet = quiet
        self._stand_up_only = stand_up_only
        self._period = 1.0 / max(config.rate_hz, 1.0)

        # 4 路 CAN 总线
        self._buses: dict[str, El05Bus] = {}

        # 状态机
        self._state = "init"       # init → standup → standing → trotting → fault
        self._fault_reason = ""
        self._stop = False

        # 目标数组 (逻辑角)
        self._num_joints = 12
        self._logical_targets = [0.0] * self._num_joints
        self._limited_targets = [0.0] * self._num_joints
        self._motor_targets = [0.0] * self._num_joints

        # 电机反馈
        self._motor_states = [MotorState() for _ in range(self._num_joints)]

        # 步态相位
        self._phase = 0.0          # [0, 1)
        self._trot_ramp = 0.0      # trot 进入平滑 (0→1 over 0.5s)

        # 起立计时
        self._standup_start_s = 0.0

        # IMU 参考 + 倾斜计时
        self._gz_init: Optional[float] = None
        self._tilt_start_s = 0.0

        # 指令超时
        self._last_cmd_mtime = 0.0

        # 状态打印计时
        self._next_status_s = 0.0

        # 实时 kp/kd (起立期会变化)
        self._current_kp = 0.0
        self._current_kd = config.safe_kd

        # 统计
        self._tick_count = 0
        self._late_count = 0

    # ── 公开 API ──────────────────────────────────────

    def run(self) -> int:
        """主入口: 初始化 CAN → 起立 → trot 循环 → 安全退出。返回 0=正常, 1=故障。"""
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

        if not self._dry_run:
            try:
                self._open_can()
            except OSError as e:
                print(f"[FATAL] 无法打开 CAN: {e}")
                print("请检查: sudo ip link set can0 type can bitrate 1000000 && sudo ip link set up can0")
                return 1
            self._init_motors()

        print(f"\n{'='*60}")
        print(f"OpenDoge Trot Walker 启动")
        print(f"{'='*60}")
        print(f"  模式:         {'干运行(无CAN)' if self._dry_run else '实机CAN控制'}")
        print(f"  控制频率:     {self._cfg.rate_hz} Hz")
        print(f"  起立时长:     {self._cfg.standup_duration} s")
        print(f"  PD 增益:      kp={self._cfg.kp}, kd={self._cfg.kd}, safe_kd={self._cfg.safe_kd}")
        if self._stand_up_only:
            print(f"  模式:         仅起立 (起立完成后停止)")
        print(f"  手柄文件:     {self._command_file}")
        print(f"  IMU 文件:     {self._imu_file}")
        print(f"{'='*60}")
        print()
        print("操作说明:")
        print("  A 按钮       → 激活 + 位置控制模式")
        print("  B 按钮       → 失活 (回到站立)")
        print("  BACK 按钮    → 急停 (进入 fault)")
        print("  RB (按住)    → 死手开关 (需配置 --require-rb)")
        print("  左摇杆       → vx (前后) / vy (左右)")
        print("  右摇杆       → yaw_rate (转向)")
        print(f"{'='*60}\n")

        self._state = "standup"
        self._standup_start_s = time.monotonic()

        try:
            self._main_loop()
        except KeyboardInterrupt:
            print("\n[INFO] 用户中断")
        finally:
            self._shutdown()

        if self._state == "fault":
            print(f"\n[FAULT] 故障退出: {self._fault_reason}")
            return 1
        print(f"\n[OK] Trot Walker 正常退出, 共 {self._tick_count} 拍, 超时 {self._late_count} 次")
        return 0

    # ── CAN 管理 ─────────────────────────────────────

    def _open_can(self) -> None:
        """打开全部 4 路 CAN。"""
        for ch in CAN_CHANNELS:
            bus = El05Bus(ch, self._master_id)
            bus.open()
            self._buses[ch] = bus
        print("[CAN] 4 路 CAN 总线已打开 (can0-3)")

    def _init_motors(self) -> None:
        """初始化全部 12 电机: stop → 运控模式 → 使能。"""
        print("[MOTOR] 初始化 12 个电机...")
        for name, motor_id in DEFAULT_MOTORS:
            ch = self._channel_for(motor_id)
            bus = self._buses[ch]
            # Stop (含清故障)
            bus.stop(motor_id, clear_fault=True)
            time.sleep(0.02)
            # 写运控模式
            bus.write_run_mode_motion(motor_id)
            time.sleep(0.02)
            # 使能
            status = bus.enable(motor_id)
            if status is not None:
                print(f"  [{ch}] ID={motor_id:2d} {name:18s}  位置={status.position:+.4f}  模式=运控  ✅")
            else:
                print(f"  [{ch}] ID={motor_id:2d} {name:18s}  ⚠️ 无响应")
        print("[MOTOR] 全部使能完成")

    def _channel_for(self, motor_id: int) -> str:
        """根据电机 ID 返回所属 CAN 通道。"""
        if motor_id <= 3:
            return "can0"
        elif motor_id <= 6:
            return "can1"
        elif motor_id <= 9:
            return "can2"
        else:
            return "can3"

    def _drain_all(self) -> None:
        """非阻塞清空所有 CAN 接收缓冲, 更新电机状态。"""
        for ch in CAN_CHANNELS:
            bus = self._buses[ch]
            while True:
                frame = bus.recv(0.0)
                if frame is None:
                    break
                status = parse_status(*frame)
                if status is None:
                    continue
                idx = status.motor_id - 1   # motor_id 1→index 0
                if 0 <= idx < self._num_joints:
                    ms = self._motor_states[idx]
                    ms.position = status.position
                    ms.velocity = status.velocity
                    ms.torque = status.torque
                    ms.temperature = status.temperature
                    ms.fault = status.fault
                    ms.mode = status.mode
                    ms.received = True

    def _send_control_all(self) -> None:
        """向全部 12 个电机发送运控帧 (q, dq=0, tau=0, kp, kd)。"""
        for i, (name, motor_id) in enumerate(DEFAULT_MOTORS):
            ch = self._channel_for(motor_id)
            bus = self._buses[ch]
            cal = self._cfg.joints[i]
            actual_kp = min(self._current_kp, cal.max_kp)
            actual_kd = min(self._current_kd, cal.max_kd)
            bus.control_motion(
                motor_id,
                self._motor_targets[i],
                0.0, 0.0,
                actual_kp, actual_kd,
            )

    # ── 主循环 ───────────────────────────────────────

    def _main_loop(self) -> None:
        """200 Hz 主循环。"""
        t0 = time.monotonic()
        next_tick = t0

        while not self._stop:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(max(0.0, min(next_tick - now, self._period * 0.5)))
                now = time.monotonic()

            if now >= next_tick + self._period:
                self._late_count += 1
                next_tick = now

            self._tick_count += 1

            # 1. CAN 接收
            if not self._dry_run:
                self._drain_all()

            # 2. 读取文件 IPC (每 ~20ms 实际读取即可, 但每次检查 mtime 很快)
            command = read_command_file(self._command_file)
            imu = read_imu_file(self._imu_file)

            # 3. 安全检查
            if self._safety_check(command, imu):
                if self._state == "fault":
                    self._send_damping()
                    self._send_damping()  # 额外一次确保送达
                    break

            # 4. 状态机
            self._state_machine(command, now)

            # 5. 计算目标 (逻辑角)
            if self._state == "standup":
                self._compute_standup_targets(now)
            elif self._state == "standing":
                self._logical_targets = list(DEFAULT_POS)
                self._current_kp = self._cfg.kp
                self._current_kd = self._cfg.kd
            elif self._state == "trotting":
                self._advance_phase(command)
                self._compute_trot_targets(command)
                self._current_kp = self._cfg.kp
                self._current_kd = self._cfg.kd
            elif self._state == "fault":
                # 保持当前位置, 阻尼模式
                self._current_kp = 0.0
                self._current_kd = self._cfg.safe_kd

            # 6. Rate-limit + clamp + 转换到电机空间
            for i in range(self._num_joints):
                cal = self._cfg.joints[i]
                self._logical_targets[i] = clamp(self._logical_targets[i], cal.lower, cal.upper)
                self._limited_targets[i] = rate_limit(
                    self._logical_targets[i], self._limited_targets[i], cal.max_position_step,
                )
                self._motor_targets[i] = motor_position(self._limited_targets[i], cal)

            # 7. 发送 CAN 控制帧
            if not self._dry_run:
                self._send_control_all()

            # 8. 状态打印 (1 Hz)
            if not self._quiet and now >= self._next_status_s:
                self._print_status(command)
                self._next_status_s = now + 1.0

            # 9. 如果是仅起立模式, 完成后退出
            if self._stand_up_only and self._state == "standing":
                elapsed = now - self._standup_start_s
                if elapsed > self._cfg.standup_duration + 1.0:
                    print("\n[INFO] 仅起立模式: 起立完成, 保持站立 1s 后退出")
                    break

            next_tick += self._period

    # ── 状态机 ───────────────────────────────────────

    def _state_machine(self, command: OperatorCommand, now: float) -> None:
        """状态转移逻辑。"""
        if self._state == "fault":
            return

        # standup → standing
        if self._state == "standup":
            elapsed = now - self._standup_start_s
            if elapsed >= self._cfg.standup_duration:
                self._state = "standing"
                if not self._quiet:
                    print("[STATE] 起立完成 → standing (等待手柄激活)")

        # standing → trotting
        elif self._state == "standing":
            if command.active and command.position_control and not self._stand_up_only:
                self._state = "trotting"
                self._trot_ramp = 0.0
                if not self._quiet:
                    print("[STATE] standing → trotting (手柄激活)")

        # trotting → standing
        elif self._state == "trotting":
            if not command.active:
                self._state = "standing"
                self._trot_ramp = 0.0
                if not self._quiet:
                    print("[STATE] trotting → standing (手柄失活)")

    # ── 起立目标计算 ─────────────────────────────────

    def _compute_standup_targets(self, now: float) -> None:
        """三阶段起立斜坡。"""
        elapsed = now - self._standup_start_s
        total = self._cfg.standup_duration

        # 阶段 A: 轻柔啮合 (0 → 0.8s 或 20% 总时长)
        phase_a = 0.20 * total

        # 阶段 B: 起立 (0.8s → total-0.5s 或 20%→87.5% total)
        phase_b_start = phase_a
        phase_b_end = total - 0.5  # 最后 0.5s 是阶段 C

        if elapsed <= phase_a:
            # 阶段 A: 保持趴伏姿态, kp 从 0 斜坡上升
            frac_a = smoothstep(elapsed / phase_a)
            self._current_kp = frac_a * (0.2 * self._cfg.kp)
            self._current_kd = self._cfg.safe_kd + frac_a * 0.2 * (self._cfg.kd - self._cfg.safe_kd)
            # 读取当前电机位置作为目标 (通过 drain 获取的最新状态)
            for i in range(self._num_joints):
                if self._motor_states[i].received:
                    self._logical_targets[i] = logical_position(
                        self._motor_states[i].position, self._cfg.joints[i],
                    )
                else:
                    self._logical_targets[i] = DEFAULT_POS[i]

        elif elapsed <= phase_b_end:
            # 阶段 B: smoothstep 从趴伏逻辑角到站立默认角
            frac_b = smoothstep(
                (elapsed - phase_b_start) / (phase_b_end - phase_b_start)
            )
            self._current_kp = (0.2 + 0.8 * frac_b) * self._cfg.kp
            self._current_kd = self._cfg.safe_kd + frac_b * (self._cfg.kd - self._cfg.safe_kd)

            for i in range(self._num_joints):
                cal = self._cfg.joints[i]
                # 趴伏逻辑角 = logical_position(0, cal) = -cal.offset (因为 direction=1)
                prone_logical = logical_position(0.0, cal)
                standing_logical = DEFAULT_POS[i]
                self._logical_targets[i] = (
                    prone_logical + frac_b * (standing_logical - prone_logical)
                )
        else:
            # 阶段 C: 保持站立, 满增益
            self._logical_targets = list(DEFAULT_POS)
            self._current_kp = self._cfg.kp
            self._current_kd = self._cfg.kd

    # ── Trot 目标计算 ────────────────────────────────

    def _advance_phase(self, command: OperatorCommand) -> None:
        """相位推进 (与 C++ observer.cpp 公式一致)。"""
        cmd_speed = math.sqrt(
            command.vx ** 2 + command.vy ** 2 + command.yaw_rate ** 2
        )
        freq = clamp(1.2 + 1.3 * cmd_speed / 0.6, 1.2, 2.5)  # Hz
        self._phase = math.fmod(self._phase + self._period * freq, 1.0)

    def _compute_trot_targets(self, command: OperatorCommand) -> None:
        """计算 trot 步态关节空间目标 (逻辑角)。"""
        speed = math.sqrt(command.vx ** 2 + command.vy ** 2)

        # 幅值 (随速度增大)
        A_step = clamp(0.04 + speed * 0.18, 0.04, 0.20)
        A_lift = clamp(0.06 + speed * 0.15, 0.06, 0.22)
        A_lat = clamp(0.02 + speed * 0.04, 0.02, 0.06)
        yaw_bias = -command.yaw_rate * 0.12

        # trot 进入平滑
        self._trot_ramp = min(1.0, self._trot_ramp + self._period / 0.5)
        ramp = smoothstep(self._trot_ramp)

        phi = 2.0 * math.pi * self._phase
        s = math.sin(phi)
        c = math.cos(phi)

        # 摆动相检测: swing_FLRR > 0 when FL+RR in swing (φ∈[0.5,1.0])
        swing_FLRR = max(0.0, -s)  # >0 when sin(φ) < 0
        swing_FRRL = max(0.0, s)   # >0 when sin(φ) > 0

        targets = list(DEFAULT_POS)

        # FL (索引 0-2): phase offset = 0
        targets[0] += ramp * A_lat * s               # hip lateral
        targets[1] += ramp * (A_step * c + yaw_bias)  # thigh step + yaw
        targets[2] -= ramp * A_lift * swing_FLRR       # calf lift

        # FR (索引 3-5): phase offset = 0.5 (antiphase)
        targets[3] -= ramp * A_lat * s                # hip lateral (opposite)
        targets[4] -= ramp * (A_step * c + yaw_bias)  # thigh (opposite)
        targets[5] -= ramp * A_lift * swing_FRRL       # calf lift

        # RL (索引 6-8): phase offset = 0.5
        targets[6] -= ramp * A_lat * s
        targets[7] -= ramp * (A_step * c - yaw_bias)   # yaw_bias sign flipped for rear
        targets[8] -= ramp * A_lift * swing_FRRL

        # RR (索引 9-11): phase offset = 0
        targets[9] += ramp * A_lat * s
        targets[10] += ramp * (A_step * c - yaw_bias)
        targets[11] -= ramp * A_lift * swing_FLRR

        self._logical_targets = targets

    # ── 安全 ─────────────────────────────────────────

    def _safety_check(self, command: OperatorCommand, imu: ImuSample) -> bool:
        """安全检查。返回 True 表示进入 fault。"""
        # 1. 急停
        if command.estop:
            self._enter_fault("手柄急停 (estop=true)")
            return True

        # 2. 指令超时 (仅 trotting/standing 状态检查)
        if self._state in ("standing", "trotting"):
            mtime = command_file_mtime(self._command_file)
            if self._last_cmd_mtime == 0.0:
                self._last_cmd_mtime = mtime
            elif mtime == self._last_cmd_mtime:
                # 文件未更新 — 用当前时间估算
                pass
            else:
                self._last_cmd_mtime = mtime

            # 简单检查: 如果超过 1s 文件未更新且正在 trot, 回到站立
            age = time.time() - mtime if mtime > 0 else 0
            if age > self._cfg.command_timeout_s and self._state == "trotting":
                print(f"[WARN] 指令文件超时 ({age:.1f}s), 回到站立")
                self._state = "standing"
                self._trot_ramp = 0.0
                return False

        # 3. 过温
        for i, ms in enumerate(self._motor_states):
            if ms.received and ms.temperature > self._cfg.over_temperature_c:
                self._enter_fault(
                    f"{JOINT_NAMES[i]} 过温: {ms.temperature:.1f}°C > {self._cfg.over_temperature_c}°C"
                )
                return True

        # 4. IMU 倾斜 (仅 trotting 状态检查, 使用相对于初始 gz 的变化)
        if self._state == "trotting" and imu.valid:
            if self._gz_init is None:
                self._gz_init = imu.gz
                self._tilt_start_s = 0.0
            elif abs(imu.gz - self._gz_init) > 0.5:
                now = time.monotonic()
                if self._tilt_start_s == 0.0:
                    self._tilt_start_s = now
                elif now - self._tilt_start_s > 0.3:
                    self._enter_fault(
                        f"IMU 倾斜: gz 从 {self._gz_init:+.3f} 变为 {imu.gz:+.3f}"
                    )
                    return True
            else:
                self._tilt_start_s = 0.0

        return False

    def _enter_fault(self, reason: str) -> None:
        """进入 fault 状态。"""
        self._state = "fault"
        self._fault_reason = reason
        print(f"\n[FAULT] {reason}")

    def _send_damping(self) -> None:
        """发送阻尼控制帧 (kp=0, kd=safe_kd, 保持当前电机位置)。"""
        if self._dry_run:
            return
        for i, (_name, motor_id) in enumerate(DEFAULT_MOTORS):
            ch = self._channel_for(motor_id)
            bus = self._buses[ch]
            cal = self._cfg.joints[i]
            actual_kd = min(self._cfg.safe_kd, cal.max_kd)
            # 使用当前接收到的位置作为阻尼目标
            pos = self._motor_states[i].position if self._motor_states[i].received else self._motor_targets[i]
            bus.control_motion(motor_id, pos, 0.0, 0.0, 0.0, actual_kd)

    # ── 关闭 ─────────────────────────────────────────

    def _shutdown(self) -> None:
        """安全关闭: 阻尼脉冲 → stop → 关闭 CAN。"""
        print("[SHUTDOWN] 正在安全关闭...")
        if not self._dry_run:
            # 阻尼脉冲 (20 次, 1ms 间隔)
            for _ in range(20):
                self._send_damping()
                time.sleep(0.001)
            # Stop 所有电机
            for _name, motor_id in DEFAULT_MOTORS:
                ch = self._channel_for(motor_id)
                self._buses[ch].stop(motor_id)
                time.sleep(0.002)
            print("[SHUTDOWN] 全部电机已停止")
            # 关闭 CAN
            for bus in self._buses.values():
                bus.close()
            print("[SHUTDOWN] CAN 总线已关闭")

    # ── 辅助 ─────────────────────────────────────────

    def _handle_signal(self, _signum, _frame) -> None:
        """SIGINT/SIGTERM 处理器。"""
        self._stop = True

    def _print_status(self, command: OperatorCommand) -> None:
        """打印 1 Hz 状态行。"""
        temps = [ms.temperature for ms in self._motor_states if ms.received]
        max_temp = max(temps) if temps else 0.0
        faults = sum(1 for ms in self._motor_states if ms.received and ms.fault != 0)
        speed = math.sqrt(command.vx ** 2 + command.vy ** 2)

        state_cn = {
            "init": "初始化", "standup": "起立中", "standing": "站立保持",
            "trotting": "TROT 行走", "fault": "故障",
        }

        mode_flags = []
        if command.active:
            mode_flags.append("ACT")
        if command.position_control:
            mode_flags.append("POS")
        if command.low_gain_mode:
            mode_flags.append("LOW")

        flag_str = "|".join(mode_flags) if mode_flags else "IDLE"

        elapsed = time.monotonic() - self._standup_start_s if self._state in ("standup",) else 0
        standup_info = f" t={elapsed:.1f}s" if self._state == "standup" else ""

        print(
            f"[{state_cn.get(self._state, self._state):8s}] "
            f"φ={self._phase:.3f}  "
            f"speed={speed:+.2f}  "
            f"vx={command.vx:+.2f} vy={command.vy:+.2f} yaw={command.yaw_rate:+.2f}  "
            f"kp={self._current_kp:.1f} kd={self._current_kd:.2f}  "
            f"Tmax={max_temp:.0f}°C  faults={faults}  "
            f"[{flag_str}]"
            f"{standup_info}"
        )


# ──────────────────── CLI ────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="OpenDoge Trot Walker — 手柄 + IMU 位置控制 trot 步态",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s --dry-run                     # 空跑测试
  %(prog)s --stand-up-only               # 仅起立
  %(prog)s --kp 5 --kd 0.1               # 低增益首次测试
  %(prog)s                                # 完整 trot
        """,
    )
    p.add_argument("--config", default="deploy/configs/opendoge_deploy.conf",
                   help="opendoge_deploy.conf 路径")
    p.add_argument("--command-file", default="/tmp/opendoge_command.state",
                   help="手柄指令文件路径")
    p.add_argument("--imu-file", default="/tmp/opendoge_imu.state",
                   help="IMU 数据文件路径")
    p.add_argument("--master-id", default="0xFD",
                   help="主机 CAN ID (默认 0xFD)")
    p.add_argument("--standup-duration", type=float, default=4.0,
                   help="起立时长 (秒, 默认 4.0)")
    p.add_argument("--rate-hz", type=float, default=200.0,
                   help="控制频率 (Hz, 默认 200)")
    p.add_argument("--kp", type=float, default=None,
                   help="PD Kp (覆盖配置文件)")
    p.add_argument("--kd", type=float, default=None,
                   help="PD Kd (覆盖配置文件)")
    p.add_argument("--safe-kd", type=float, default=None,
                   help="阻尼 Kd (覆盖配置文件)")
    p.add_argument("--dry-run", action="store_true",
                   help="空跑测试: 不发 CAN, 仅打印目标值")
    p.add_argument("--stand-up-only", action="store_true",
                   help="仅起立: 起立完成后保持站立并退出")
    p.add_argument("--quiet", action="store_true",
                   help="不打印 1 Hz 状态行")
    return p


def main() -> int:
    args = build_parser().parse_args()

    # 解析配置
    config_path = args.config
    if not os.path.isabs(config_path):
        # 相对路径 — 尝试从仓库根目录解析
        repo_root = Path(__file__).resolve().parent.parent.parent
        config_path = str(repo_root / config_path)
    if not os.path.exists(config_path):
        # 回退到默认路径
        alt_paths = [
            "deploy/configs/opendoge_deploy.conf",
            "install/opendoge_deploy/share/opendoge_deploy/configs/opendoge_deploy.conf",
        ]
        for alt in alt_paths:
            alt_abs = str(repo_root / alt)
            if os.path.exists(alt_abs):
                config_path = alt_abs
                break

    config = parse_opendoge_config(config_path)

    # CLI 覆盖
    if args.kp is not None:
        config.kp = args.kp
    if args.kd is not None:
        config.kd = args.kd
    if args.safe_kd is not None:
        config.safe_kd = args.safe_kd
    config.standup_duration = args.standup_duration
    config.rate_hz = args.rate_hz

    # 验证
    if not config.joints:
        print("[FATAL] 配置文件中无关节标定参数, 无法继续")
        return 1

    master_id = int(args.master_id, 0)

    walker = TrotWalker(
        config=config,
        command_file=args.command_file,
        imu_file=args.imu_file,
        master_id=master_id,
        dry_run=args.dry_run,
        quiet=args.quiet,
        stand_up_only=args.stand_up_only,
    )
    return walker.run()


if __name__ == "__main__":
    raise SystemExit(main())
