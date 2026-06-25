#!/usr/bin/env python3
"""
deploy_mujoco.py — OpenDoge MuJoCo 仿真测试

在 MuJoCo 物理引擎中复现 opendoge_deploy 的完整控制回路，验证：
  - 状态机：WaitFeedback → Ready → EnteringPosition → ActivePC / ActiveRL
  - 位置控制预启动斜坡（kp/kd 从阻尼值平滑过渡到满 PD）
  - RL 推理直跳路径（跳过 EnteringPosition）
  - 模式切换和故障降级

与 C++ 固件 (main.cpp) 保持一致：
  - 12 关节顺序：FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf,
                  RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf
  - 默认姿态：{0, 0.5, -1.3} (前腿), {0, 0.7, -1.3} (后腿)
  - PD 增益：kp=20, kd=0.3, safe_kd=2.0
  - 动作缩放：action_scale=0.25
  - 斜坡时长：pc_startup_ramp_s=2.0

用法:
  python3 deploy_mujoco.py --policy policy/opendoge_r5.onnx  # idle 起步, 手动走完整流程
  python3 deploy_mujoco.py --mode rl --policy policy/opendoge_r5.onnx  # 直接进入 RL
  python3 deploy_mujoco.py --no-render --duration 10          # headless 测试

完整手动流程 (键盘):
  idle (Ready 趴伏) → A/空格 → EnteringPosition → ActivePC (站立)
  ActivePC → X → ActiveRL (RL 推理, 方向键控制运动)
  ActiveRL → Y → ActivePC (切回站立)
  ActivePC/ActiveRL → B → Ready (趴伏)

键盘控制 (渲染窗口):
  A / 空格   — 激活位置控制 (EnteringPosition → ActivePC)
  B          — 停用 (回 Ready / 阻尼)
  X          — 切换到 RL 推理模式
  Y          — 切换回位置控制模式
  BACKSPACE  — 急停 (estop)
  ESC / Q    — 退出

依赖: pip install mujoco numpy onnxruntime
"""

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple

import numpy as np

# ─── 尝试导入 ONNX Runtime ──────────────────────────────────────────
try:
    import onnxruntime as ort
    HAS_ONNX = True
except ImportError:
    HAS_ONNX = False
    ort = None
    print("[WARN] onnxruntime 未安装。运行: pip install onnxruntime", file=sys.stderr)

# ─── 尝试导入 MuJoCo ────────────────────────────────────────────────
try:
    import mujoco
    from mujoco import viewer as mujoco_viewer
    HAS_MUJOCO = True
except ImportError:
    HAS_MUJOCO = False
    mujoco_viewer = None
    print("[WARN] mujoco 未安装。运行: pip install mujoco", file=sys.stderr)


# ══════════════════════════════════════════════════════════════════════
# 常量 (与 types.hpp 保持一致)
# ══════════════════════════════════════════════════════════════════════

NUM_JOINTS = 12
OBS_DIM = 49

JOINT_NAMES = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]

# 站立默认姿态 (rad) — 与 UniLab scene_flat.xml keyframe 一致
# 策略以此位姿训练, 用于观测计算和目标位置基准
# Front/rear asymmetry is intentional — centres CoM between legs
DEFAULT_POS = np.array([
    0.0, 0.5, -1.3,   # FL
    0.0, 0.5, -1.3,   # FR
    0.0, 0.7, -1.3,   # RL
    0.0, 0.7, -1.3,   # RR
])

# 自然趴伏姿态 (rad) — 关节失能、纯阻尼稳态在平地上
# 用于 Ready 态的初始位置, 不参与观测/目标计算
# 实测: base/hip/calf mesh 碰撞 + 脚球, thigh visual only
REST_POSE = np.array([
    0.230,  1.079, -2.681,   # FL
   -0.230,  1.079, -2.681,   # FR
    0.231,  1.090, -2.681,   # RL
   -0.230,  1.084, -2.681,   # RR
])

# 关节限位 (rad) — 与 URDF/deploy config 一致
JOINT_LOWER = np.array([
    -0.785, -0.785, -2.68,   # FL
    -0.26,  -0.785, -2.68,   # FR
    -0.785, -0.785, -2.68,   # RL
    -0.26,  -0.785, -2.68,   # RR
])

JOINT_UPPER = np.array([
    0.26,  1.134, -1.04,     # FL
    0.785, 1.134, -1.04,     # FR
    0.26,  1.134, -1.04,     # RL
    0.785, 1.134, -1.04,     # RR
])

MAX_POSITION_STEP = 0.015   # rad/target_tick @ 200Hz = 3 rad/s


# ══════════════════════════════════════════════════════════════════════
# 配置与数据结构
# ══════════════════════════════════════════════════════════════════════

class RuntimeState(Enum):
    WaitFeedback = 0
    Ready = 1
    EnteringPosition = 2
    ActivePC = 3
    ActiveRL = 4
    DampingFault = 5


@dataclass
class DeployConfig:
    inference_hz: float = 100.0
    target_hz: float = 200.0
    control_hz: float = 1000.0
    kp: float = 20.0
    kd: float = 0.3  # 与训练配置一致 (OpenDoge ControlConfig)
    safe_kd: float = 2.0
    action_scale: float = 0.25
    pc_startup_ramp_s: float = 2.0
    pc_startup_max_deviation: float = 2.0  # 趴伏→站立全程跟踪容差


@dataclass
class OperatorCommand:
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    active: bool = False
    estop: bool = False
    position_control: bool = False
    rl_inference: bool = False


@dataclass
class ImuSample:
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    projected_gravity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -1.0]))
    valid: bool = False


# ══════════════════════════════════════════════════════════════════════
# 工具函数 (与 main.cpp 一致)
# ══════════════════════════════════════════════════════════════════════

def rate_limit(desired: float, previous: float, max_step: float) -> float:
    """速率限制器 — 与 main.cpp rateLimit() 一致"""
    return previous + np.clip(desired - previous, -max_step, max_step)


def advance_phase(command: OperatorCommand, phase: float, dt: float) -> float:
    """自适应步态相位 — 与 main.cpp advancePhase() 一致"""
    cmd_speed = math.sqrt(command.vx**2 + command.vy**2 + command.yaw_rate**2)
    freq = np.clip(1.2 + 1.3 * cmd_speed / 0.6, 1.2, 2.5)
    return math.fmod(phase + dt * freq, 1.0)


def build_observation(
    joint_positions: np.ndarray,
    joint_velocities: np.ndarray,
    default_pos: np.ndarray,
    last_action: np.ndarray,
    command: OperatorCommand,
    imu: ImuSample,
    phase: float,
) -> np.ndarray:
    """构建 49 维观测 — 与 main.cpp buildObservation() 一致"""
    obs = np.zeros(OBS_DIM)
    offset = 0

    # 1. gyro (3)
    obs[offset:offset+3] = imu.angular_velocity
    offset += 3

    # 2. projected_gravity (3)
    obs[offset:offset+3] = imu.projected_gravity
    offset += 3

    # 3. dof_pos - default_pos (12)
    obs[offset:offset+NUM_JOINTS] = joint_positions - default_pos
    offset += NUM_JOINTS

    # 4. dof_vel (12)
    obs[offset:offset+NUM_JOINTS] = joint_velocities
    offset += NUM_JOINTS

    # 5. last_action (12)
    obs[offset:offset+NUM_JOINTS] = last_action
    offset += NUM_JOINTS

    # 6. commands (3)
    obs[offset:offset+3] = [command.vx, command.vy, command.yaw_rate]
    offset += 3

    # 7. feet_phase (4)
    obs[offset + 0] = phase
    obs[offset + 1] = math.fmod(phase + 0.5, 1.0)
    obs[offset + 2] = math.fmod(phase + 0.5, 1.0)
    obs[offset + 3] = phase

    return obs


# ══════════════════════════════════════════════════════════════════════
# MuJoCo 仿真器封装
# ══════════════════════════════════════════════════════════════════════

class OpenDogeSimulator:
    """OpenDoge MuJoCo 仿真器"""

    def __init__(self, model_path: str, render: bool = True, key_callback=None):
        if not HAS_MUJOCO:
            raise RuntimeError("MuJoCo 未安装: pip install mujoco")

        self.render_enabled = render
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # 构建关节映射：逻辑索引 → (joint_id, qpos_adr, dof_adr, actuator_id)
        self._qpos_adr = np.zeros(NUM_JOINTS, dtype=int)
        self._dof_adr = np.zeros(NUM_JOINTS, dtype=int)
        self._actuator_ids = np.zeros(NUM_JOINTS, dtype=int)

        for i, name in enumerate(JOINT_NAMES):
            try:
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            except Exception:
                raise RuntimeError(f"MuJoCo 模型中找不到关节: {name}")
            self._qpos_adr[i] = self.model.jnt_qposadr[jid]
            self._dof_adr[i] = self.model.jnt_dofadr[jid]
            # 查找对应的 actuator
            try:
                aid = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
                )
            except Exception:
                aid = i  # fallback: 顺序匹配
            self._actuator_ids[i] = aid

        # 传感器地址
        try:
            self._gyro_adr = self.model.sensor("angular-velocity").id
            self._quat_adr = self.model.sensor("orientation").id
            self._has_sensors = True
        except Exception:
            self._gyro_adr = 0
            self._quat_adr = 0
            self._has_sensors = False

        # 渲染
        if self.render_enabled:
            self.viewer = mujoco_viewer.launch_passive(
                self.model, self.data, key_callback=key_callback,
            )
        else:
            self.viewer = None

        self.step_count = 0

    def get_joint_positions(self) -> np.ndarray:
        """读取 12 个关节的当前位置 (rad) — 使用 jnt_qposadr"""
        q = np.zeros(NUM_JOINTS)
        for i in range(NUM_JOINTS):
            q[i] = self.data.qpos[self._qpos_adr[i]]
        return q

    def get_joint_velocities(self) -> np.ndarray:
        """读取 12 个关节的当前速度 (rad/s) — 使用 jnt_dofadr"""
        dq = np.zeros(NUM_JOINTS)
        for i in range(NUM_JOINTS):
            dq[i] = self.data.qvel[self._dof_adr[i]]
        return dq

    def apply_pd_control(
        self,
        targets: np.ndarray,
        kp_gains: np.ndarray,
        kd_gains: np.ndarray,
    ):
        """施加 PD 控制：力矩 = kp*(target - q) - kd*dq, 写入 data.ctrl"""
        q = self.get_joint_positions()
        dq = self.get_joint_velocities()
        torque = kp_gains * (targets - q) - kd_gains * dq
        for i in range(NUM_JOINTS):
            self.data.ctrl[self._actuator_ids[i]] = torque[i]

    def reset_to_rest_pose(self):
        """将机器人重置到自然趴伏姿态 (Ready 态初始位)"""
        mujoco.mj_resetData(self.model, self.data)
        for i in range(NUM_JOINTS):
            self.data.qpos[self._qpos_adr[i]] = REST_POSE[i]
        self.data.qpos[2] = 0.05  # 趴伏 base 贴近地面
        mujoco.mj_forward(self.model, self.data)

    @staticmethod
    def _rotate_vector_by_quat(v: np.ndarray, q: np.ndarray) -> np.ndarray:
        """用四元数 q=[w,x,y,z] 旋转向量 v: q * v * q^-1"""
        w, x, y, z = q
        t = 2.0 * (x * v[0] + y * v[1] + z * v[2])
        t2 = w * w - (x * x + y * y + z * z)
        return np.array([
            t * x + t2 * v[0] + 2.0 * w * (y * v[2] - z * v[1]),
            t * y + t2 * v[1] + 2.0 * w * (z * v[0] - x * v[2]),
            t * z + t2 * v[2] + 2.0 * w * (x * v[1] - y * v[0]),
        ])

    def get_imu(self) -> ImuSample:
        """读取 IMU 传感器数据 — 用 framequat 计算 projected_gravity 与训练一致"""
        imu = ImuSample()
        if self._has_sensors:
            imu.angular_velocity = self.data.sensordata[
                self._gyro_adr:self._gyro_adr + 3
            ].copy()
            # 用 IMU 姿态四元数将世界系重力 [0,0,-1] 旋转到机体坐标系
            quat = self.data.sensordata[
                self._quat_adr:self._quat_adr + 4
            ].copy()  # [w, x, y, z]
            imu.projected_gravity = self._rotate_vector_by_quat(
                np.array([0.0, 0.0, -1.0]), quat
            )
            imu.valid = True
        return imu

    def step(self):
        """执行一步物理仿真"""
        mujoco.mj_step(self.model, self.data)
        self.step_count += 1

    def render(self):
        """渲染当前帧"""
        if self.viewer is not None:
            try:
                if self.viewer.is_running():
                    self.viewer.sync()
            except Exception:
                pass  # 窗口已关闭

    def close(self):
        """关闭仿真 — 不显式调用 viewer.close() 以避免 GLFW segfault"""
        self.viewer = None  # 放弃引用, 让进程退出时 OS 清理 GLFW

    def is_running(self) -> bool:
        """检查渲染窗口是否仍在运行"""
        if self.viewer is not None:
            return self.viewer.is_running()
        return True


# ══════════════════════════════════════════════════════════════════════
# 模拟策略 (替代真实的 ONNX 推理)
# ══════════════════════════════════════════════════════════════════════

class MockPolicy:
    """
    模拟 RL 策略 — 生成简单的正弦波动作用于测试。
    实际部署时替换为 ONNX 推理。
    """

    def __init__(self, failure_rate: float = 0.0):
        self.failure_rate = failure_rate
        self.call_count = 0

    def infer(self, obs: np.ndarray) -> Tuple[bool, np.ndarray, str]:
        """
        模拟推理。
        返回 (success, action, error_msg)
        """
        self.call_count += 1

        if self.failure_rate > 0 and np.random.random() < self.failure_rate:
            return False, np.zeros(NUM_JOINTS), "mock: simulated inference failure"

        t = self.call_count * 0.01  # 100 Hz
        action = np.zeros(NUM_JOINTS)
        for leg_idx in range(4):
            hip_idx = leg_idx * 3
            thigh_idx = hip_idx + 1
            action[hip_idx] = 0.05 * math.sin(t * 3.0 + leg_idx * math.pi / 2)
            action[thigh_idx] = 0.03 * math.sin(t * 2.5 + leg_idx * math.pi / 2 + 0.3)
        return True, np.clip(action, -1.0, 1.0), ""


class OnnxPolicy:
    """通过 ONNX Runtime 加载真实 RL 策略进行推理。"""

    def __init__(self, model_path: str):
        if not HAS_ONNX:
            raise RuntimeError("onnxruntime 未安装: pip install onnxruntime")
        self._session = ort.InferenceSession(
            model_path,
            providers=["CPUExecutionProvider"],
        )
        self.call_count = 0

    def infer(self, obs: np.ndarray) -> Tuple[bool, np.ndarray, str]:
        """ONNX 推理。返回 (success, action, error_msg)"""
        self.call_count += 1
        try:
            input_feed = {"obs": obs.astype(np.float32).reshape(1, -1)}
            action = self._session.run(None, input_feed)[0]
            return True, action.flatten(), ""
        except Exception as e:
            return False, np.zeros(NUM_JOINTS), f"ONNX inference error: {e}"


# ══════════════════════════════════════════════════════════════════════
# 键盘输入 (模拟手柄)
# ══════════════════════════════════════════════════════════════════════

class KeyboardHandler:
    """通过 MuJoCo viewer 的键盘回调模拟手柄输入

    GLFW 键码参考:
      ESC=256, Q=81, Backspace=259, Space=32
      A=65, B=66, X=88, Y=89
      上=265, 下=266, 左=263, 右=262
    """

    def __init__(self):
        self.active_requested = False
        self.estop = False
        self.position_control = False
        self.rl_inference = False
        self.should_quit = False
        self._pending_keys = []
        # 方向键速度命令
        self._arrow_vx = 0.0
        self._arrow_vy = 0.0
        self._arrow_yaw = 0.0

    def feed_key(self, keycode: int):
        """从 viewer callback 接收按键 (GLFW 线程调用)"""
        self._pending_keys.append(keycode)

    def process_keys(self):
        """处理累积的按键 (主线程调用)"""
        for keycode in self._pending_keys:
            self._handle_key(keycode)
        self._pending_keys.clear()

    def _handle_key(self, keycode: int):
        if keycode in (256, 257, 81):  # ESC, GLFW_ESCAPE, Q
            self.should_quit = True
            print(f"[KEY] 退出 (key={keycode})")

        elif keycode in (259, 8):  # Backspace (GLFW=259, ASCII=8)
            self.estop = True
            self.active_requested = False
            self.position_control = False
            self.rl_inference = False
            self._arrow_vx = 0.0
            self._arrow_vy = 0.0
            self._arrow_yaw = 0.0
            print(f"[KEY] 急停 estop (key={keycode})")

        elif keycode in (65, 32):  # A, Space
            self.estop = False
            self.active_requested = True
            self.position_control = True
            self.rl_inference = False
            print(f"[KEY] 位置控制 → EnteringPosition (key={keycode})")

        elif keycode in (66,):  # B
            self.active_requested = False
            self.position_control = False
            self.rl_inference = False
            self._arrow_vx = 0.0
            self._arrow_vy = 0.0
            self._arrow_yaw = 0.0
            print(f"[KEY] 停用 → Ready (key={keycode})")

        elif keycode in (88,):  # X
            if self.active_requested and not self.estop:
                self.rl_inference = True
                self.position_control = False
                print(f"[KEY] RL 推理 → ActiveRL (key={keycode})")

        elif keycode in (89,):  # Y
            if self.active_requested and not self.estop:
                self.rl_inference = False
                self.position_control = True
                print(f"[KEY] 位置控制 → ActivePC (key={keycode})")

        # ── 方向键速度命令 ──────────────────────────────────────────
        elif keycode == 265:  # ↑ 前进
            self._arrow_vx = 0.6
            print(f"[KEY] ↑ vx=+{self._arrow_vx}")
        elif keycode == 266:  # ↓ 后退
            self._arrow_vx = -0.4
            print(f"[KEY] ↓ vx={self._arrow_vx}")
        elif keycode == 263:  # ← 左移
            self._arrow_vy = 0.3
            print(f"[KEY] ← vy=+{self._arrow_vy}")
        elif keycode == 262:  # → 右移
            self._arrow_vy = -0.3
            print(f"[KEY] → vy={self._arrow_vy}")
        elif keycode == 32:  # Space (已在上面处理, 这里重置方向)
            pass
        elif keycode == 48:  # 0 键: 停止移动
            self._arrow_vx = 0.0
            self._arrow_vy = 0.0
            self._arrow_yaw = 0.0
            print(f"[KEY] 0 停止移动")

    def get_command(
        self, vx: float = 0.0, vy: float = 0.0, yaw_rate: float = 0.0
    ) -> OperatorCommand:
        """生成 OperatorCommand — 方向键优先于静态命令"""
        cmd = OperatorCommand()
        cmd.vx = self._arrow_vx if self._arrow_vx != 0.0 else vx
        cmd.vy = self._arrow_vy if self._arrow_vy != 0.0 else vy
        cmd.yaw_rate = self._arrow_yaw if self._arrow_yaw != 0.0 else yaw_rate
        cmd.estop = self.estop

        active = self.active_requested and not self.estop
        cmd.active = active
        cmd.position_control = self.position_control if active else False
        cmd.rl_inference = self.rl_inference if active else False

        return cmd


# ══════════════════════════════════════════════════════════════════════
# 日志记录
# ══════════════════════════════════════════════════════════════════════

class DeployLogger:
    """CSV 日志记录器 — 每个控制周期记录完整状态, 方便 debug。"""

    COLUMNS = [
        "t", "state", "fault",
        "cmd_vx", "cmd_vy", "cmd_yaw", "cmd_active", "cmd_pos", "cmd_rl",
        "z", "phase", "rl_fb",
        # 12 joints × {q, dq, target, action, kp, kd, torque}
        *[f"{j}_{s}" for j in JOINT_NAMES for s in (
            "q", "dq", "target", "action", "kp", "kd", "torque",
        )],
    ]

    def __init__(self, log_path: str, decimation: int = 1):
        import csv
        self._file = open(log_path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.COLUMNS)
        self._decimation = decimation
        self._step = 0
        self._last_torque = np.zeros(NUM_JOINTS)

    def record(
        self,
        t: float,
        state: "RuntimeState",
        fault_latched: bool,
        command: OperatorCommand,
        q: np.ndarray,
        dq: np.ndarray,
        target: np.ndarray,
        action: np.ndarray,
        kp: np.ndarray,
        kd: np.ndarray,
        z: float,
        phase: float,
        rl_fallback: bool,
        torque: np.ndarray,
    ):
        self._step += 1
        if self._step % self._decimation != 0:
            return
        row = [
            f"{t:.6f}", state.name, int(fault_latched),
            f"{command.vx:.4f}", f"{command.vy:.4f}", f"{command.yaw_rate:.4f}",
            int(command.active), int(command.position_control), int(command.rl_inference),
            f"{z:.6f}", f"{phase:.4f}", int(rl_fallback),
        ]
        for i in range(NUM_JOINTS):
            row += [
                f"{q[i]:.6f}", f"{dq[i]:.6f}", f"{target[i]:.6f}",
                f"{action[i]:.6f}", f"{kp[i]:.4f}", f"{kd[i]:.4f}",
                f"{torque[i]:.6f}",
            ]
        self._writer.writerow(row)

    def close(self):
        self._file.close()


# ══════════════════════════════════════════════════════════════════════
# 主控制回路 (复现 main.cpp 的完整逻辑)
# ══════════════════════════════════════════════════════════════════════

class DeployController:
    """复现 C++ opendoge_deploy 的完整控制回路"""

    def __init__(
        self,
        sim: OpenDogeSimulator,
        config: DeployConfig,
        keyboard: KeyboardHandler,
        mock_policy: Optional[MockPolicy] = None,
        static_vx: float = 0.0,
        static_vy: float = 0.0,
        static_yaw: float = 0.0,
        logger: Optional["DeployLogger"] = None,
    ):
        self.sim = sim
        self.config = config
        self.keyboard = keyboard
        self.policy = mock_policy
        self.static_vx = static_vx
        self.static_vy = static_vy
        self.static_yaw = static_yaw
        self.logger = logger

        # 状态机变量
        self.runtime_state = RuntimeState.Ready
        self.fault_latched = False
        self.fault_reason = ""
        self.rl_fallback_active = False

        # 控制变量
        self.phase = 0.0
        self.action = np.zeros(NUM_JOINTS)
        self.last_action = np.zeros(NUM_JOINTS)
        self.logical_target = DEFAULT_POS.copy()
        self.limited_target = DEFAULT_POS.copy()
        self.pc_startup_start_s = 0.0
        self._pc_startup_snapshot = DEFAULT_POS.copy()

        # 快照时间戳
        self._last_infer_tick = -1
        self._last_target_tick = -1

    def _get_command(self) -> OperatorCommand:
        cmd = self.keyboard.get_command(
            vx=self.static_vx, vy=self.static_vy, yaw_rate=self.static_yaw
        )
        if cmd.rl_inference and cmd.position_control:
            cmd.position_control = False
        return cmd

    def _check_startup_deviation(self) -> bool:
        """检查 ramp 期间关节跟踪 limited_target (防失控)"""
        q = self.sim.get_joint_positions()
        for i in range(NUM_JOINTS):
            if abs(q[i] - self.limited_target[i]) > self.config.pc_startup_max_deviation:
                return False
        return True

    def step(self) -> bool:
        """执行一个控制周期。返回 True 继续，False 退出。"""
        t = self.sim.data.time
        config = self.config
        self.keyboard.process_keys()
        command = self._get_command()

        # --- 急停 ---
        if command.estop and not self.fault_latched:
            self.fault_latched = True
            self.fault_reason = "operator estop"
            self.runtime_state = RuntimeState.DampingFault
            print(f"\n[{t:.3f}s] 故障锁存: {self.fault_reason}")

        # --- 状态转换 ---
        if not self.fault_latched:

            if self.runtime_state == RuntimeState.Ready and command.active:
                if command.rl_inference:
                    self.runtime_state = RuntimeState.ActiveRL
                    print(f"\n[{t:.3f}s] Ready → ActiveRL (RL 直跳)")
                else:
                    self.runtime_state = RuntimeState.EnteringPosition
                    self.pc_startup_start_s = t
                    self._pc_startup_snapshot = self.sim.get_joint_positions().copy()
                    # 将 limited_target 重置到当前关节位, rate_limiter 逐步拉到站立
                    self.limited_target = self.sim.get_joint_positions().copy()
                    print(
                        f"\n[{t:.3f}s] Ready → EnteringPosition "
                        f"(斜坡 {config.pc_startup_ramp_s}s, "
                        f"limited_target←current)"
                    )

            elif self.runtime_state == RuntimeState.EnteringPosition:
                if not command.active:
                    self.runtime_state = RuntimeState.Ready
                    print(f"\n[{t:.3f}s] EnteringPosition → Ready (取消)")
                elif command.rl_inference:
                    self.runtime_state = RuntimeState.ActiveRL
                    print(f"\n[{t:.3f}s] EnteringPosition → ActiveRL")
                else:
                    if not self._check_startup_deviation():
                        self.fault_latched = True
                        self.fault_reason = (
                            "position control startup: joint deviation exceeds limit"
                        )
                        self.runtime_state = RuntimeState.DampingFault
                        print(f"\n[{t:.3f}s] 故障: {self.fault_reason}")
                    elif t - self.pc_startup_start_s >= config.pc_startup_ramp_s:
                        self.runtime_state = RuntimeState.ActivePC
                        print(f"\n[{t:.3f}s] EnteringPosition → ActivePC (斜坡完成)")

            elif (
                self.runtime_state in (RuntimeState.ActivePC, RuntimeState.ActiveRL)
                and not command.active
            ):
                print(f"\n[{t:.3f}s] {self.runtime_state.name} → Ready")
                self.runtime_state = RuntimeState.Ready
                self.rl_fallback_active = False

            elif self.runtime_state == RuntimeState.ActivePC and command.rl_inference:
                self.runtime_state = RuntimeState.ActiveRL
                print(f"\n[{t:.3f}s] ActivePC → ActiveRL")

            elif (
                self.runtime_state == RuntimeState.ActiveRL
                and not command.rl_inference
            ):
                self.runtime_state = RuntimeState.ActivePC
                self.rl_fallback_active = False
                print(f"\n[{t:.3f}s] ActiveRL → ActivePC")

        # --- 推理块 (100 Hz) ---
        infer_tick = int(t * config.inference_hz)
        if infer_tick != self._last_infer_tick:
            self._last_infer_tick = infer_tick
            if (
                self.runtime_state == RuntimeState.ActiveRL
                and command.rl_inference
                and self.policy is not None
            ):
                self.phase = advance_phase(command, self.phase, 1.0 / config.inference_hz)
                imu = self.sim.get_imu()
                obs = build_observation(
                    self.sim.get_joint_positions(),
                    self.sim.get_joint_velocities(),
                    DEFAULT_POS,
                    self.last_action,
                    command,
                    imu,
                    self.phase,
                )
                success, action, error = self.policy.infer(obs)
                if not success:
                    self.runtime_state = RuntimeState.ActivePC
                    self.rl_fallback_active = True
                    self.action.fill(0.0)
                    print(f"\n[{t:.3f}s] RL 推理失败 → ActivePC (降级): {error}")
                else:
                    self.action = action
            else:
                self.action.fill(0.0)

        # --- 目标计算块 (200 Hz) ---
        target_tick = int(t * config.target_hz)
        if target_tick != self._last_target_tick:
            self._last_target_tick = target_tick
            for i in range(NUM_JOINTS):
                self.last_action[i] = self.action[i]
                tgt = DEFAULT_POS[i] + self.last_action[i] * config.action_scale
                tgt = np.clip(tgt, JOINT_LOWER[i], JOINT_UPPER[i])
                self.logical_target[i] = tgt
                # RL 模式跳过 rate_limit，策略训练时目标瞬时切换，无平滑延迟
                if self.runtime_state == RuntimeState.ActiveRL:
                    self.limited_target[i] = tgt
                else:
                    self.limited_target[i] = rate_limit(
                        self.logical_target[i], self.limited_target[i], MAX_POSITION_STEP
                    )

        # --- 控制块 (1000 Hz) — 每步都执行 ---
        is_active = self.runtime_state in (RuntimeState.ActivePC, RuntimeState.ActiveRL)
        is_ramping = self.runtime_state == RuntimeState.EnteringPosition

        if not is_active and not is_ramping:
            # Ready / WaitFeedback / DampingFault: 关节全失能, 仅阻尼
            kp_gains = np.zeros(NUM_JOINTS)
            kd_gains = np.full(NUM_JOINTS, config.safe_kd)
            target_positions = self.sim.get_joint_positions()
        elif is_ramping:
            ramp_elapsed = t - self.pc_startup_start_s
            ramp_frac = min(ramp_elapsed / config.pc_startup_ramp_s, 1.0)
            kp_gains = np.full(NUM_JOINTS, ramp_frac * config.kp)
            kd_gains = np.full(
                NUM_JOINTS,
                config.safe_kd + ramp_frac * (config.kd - config.safe_kd),
            )
            target_positions = self.limited_target
        else:
            kp_gains = np.full(NUM_JOINTS, config.kp)
            kd_gains = np.full(NUM_JOINTS, config.kd)
            target_positions = self.limited_target

        self.sim.apply_pd_control(target_positions, kp_gains, kd_gains)

        # ── 日志: 记录控制输出 ──────────────────────────────────────────
        if self.logger is not None:
            q = self.sim.get_joint_positions()
            dq = self.sim.get_joint_velocities()
            torque = np.array([self.sim.data.ctrl[self.sim._actuator_ids[i]]
                               for i in range(NUM_JOINTS)])
            self.logger.record(
                t=t,
                state=self.runtime_state,
                fault_latched=self.fault_latched,
                command=command,
                q=q, dq=dq,
                target=target_positions,
                action=self.last_action,
                kp=kp_gains, kd=kd_gains,
                z=self.sim.data.qpos[2],
                phase=self.phase,
                rl_fallback=self.rl_fallback_active,
                torque=torque,
            )

        # 物理步进
        self.sim.step()

        if self.runtime_state == RuntimeState.DampingFault:
            self.fault_latched = True

        return not self.keyboard.should_quit

    def status_line(self) -> str:
        """生成状态行 (与 main.cpp 1Hz 输出一致)"""
        ramp_pct = 100
        if self.runtime_state == RuntimeState.EnteringPosition:
            elapsed = self.sim.data.time - self.pc_startup_start_s
            ramp_pct = min(int(100.0 * elapsed / self.config.pc_startup_ramp_s), 100)

        cmd = self._get_command()
        q = self.sim.get_joint_positions()
        z = self.sim.data.qpos[2]

        line = (
            f"state={self.runtime_state.name:<18} "
            f"active_cmd={1 if cmd.active else 0} "
            f"pos_ctrl={1 if cmd.position_control else 0} "
            f"rl_infer={1 if cmd.rl_inference else 0} "
            f"ramp_pct={ramp_pct:>3} "
            f"rl_fb={1 if self.rl_fallback_active else 0} "
            f"z={z:.4f} "
            f"q0={q[0]:+.3f} q1={q[1]:+.3f} q2={q[2]:+.3f}"
        )
        if self.fault_latched:
            line += f' fault="{self.fault_reason}"'
        return line


# ══════════════════════════════════════════════════════════════════════
# 主函数
# ══════════════════════════════════════════════════════════════════════

def find_model_path() -> str:
    """查找 MuJoCo XML 模型路径"""
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidates = [
        os.path.join(repo_root, "docs", "URDF", "xml", "scene.xml"),
        os.path.join(repo_root, "docs", "URDF", "xml", "Opendoge.xml"),
    ]
    for p in candidates:
        if os.path.exists(p):
            return p
    raise FileNotFoundError(
        "找不到 MuJoCo XML 模型文件。\n"
        f"尝试过的路径: {candidates}\n"
        "请使用 --model 参数指定路径。"
    )


def parse_args():
    p = argparse.ArgumentParser(
        description="OpenDoge MuJoCo 仿真测试 — 复现 deploy 控制回路",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
键盘控制:
  A / Space  激活位置控制
  B          停用 (阻尼)
  X          切换到 RL 推理
  Y          切换回位置控制
  Backspace  急停
  Esc / Q    退出
""",
    )
    p.add_argument(
        "--model", type=str, default="",
        help="MuJoCo XML 模型文件路径 (默认自动查找)"
    )
    p.add_argument(
        "--mode", choices=["pc", "rl", "idle"], default="idle",
        help="初始模式"
    )
    p.add_argument(
        "--duration", type=float, default=0.0,
        help="运行时长 (秒), 0=直到手动退出"
    )
    p.add_argument(
        "--cmd", nargs=3, type=float, default=[0.0, 0.0, 0.0],
        metavar=("VX", "VY", "YAW"), help="静态速度命令"
    )
    p.add_argument("--no-render", action="store_true", help="无渲染模式")
    p.add_argument(
        "--ramp", type=float, default=2.0,
        help="位置控制启动斜坡时长 (秒)"
    )
    p.add_argument(
        "--max-deviation", type=float, default=2.0,
        help="启动时关节最大允许偏差 (rad)"
    )
    p.add_argument(
        "--rl-failure-rate", type=float, default=0.0,
        help="模拟 RL 推理失败率 (0-1, 用于测试降级)"
    )
    p.add_argument(
        "--policy", type=str, default="",
        help="ONNX 策略模型路径 (使用真实 RL 推理替代 mock 策略)"
    )
    p.add_argument(
        "--log-dir", type=str, default="logs",
        help="日志输出目录 (默认 logs/)"
    )
    return p.parse_args()


def key_callback_wrapper(keyboard: KeyboardHandler):
    """返回一个适合 mujoco viewer 的 key callback"""
    def callback(keycode: int):
        keyboard.feed_key(keycode)
    return callback


def main():
    args = parse_args()

    if not HAS_MUJOCO:
        print("错误: MuJoCo 未安装。运行: pip install mujoco", file=sys.stderr)
        sys.exit(1)

    model_path = args.model if args.model else find_model_path()
    print(f"模型: {model_path}")

    config = DeployConfig(
        pc_startup_ramp_s=args.ramp,
        pc_startup_max_deviation=args.max_deviation,
    )
    keyboard = KeyboardHandler()
    if args.policy:
        print(f"ONNX 策略: {args.policy}")
        mock_policy = OnnxPolicy(args.policy)
    else:
        mock_policy = MockPolicy(failure_rate=args.rl_failure_rate)

    if args.mode == "pc":
        keyboard.active_requested = True
        keyboard.position_control = True
    elif args.mode == "rl":
        keyboard.active_requested = True
        keyboard.rl_inference = True

    sim = OpenDogeSimulator(
        model_path, render=not args.no_render,
        key_callback=key_callback_wrapper(keyboard) if not args.no_render else None,
    )
    sim.reset_to_rest_pose()

    print(f"站立默认位 (DEFAULT_POS, 策略训练用): {dict(zip(JOINT_NAMES, DEFAULT_POS))}")
    print(f"趴伏初始位 (REST_POSE, Ready 态):       {dict(zip(JOINT_NAMES, REST_POSE))}")
    print()
    print(
        f"配置: kp={config.kp}, kd={config.kd} (joint damping=0.5), safe_kd={config.safe_kd}, "
        f"action_scale={config.action_scale}"
    )
    print(f"斜坡: {config.pc_startup_ramp_s}s, 最大偏差: {config.pc_startup_max_deviation}rad")
    print(f"初始模式: {args.mode}")
    print(f"速度命令: vx={args.cmd[0]}, vy={args.cmd[1]}, yaw={args.cmd[2]}")
    print(f"RL 失败率: {args.rl_failure_rate}")
    print()
    print("═" * 80)
    print("键盘控制:")
    print("  A / Space  → 位置控制    B → 停用")
    print("  X          → RL 推理     Y → 位置控制")
    print("  ↑ ↓        → 前进/后退   ← → → 左移/右移")
    print("  0          → 停止移动")
    print("  Backspace  → 急停        Esc / Q → 退出")
    print("═" * 80)
    print()

    # ── 日志 ──────────────────────────────────────────────────────────
    logger = None
    if args.log_dir:
        # 相对于脚本仓库根目录
        repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if not os.path.isabs(args.log_dir):
            log_dir = os.path.join(repo_root, args.log_dir)
        else:
            log_dir = args.log_dir
        os.makedirs(log_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        mode_tag = args.mode
        policy_tag = "onnx" if args.policy else "mock"
        log_path = os.path.join(
            log_dir, f"deploy_{mode_tag}_{policy_tag}_{timestamp}.csv"
        )
        logger = DeployLogger(log_path)
        print(f"日志: {log_path}")

    controller = DeployController(
        sim=sim, config=config, keyboard=keyboard,
        mock_policy=mock_policy,
        static_vx=args.cmd[0], static_vy=args.cmd[1], static_yaw=args.cmd[2],
        logger=logger,
    )

    start_time = time.time()
    last_status_time = start_time
    step_count = 0
    paused = False

    try:
        while sim.is_running() and not keyboard.should_quit:
            # 检查 duration
            if args.duration > 0 and sim.data.time >= args.duration:
                print(f"\n达到设定运行时长 {args.duration}s，退出。")
                break

            # 键盘输入 (通过 GLFW 回调)
            if sim.viewer is not None:
                # MuJoCo viewer 通过 with 上下文获取 key events
                # 我们在这里轮询
                pass

            if not paused:
                if not controller.step():
                    break
                step_count += 1
            else:
                # 暂停时仍推进渲染
                sim.step_count += 1

            # 每秒打印状态
            now = time.time()
            if now - last_status_time >= 1.0:
                print(f"\r{controller.status_line()}", end="", flush=True)
                last_status_time = now

            # 渲染
            sim.render()

    except KeyboardInterrupt:
        print("\n用户中断 (Ctrl+C)")
    finally:
        sim_time = sim.data.time
        sim.close()
        if logger is not None:
            logger.close()
            size_kb = os.path.getsize(log_path) / 1024
            print(f"日志已保存: {log_path} ({size_kb:.1f} KB)")
        elapsed = time.time() - start_time
        print(
            f"\n仿真结束。运行 {elapsed:.1f}s, "
            f"物理时间 {sim_time:.2f}s, "
            f"{step_count} 步, "
            f"平均 {step_count / max(elapsed, 0.001):.0f} Hz",
            flush=True,
        )


if __name__ == "__main__":
    main()
    # GLFW viewer 在 Python 析构阶段会 segfault, 直接干净退出
    os._exit(0)
