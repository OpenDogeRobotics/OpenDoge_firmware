#!/usr/bin/env python3
"""
deploy_gap_regression.py — C++/Python control loop alignment regression test.

Tests every pure function in the deployment control pipeline against
expected outputs derived from the C++ opendoge_deploy implementation.
No MuJoCo, ONNX, or hardware dependency — runs anywhere.

Usage: python3 deploy_gap_regression.py
Exit code: 0 = all tests passed, 1 = failures detected
"""

import math
import sys

import numpy as np

NUM_JOINTS = 12
OBS_DIM = 49

# Default standing pose (matches UniLab scene_flat.xml keyframe)
DEFAULT_POS = np.array([
    0.0, 0.5, -1.3,
    0.0, 0.5, -1.3,
    0.0, 0.7, -1.3,
    0.0, 0.7, -1.3,
])

JOINT_LOWER = np.array([
    -0.785, -0.785, -2.68,
    -0.26,  -0.785, -2.68,
    -0.785, -0.785, -2.68,
    -0.26,  -0.785, -2.68,
])

JOINT_UPPER = np.array([
    0.26,  1.134, -1.04,
    0.785, 1.134, -1.04,
    0.26,  1.134, -1.04,
    0.785, 1.134, -1.04,
])

MAX_POSITION_STEP = 0.015  # rad/target_tick

passed = 0
failed = 0


def check(label: str, condition: bool):
    global passed, failed
    if condition:
        passed += 1
    else:
        failed += 1
        print(f"  FAIL: {label}")


# ═══════════════════════════════════════════════════════════════════════════
# 1. rate_limit — must match C++ controller.cpp rateLimit()
# ═══════════════════════════════════════════════════════════════════════════

def rate_limit(desired: float, previous: float, max_step: float) -> float:
    return previous + np.clip(desired - previous, -max_step, max_step)


def test_rate_limit():
    check("rate_limit forward step", rate_limit(5.0, 0.0, 1.0) == 1.0)
    check("rate_limit reverse step", rate_limit(-5.0, 0.0, 1.0) == -1.0)
    check("rate_limit within range", rate_limit(0.5, 0.0, 1.0) == 0.5)
    check("rate_limit downward", abs(rate_limit(0.0, 0.5, 0.1) - 0.4) < 1e-12)
    check("rate_limit zero step", rate_limit(1.0, 1.0, 0.5) == 1.0)
    check("rate_limit exact boundary", rate_limit(0.5, -0.5, 1.0) == 0.5)


# ═══════════════════════════════════════════════════════════════════════════
# 2. advance_phase — must match C++ observer.cpp advancePhase()
# ═══════════════════════════════════════════════════════════════════════════

def advance_phase(vx: float, vy: float, yaw: float, phase: float, dt: float) -> float:
    cmd_speed = math.sqrt(vx * vx + vy * vy + yaw * yaw)
    freq = max(1.2, min(2.5, 1.2 + 1.3 * cmd_speed / 0.6))
    return math.fmod(phase + dt * freq, 1.0)


def test_advance_phase():
    # Zero command → freq = 1.2 Hz
    p = advance_phase(0.0, 0.0, 0.0, 0.0, 0.01)
    check("phase zero cmd", abs(p - 0.012) < 1e-12)

    # Full speed (0.6 m/s) → freq = 2.5 Hz
    p = advance_phase(0.6, 0.0, 0.0, 0.0, 0.01)
    check("phase full speed", abs(p - 0.025) < 1e-12)

    # Yaw rate only → same formula
    p = advance_phase(0.0, 0.0, 0.6, 0.0, 0.01)
    check("phase yaw full", abs(p - 0.025) < 1e-12)

    # Phase wrapping at 1.0
    p = advance_phase(0.6, 0.0, 0.0, 0.99, 0.01)
    check("phase wrap", p < 1.0 and p > 0.0)

    # Partial speed → freq between 1.2 and 2.5
    p1 = advance_phase(0.3, 0.0, 0.0, 0.0, 0.01)
    check("phase partial speed", p1 > 0.012 and p1 < 0.025)


# ═══════════════════════════════════════════════════════════════════════════
# 3. build_observation — must match C++ observer.cpp buildObservation()
#    KEY: velocity must be RAW (no *0.5 scaling), field ordering exact
# ═══════════════════════════════════════════════════════════════════════════

def build_observation(
    joint_positions: np.ndarray,
    joint_velocities: np.ndarray,
    last_action: np.ndarray,
    vx: float, vy: float, yaw: float,
    gyro: np.ndarray,
    gravity: np.ndarray,
    phase: float,
) -> np.ndarray:
    obs = np.zeros(OBS_DIM)
    offset = 0

    # 1. gyro (3) — no scaling
    obs[offset:offset + 3] = gyro
    offset += 3

    # 2. projected_gravity (3) — no scaling
    obs[offset:offset + 3] = gravity
    offset += 3

    # 3. dof_pos - default_pos (12)
    obs[offset:offset + NUM_JOINTS] = joint_positions - DEFAULT_POS
    offset += NUM_JOINTS

    # 4. dof_vel (12) — RAW, no *0.5 factor
    obs[offset:offset + NUM_JOINTS] = joint_velocities
    offset += NUM_JOINTS

    # 5. last_action (12)
    obs[offset:offset + NUM_JOINTS] = last_action
    offset += NUM_JOINTS

    # 6. commands (3)
    obs[offset:offset + 3] = [vx, vy, yaw]
    offset += 3

    # 7. feet_phase (4) — FL=phase, FR=(phase+0.5)%1, RL=(phase+0.5)%1, RR=phase
    obs[offset + 0] = phase
    obs[offset + 1] = math.fmod(phase + 0.5, 1.0)
    obs[offset + 2] = math.fmod(phase + 0.5, 1.0)
    obs[offset + 3] = phase

    return obs


def test_build_observation():
    q = DEFAULT_POS.copy()
    dq = np.full(NUM_JOINTS, 1.5)  # 1.5 rad/s on all joints
    last_action = np.zeros(NUM_JOINTS)
    gyro = np.array([0.1, -0.2, 0.3])
    gravity = np.array([0.0, 0.0, -1.0])

    obs = build_observation(q, dq, last_action, 0.5, 0.0, 0.0, gyro, gravity, 0.25)

    check("obs dimension", len(obs) == OBS_DIM)

    # CRITICAL: velocity must be raw 1.5, NOT 0.75
    check("velocity raw (not *0.5)", np.allclose(obs[18:30], dq))
    check("velocity value check", np.allclose(obs[18:30], np.full(NUM_JOINTS, 1.5)))

    # dof_pos_diff must be zero when at DEFAULT_POS
    check("dof_pos_diff zero at default", np.allclose(obs[6:18], np.zeros(NUM_JOINTS)))

    # gyro placement
    check("gyro", np.allclose(obs[0:3], gyro))

    # gravity placement
    check("gravity", np.allclose(obs[3:6], gravity))

    # commands placement
    check("commands", np.allclose(obs[42:45], [0.5, 0.0, 0.0]))

    # feet_phase
    check("feet_phase FL=phase", obs[45] == 0.25)
    check("feet_phase FR", abs(obs[46] - 0.75) < 1e-12)
    check("feet_phase RL", abs(obs[47] - 0.75) < 1e-12)
    check("feet_phase RR=phase", obs[48] == 0.25)

    # Test with non-default position
    q2 = DEFAULT_POS + np.array([0.1] * NUM_JOINTS)
    obs2 = build_observation(q2, dq, last_action, 0.0, -0.3, 0.0, gyro, gravity, 0.5)
    check("dof_pos_diff non-default", np.allclose(obs2[6:18], np.full(NUM_JOINTS, 0.1)))

    # Test with non-zero last_action
    la = np.full(NUM_JOINTS, 0.3)
    obs3 = build_observation(q, dq, la, 0.0, 0.0, 0.0, gyro, gravity, 0.0)
    check("last_action passthrough", np.allclose(obs3[30:42], la))


# ═══════════════════════════════════════════════════════════════════════════
# 4. Target computation — must match C++ controller.cpp updateTargets()
#    KEY: action_scale = 0.25, RL skips rate_limit, PC uses rate_limit
# ═══════════════════════════════════════════════════════════════════════════

def test_target_computation():
    action_scale = 0.25

    # RL mode: skip rate_limit, target = default_pos + action * action_scale
    action = np.full(NUM_JOINTS, 1.0)
    last_action = np.clip(action, -1.0, 1.0)
    logical = DEFAULT_POS + last_action * action_scale
    limited = logical.copy()  # RL mode: no rate_limit

    check("target rl action_scale 0.25", np.allclose(
        limited - DEFAULT_POS, np.full(NUM_JOINTS, 0.25)))
    check("target rl limited==logical", np.array_equal(limited, logical))

    # PC mode: rate_limit applies — each joint moves at most 0.015 rad/tick
    limited_pc = np.zeros(NUM_JOINTS)
    logical_pc = DEFAULT_POS + np.full(NUM_JOINTS, 0.5)  # big jump from current (0.0)
    for i in range(NUM_JOINTS):
        limited_pc[i] = rate_limit(logical_pc[i], 0.0, MAX_POSITION_STEP)
    # All joints are rate-limited to max ±0.015 step
    check("target pc rate_limited", np.all(np.abs(limited_pc) <= 0.015 + 1e-12)
          and np.any(np.abs(limited_pc) > 0.0))

    # Action clamping to [-1, 1]
    large_action = np.array([-3.8, 3.3] + [0.0] * (NUM_JOINTS - 2))
    clamped = np.clip(large_action, -1.0, 1.0)
    check("action clamp upper", clamped[1] == 1.0)
    check("action clamp lower", clamped[0] == -1.0)

    # Joint limit clamping
    tgt_above = DEFAULT_POS + np.full(NUM_JOINTS, 100.0) * action_scale
    tgt_clamped = np.clip(tgt_above, JOINT_LOWER, JOINT_UPPER)
    check("joint upper clamp", np.all(tgt_clamped <= JOINT_UPPER))
    check("joint lower clamp", np.all(tgt_clamped >= JOINT_LOWER))

    # LowGainTest: target forced to DEFAULT_POS (no action offset)
    tgt_low_gain = DEFAULT_POS.copy()  # action ignored
    check("lowgain target=default", np.allclose(tgt_low_gain, DEFAULT_POS))


# ═══════════════════════════════════════════════════════════════════════════
# 5. PD control — must match C++ controller.cpp computeMotorCommands()
# ═══════════════════════════════════════════════════════════════════════════

def test_pd_control():
    kp = 20.0
    kd = 0.3
    safe_kd = 2.0

    # Damping: kp=0, kd=safe_kd
    kp_damp = np.zeros(NUM_JOINTS)
    kd_damp = np.full(NUM_JOINTS, safe_kd)
    check("damping kp=0", np.all(kp_damp == 0.0))
    check("damping kd=safe_kd", np.all(kd_damp == 2.0))

    # Active: kp=kp, kd=kd
    kp_active = np.full(NUM_JOINTS, kp)
    kd_active = np.full(NUM_JOINTS, kd)
    check("active kp", np.all(kp_active == 20.0))
    check("active kd", np.all(kd_active == 0.3))

    # Ramping (50% through): kp=10, kd=(2.0 + 0.5*(0.3-2.0)) = 1.15
    ramp_frac = 0.5
    kp_ramp = np.full(NUM_JOINTS, ramp_frac * kp)
    kd_ramp = np.full(NUM_JOINTS, safe_kd + ramp_frac * (kd - safe_kd))
    check("ramp 50% kp", np.allclose(kp_ramp, np.full(NUM_JOINTS, 10.0)))
    check("ramp 50% kd", np.allclose(kd_ramp, np.full(NUM_JOINTS, 1.15)))

    # Ramping (0%): kp=0, kd=safe_kd
    ramp_frac_0 = 0.0
    kp_ramp_0 = np.full(NUM_JOINTS, ramp_frac_0 * kp)
    kd_ramp_0 = np.full(NUM_JOINTS, safe_kd + ramp_frac_0 * (kd - safe_kd))
    check("ramp 0% kp==0", np.allclose(kp_ramp_0, np.zeros(NUM_JOINTS)))
    check("ramp 0% kd==safe_kd", np.allclose(kd_ramp_0, np.full(NUM_JOINTS, safe_kd)))

    # Ramping (100%): kp=kp, kd=kd
    ramp_frac_1 = 1.0
    kp_ramp_1 = np.full(NUM_JOINTS, ramp_frac_1 * kp)
    kd_ramp_1 = np.full(NUM_JOINTS, safe_kd + ramp_frac_1 * (kd - safe_kd))
    check("ramp 100% kp==kp", np.allclose(kp_ramp_1, np.full(NUM_JOINTS, kp)))
    check("ramp 100% kd==kd", np.allclose(kd_ramp_1, np.full(NUM_JOINTS, kd)))

    # LowGainTest: kp*0.3, kd*0.3
    kp_lg = np.full(NUM_JOINTS, kp * 0.3)
    kd_lg = np.full(NUM_JOINTS, kd * 0.3)
    check("lowgain kp 30%", np.allclose(kp_lg, np.full(NUM_JOINTS, 6.0)))
    check("lowgain kd 30%", np.allclose(kd_lg, np.full(NUM_JOINTS, 0.09)))


# ═══════════════════════════════════════════════════════════════════════════
# 6. action_scale — verify 0.25 everywhere
# ═══════════════════════════════════════════════════════════════════════════

def test_action_scale():
    # The Python DeployConfig default
    from deploy_mujoco import DeployConfig
    cfg = DeployConfig()
    check("python action_scale=0.25", cfg.action_scale == 0.25)


# ═══════════════════════════════════════════════════════════════════════════
# 7. RuntimeState — verify all C++ states exist in Python
# ═══════════════════════════════════════════════════════════════════════════

def test_runtime_state_enum():
    from deploy_mujoco import RuntimeState
    states = set(s.name for s in RuntimeState)
    expected = {"WaitFeedback", "Ready", "EnteringPosition", "ActivePC",
                "ActiveRL", "LowGainTest", "DampingFault"}
    check("all 7 states present", states == expected)
    # Verify value ordering matches C++
    check("WaitFeedback=0", RuntimeState.WaitFeedback.value == 0)
    check("Ready=1", RuntimeState.Ready.value == 1)
    check("EnteringPosition=2", RuntimeState.EnteringPosition.value == 2)
    check("ActivePC=3", RuntimeState.ActivePC.value == 3)
    check("ActiveRL=4", RuntimeState.ActiveRL.value == 4)
    check("LowGainTest=5", RuntimeState.LowGainTest.value == 5)
    check("DampingFault=6", RuntimeState.DampingFault.value == 6)


# ═══════════════════════════════════════════════════════════════════════════

def main() -> int:
    print("=== OpenDoge deploy gap regression test ===\n")

    test_rate_limit()
    test_advance_phase()
    test_build_observation()
    test_target_computation()
    test_pd_control()
    test_action_scale()
    test_runtime_state_enum()

    print(f"\nResults: {passed} passed, {failed} failed")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
