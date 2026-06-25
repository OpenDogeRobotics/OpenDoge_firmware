#!/usr/bin/env python3
"""
calc_zero_offset.py — 计算机器人在平地上自然趴伏时的补偿角度。

方法:
  1. 使用与 Opendoge.xml 一致的碰撞配置 (全 mesh 碰撞, thigh 除外)
  2. 从站立 default pose 出发, 关节全失能 (仅阻尼 kd=2.0)
  3. 自由落体 15 秒至稳态
  4. 稳态关节角度即"平地趴伏位", 与 URDF 零位 (全 0) 做差 = 补偿角

用法:
  python3 test/calc_zero_offset.py          # 计算补偿角
  python3 test/calc_zero_offset.py --long   # 30s 长稳

依赖: pip install mujoco numpy
"""

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np

# ─── 路径 ────────────────────────────────────────────────────────────────
REPO_ROOT = Path(__file__).resolve().parent.parent
XML_PATH = str(REPO_ROOT / "docs" / "URDF" / "xml" / "scene.xml")

# ─── 常量 (与 Opendoge.xml / deploy_mujoco.py 一致) ──────────────────────
JOINT_NAMES = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]
NUM_JOINTS = 12

# 初始姿态 (站立 default pose, 用于启动下落)
START_POSE = np.array([
    0.0, 0.5, -1.3,   # FL
    0.0, 0.5, -1.3,   # FR
    0.0, 0.7, -1.3,   # RL
    0.0, 0.7, -1.3,   # RR
])

SIM_DT = 0.002       # 仿真步长
DAMPING = 2.0         # 关节阻尼
DROP_HEIGHT = 0.15    # base 初始离地高度


def resolve_joint_indices(model: mujoco.MjModel):
    """返回 qpos_adr, dof_adr, 以及各关节的 range."""
    qpos_adr = np.zeros(NUM_JOINTS, dtype=int)
    dof_adr = np.zeros(NUM_JOINTS, dtype=int)
    jnt_range = np.zeros((NUM_JOINTS, 2))
    for i, name in enumerate(JOINT_NAMES):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qpos_adr[i] = model.jnt_qposadr[jid]
        dof_adr[i] = model.jnt_dofadr[jid]
        if model.jnt_limited[jid]:
            jnt_range[i] = model.jnt_range[jid]
    return qpos_adr, dof_adr, jnt_range


def run_settling(model, data, qpos_adr, dof_adr, settle_s: float) -> np.ndarray:
    """自由落体 settle_s 秒, 返回稳态关节角度."""
    # 设置阻尼
    for i in range(NUM_JOINTS):
        model.dof_damping[dof_adr[i]] = DAMPING

    # 初始状态
    mujoco.mj_resetData(model, data)
    for i in range(NUM_JOINTS):
        data.qpos[qpos_adr[i]] = START_POSE[i]
    data.qpos[2] = DROP_HEIGHT
    mujoco.mj_forward(model, data)

    total_steps = int(settle_s / SIM_DT)

    for step in range(total_steps):
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)

    return np.array([data.qpos[a] for a in qpos_adr])


def check_limits(final_q, jnt_range):
    """检查关节是否触及限位, 返回警告列表."""
    warnings = []
    for i, name in enumerate(JOINT_NAMES):
        lo, hi = jnt_range[i]
        q = final_q[i]
        eps = 0.01
        if q <= lo + eps:
            warnings.append(f"  {name}: {q:+.4f} ≈ lower limit [{lo:.3f}] ⚠")
        elif q >= hi - eps:
            warnings.append(f"  {name}: {q:+.4f} ≈ upper limit [{hi:.3f}] ⚠")
    return warnings


def print_results(final_q: np.ndarray, base_z: float, ncon: int, jnt_range):
    """打印补偿角结果."""

    # 检查碰撞配置
    print(f"\n{'='*68}")
    print(f"稳态结果 (base_z={base_z:.4f}m, ncon={ncon})")
    print(f"{'='*68}")

    # 关节超限警告
    lim_warnings = check_limits(final_q, jnt_range)
    if lim_warnings:
        print(f"\n关节极限警告:")
        for w in lim_warnings:
            print(w)

    print(f"\n{'关节':<20s} {'URDF零位':>10s} {'稳态值':>10s} {'补偿角':>10s} {'°':>8s}")
    print(f"{'-'*20} {'-'*10} {'-'*10} {'-'*10} {'-'*8}")
    for i, name in enumerate(JOINT_NAMES):
        deg = np.degrees(final_q[i])
        print(f"{name:<20s} {0.0:>10.4f} {final_q[i]:>10.4f} {final_q[i]:>10.4f} {deg:>7.1f}°")

    # 按腿分组
    print(f"\n补偿角按腿 (即 DEFAULT_POS 应设值):")
    leg_params = [("FL", [0, 1, 2]), ("FR", [3, 4, 5]),
                  ("RL", [6, 7, 8]), ("RR", [9, 10, 11])]
    for leg_name, idx in leg_params:
        off = [final_q[i] for i in idx]
        print(f"  {leg_name}: hip={off[0]:+.4f}, thigh={off[1]:+.4f}, calf={off[2]:+.4f}")

    # numpy 数组
    print(f"\n# --- DEFAULT_POS ---")
    print("DEFAULT_POS = np.array([")
    for leg_name, idx in leg_params:
        vals = ", ".join(f"{final_q[i]:.4f}" for i in idx)
        print(f"    {vals},   # {leg_name}")
    print("])")
    print(f"{'='*68}")


def main():
    parser = argparse.ArgumentParser(
        description="OpenDoge 平地趴伏补偿角计算")
    parser.add_argument("--long", action="store_true",
                        help="使用 30s 长稳 (默认 15s)")
    parser.add_argument("--settle", type=float, default=0,
                        help="自定义稳定时长 (秒)")
    args = parser.parse_args()

    if not XML_PATH or not Path(XML_PATH).exists():
        print(f"错误: 找不到模型文件 {XML_PATH}", file=sys.stderr)
        sys.exit(1)

    settle_s = args.settle or (30.0 if args.long else 15.0)
    print(f"模型: {XML_PATH}")
    print(f"碰撞配置: base/hip/calf mesh + 脚球 (thigh mesh visual only)")
    print(f"阻尼: kd={DAMPING}, 稳定时长: {settle_s}s")
    print(f"初始高度: {DROP_HEIGHT}m")

    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    model.opt.timestep = SIM_DT

    qpos_adr, dof_adr, jnt_range = resolve_joint_indices(model)

    print(f"\n自由落体 {settle_s}s ...")
    final_q = run_settling(model, data, qpos_adr, dof_adr, settle_s)

    print_results(final_q, data.qpos[2], data.ncon, jnt_range)


if __name__ == "__main__":
    main()
