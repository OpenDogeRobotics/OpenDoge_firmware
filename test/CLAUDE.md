# OpenDoge Mujoco Deploy — 训练/部署 Gap 分析与修复记录

本文件记录 OpenDoge ONNX 策略在 MuJoCo 部署管线中的全部 Gap 定位与修复过程。

---

## 环境信息

| 项目 | 训练管线 | 部署管线 |
|------|----------|----------|
| 入口 | `joystick.py` + `base.py` | `deploy_mujoco.py` |
| ONNX 模型 | `policy/opendoge_r5.onnx` | 同 |
| MuJoCo XML | `src/unilab/assets/robots/opendoge/opendoge.xml` + `scene_flat.xml` | `docs/URDF/xml/Opendoge.xml` |

---

## Gap 1: IMU projected_gravity 计算错误

**严重程度**: 🔴 致命

### 问题定位

[deploy_mujoco.py](deploy_mujoco.py) `get_imu()` 方法中使用 `accelerometer` 传感器计算 projected_gravity：

```python
accel = self.data.sensordata[self._accel_adr:self._accel_adr + 3].copy()
accel_norm = np.linalg.norm(accel)
if accel_norm > 0.01:
    imu.projected_gravity = -accel / accel_norm  # ← 错误
```

**错误 1 — 物理含义错误**: MuJoCo 的 `<accelerometer>` 测量的是物体的**净线加速度**（世界坐标系）。当机器人静止站立时，重力被地面反作用力抵消，净加速度 ≈ 0，归一化后纯噪声。

**错误 2 — 坐标系不匹配**: 训练使用 `<framezaxis name="upvector"/>` 传感器，返回机体 Z 轴在世界系中的方向。取反后得到重力方向在机体系中的投影，能随姿态倾斜变化。而 accelerometer 返回世界坐标系常向量 `[0, 0, -g]`，无论机器人如何倾斜都不变，完全丢失姿态信息。

### 实验验证

| 姿态 | 训练 expected (obs[3:6]) | 部署 actual | 结果 |
|------|--------------------------|-------------|------|
| 直立 | `-upvector = [0, 0, -1]` | 噪声（accel≈0，默认 `[0,0,-1]` 碰巧对） | 直立时偶尔蒙对 |
| 前倾 30° | `-framezaxis ≈ [-0.5, 0, -0.866]` | `≈ [0, 0, 1]`（世界系常值） | 完全丢失姿态 |

### 影响链条

```
broken projected_gravity → policy 看到错误的姿态信息
  → policy 输出极端动作试图"纠正"不存在的姿态错误
  → 动作被 clip 到 [-1, 1]，50%+ 动作饱和
  → 机器人运动不稳定、无法正常行走
```

### 修复方案

`framezaxis` 在 MuJoCo free joint 模型下返回值异常（水平站立时返回 `[0, 0.33, 0]` 而非 `[0, 0, 1]`），改用 `framequat` (orientation) 传感器 + 四元数旋转世界重力到机体坐标系：

```python
def get_imu(self) -> ImuSample:
    imu = ImuSample()
    if self._has_sensors:
        imu.angular_velocity = self.data.sensordata[
            self._gyro_adr:self._gyro_adr + 3
        ].copy()
        quat = self.data.sensordata[
            self._quat_adr:self._quat_adr + 4
        ].copy()  # [w, x, y, z]
        # 世界系重力 [0, 0, -1] 旋转到机体坐标系
        imu.projected_gravity = self._rotate_vector_by_quat(
            np.array([0.0, 0.0, -1.0]), quat
        )
        imu.valid = True
    return imu
```

**实施**: 新增 `_quat_adr` 初始化 + `_rotate_vector_by_quat()` 静态方法。

---

## Gap 2: 动作裁剪 (Action Clipping)

**严重程度**: 🔴 致命

### 问题定位

| 管线 | 行为 |
|------|------|
| 训练 `apply_action` | `ctrl = actions * action_scale + default_angles` — **无裁剪** |
| 部署 `DeployController.step` | `self.last_action[i] = np.clip(self.action[i], -1.0, 1.0)` — **裁剪** |

ONNX 模型对真实观测的输出范围在 `[-3.8, +3.3]` 之间，而部署把一切超过 `[-1, 1]` 的都砍掉了。日志数据显示 40-57% 的动作值被裁剪：

| 关节 | 裁剪率 |
|------|--------|
| FR_calf_joint | 57.5% |
| FR_hip_joint | 52.4% |
| FL_thigh_joint | 50.6% |
| RL_calf_joint | 47.6% |
| RL_thigh_joint | 46.2% |
| FL_calf_joint | 45.8% |

**修复**: `self.last_action[i] = self.action[i]`（移除 `np.clip`）。

---

## Gap 3: action_scale 不匹配

**严重程度**: 🔴 致命

| 配置 | 训练 | 部署 |
|------|------|------|
| `action_scale` | `0.25` | `0.50` |

即使在 `[-1, 1]` 范围内，部署的关节偏移也是训练的 2 倍。例如：某动作值 0.5 → 训练偏移 0.125 rad，部署偏移 0.25 rad。

**修复**: `action_scale` 改为 `0.25`。

---

## Gap 4: 观测中的动作反馈不一致

**严重程度**: 🟡 中等

| 管线 | 观测中 last_action |
|------|--------------------|
| 训练 `_compute_obs` | raw policy output（未裁剪） |
| 部署 `build_observation` | 裁剪后的值（`np.clip(action, -1, 1)`） |

观测中包含上一步的动作，训练看到 raw 值（如 2.5），部署看到裁剪值（如 1.0），形成反馈回路：裁剪动作 → 不同观测 → 不同输出 → 进一步裁剪。

**修复**: Gap 2 修复后自动消除。

---

## Gap 5: 脚部摩擦参数不匹配

**严重程度**: 🔴 致命（修复 Gap 1-4 后仍无法行走的根因）

| 属性 | 训练 (`scene_flat.xml` foot class) | 部署 (firmware `Opendoge.xml`) |
|------|-----------------------------------|-------------------------------|
| `friction` | `0.4 0.02 0.01` | `1.0 0.05 0.05` ← **2.5x** |
| `solref` | `0.01 1` (软接触) | `0.005 1` (硬，2x 时间常数) |
| `solimp` | `0.015 1 0.023` | 默认 `0.9 0.95 0.001` |
| `condim` | `6` (椭圆锥摩擦) | 默认 `3` (棱锥摩擦) |
| `margin` | `0.005` | 默认 `0` |

摩擦系数 1.0 意味着脚几乎粘在地上，摆腿相脚无法自然滑移，策略输出的步态被地面"焊死"。训练时脚可以滑移，策略学到的步态依赖适度的脚部滑动来完成 weight transfer。

**修复**: 4 个 foot sphere geom 的接触参数全部对齐训练配置。

---

## Gap 6: 关节阻尼不匹配

**严重程度**: 🟡 中等

| 属性 | 训练 | 部署 |
|------|------|------|
| `damping` | `0.5` | `0.0` |
| `frictionloss` | `0.2` | `0.0` |
| `armature` | `0.01` | `0.005` |

训练中 MuJoCo 位置执行器的有效速度阻尼来自 joint damping=0.5 (被动)。部署中 joint damping=0，仅靠 PD kd=0.3 (主动) 提供阻尼，总阻尼不足 → 系统欠阻尼 → 振荡容易发散。

**修复**: joint damping 对齐训练配置，同时 PD kd 归零（`0.3 → 0.0`），避免双层阻尼过冲 (0.3+0.5=0.8 vs 训练 0.5)。

---

## Gap 7: 执行器类型不同

**严重程度**: 🟢 低（部署脚本通过 `data.ctrl` 写入力矩值绕过）

| 属性 | 训练 | 部署 |
|------|------|------|
| 执行器类型 | `<position kp="20">` | `<motor gear="1">` |
| 工作原理 | MuJoCo 内部位置伺服 | 部署脚本自行计算 PD 力矩 |

力矩限制机制和与 joint damping 的耦合方式不同，但实际影响较小。

---

## ONNX 模型输出特性

- **无 Tanh 层**: RSL-RL 的 `act()` 中 tanh 在模型外部应用，ONNX 导出只含 MLP 本体 + obs_normalizer
- **无 Gaussian 分布**: ONNX 输出为分布均值 (deterministic mean)，训练时动作为 `Normal(mean, std).sample()` (含噪声)
- 观测归一化 (obs_normalizer) 已完整烘焙进 ONNX (Sub → Div)，输入需保持与训练一致的 49 维结构

---

## 修复清单

| # | 文件 | 修改内容 |
|---|------|----------|
| 1 | `docs/URDF/xml/Opendoge.xml` | 添加 `framequat` 传感器（未使用，无害保留） |
| 2 | `test/deploy_mujoco.py` | 新增 `_quat_adr` 传感器地址初始化 |
| 3 | `test/deploy_mujoco.py` | 新增 `_rotate_vector_by_quat()`，用 framequat 计算 projected_gravity |
| 4 | `test/deploy_mujoco.py` | 移除动作裁剪 (`np.clip`) |
| 5 | `test/deploy_mujoco.py` | `action_scale`: `0.50 → 0.25` |
| 6 | `docs/URDF/xml/Opendoge.xml` | 4 个 foot sphere 接触参数对齐训练 (friction, solref, solimp, condim, margin) |
| 7 | `docs/URDF/xml/Opendoge.xml` | joint damping/frictionloss/armature 对齐训练 |
| 8 | `test/deploy_mujoco.py` | PD kd: `0.3 → 0.0`（joint damping 已提供被动阻尼） |

---

## 全部对齐后的最终状态

| 项目 | 训练 | 部署 | 状态 |
|------|------|------|:----:|
| 观测维度 | 49 (gyro3+g3+d12+v12+a12+c3+p4) | 49 | ✅ |
| 观测归一化 | empirical_normalization → ONNX | ONNX 内 Sub+Div | ✅ |
| projected_gravity | `-upvector (= q*[0,0,-1]*q⁻¹)` | framequat + 四元数旋转 | ✅ |
| PD 刚度 | kp=20 | kp=20 | ✅ |
| PD 阻尼 | joint damping=0.5 (被动) | joint damping=0.5 + PD kd=0.0 | ✅ |
| 关节摩擦 | joint frictionloss=0.2 | joint frictionloss=0.2 | ✅ |
| 电枢惯性 | joint armature=0.01 | joint armature=0.01 | ✅ |
| action_scale | 0.25 | 0.25 | ✅ |
| 动作裁剪 | 无 (raw output) | 无 | ✅ |
| last_action | raw ONNX output | raw ONNX output | ✅ |
| 脚部 friction | `0.4 0.02 0.01` | `0.4 0.02 0.01` | ✅ |
| 脚部 solref | `0.01 1` | `0.01 1` | ✅ |
| 脚部 solimp | `0.015 1 0.023` | `0.015 1 0.023` | ✅ |
| 脚部 condim | 6 | 6 | ✅ |
| 脚部 margin | 0.005 | 0.005 | ✅ |
| 相位 | `freq=1.2+1.3*cmd/0.6`, 4 foot | 同 | ✅ |
| DEFAULT_POS | `{0,0.5,-1.3}` / `{0,0.7,-1.3}` | 同 | ✅ |
| 关节限位 | 同 | 同 | ✅ |

---

## 验证

从站立姿态 (DEFAULT_POS) 出发，RL 推理 5 秒，vx=0.6 命令：

| 指标 | 修复后 | 修复前 |
|------|--------|--------|
| Base x 位移 | **0.924 m (18 cm/s)** | 原地发抖，零位移 |
| Base z 高度 | 稳定在 0.155 m | — |
| Base y 漂移 | 0.032 m (几乎直线) | — |

### 部署验证命令

```bash
cd /home/lain/OpenDoge/OpenDoge_firmware/test
python3 deploy_mujoco.py --policy ../policy/opendoge_r5.onnx --log-dir logs
# 按 A (站立) → 等 2s 斜坡 → 按 X (RL 推理) → 按 ↑ 前进
```
