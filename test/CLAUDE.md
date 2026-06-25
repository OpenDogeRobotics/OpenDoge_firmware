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

---

## Round 2: 行走稳定性修复 (2026-06-25)

Round 1 修复后机器人能站立但 RL 行走仍不如 UniLab pt play 稳定。
CSV 日志分析与训练 XML/代码对比发现以下新 Gap。

### Gap 9: Rate Limiter 限制关节目标速度

**严重程度**: 🔴 致命

#### 问题定位

[deploy_mujoco.py](deploy_mujoco.py) `DeployController.step()` 目标计算块 (200Hz) 中，`rate_limit()` 将关节目标速度限制为 `MAX_POSITION_STEP = 0.015 rad/tick` = **3 rad/s**。

CSV 日志数据证实：RL 推理期间 **90.9%** 的目标更新被 rate_limit 饱和：

```
Rate limit hits: 59001/64884 (90.9%)
```

| 管线 | 目标更新 |
|------|----------|
| 训练 `apply_action` | `ctrl = actions * action_scale + default_angles` — **瞬时到位** |
| 部署 `DeployController.step` | `rate_limit(logical_target, limited_target, 0.015)` — **3 rad/s 平滑** |

策略训练时没有 rate limiter，步态时序依赖瞬时目标切换。部署中大幅动作变化被 rate limiter 延迟 → 策略输出更大动作试图补偿 → rate limiter 进一步饱和 → 脚落地时机错位 → 支撑力不足 → 姿态失稳。

**修复**: RL 模式下跳过 `rate_limit`，位置控制模式保留作为安全保护。

---

### Gap 10: 身体 Mesh 碰撞体与地面碰撞 ~~(已验证为错误假设)~~

**严重程度**: ~~🔴 致命~~ → ⬜ 已排除

#### 假设

训练中身体碰撞 geom `conaffinity="2"` 不与地面碰撞（`conaffinity=2 & ground.contype=1 = 0`），而部署 body mesh 默认 `conaffinity="1"` 会与地面碰撞，产生额外阻力。

#### 实验结论

将 body mesh 统一设 `contype="0" conaffinity="0"` 后，机器人 base 直接穿透地面（z 降至 -0.09m），无法行走。原因是：

- 训练中通过 `<contact>` 传感器**强制**检测脚-地接触，绕过 contype/conaffinity 限制
- 部署中无 contact 传感器，必须依赖自然碰撞匹配（`conaffinity=1`）
- 禁用 body mesh 碰撞后，身体完全失去碰撞检测，base 不受地面约束

**结论**: body mesh 的 `conaffinity="1"` 是部署模型正常运行所必需的，不是 bug。训练模型的碰撞架构（contact 传感器强制配对）不同，不可直接类比。

#### 不做修改

保留 body mesh 默认碰撞行为（`contype="1" conaffinity="1"`）。

---

### Gap 10: 缺少 MuJoCo 物理选项

**严重程度**: 🟡 中等

| 属性 | 训练 (`opendoge.xml`) | 部署 (`Opendoge.xml`) |
|------|----------------------|----------------------|
| `cone` | `elliptic`（椭圆锥摩擦，更精确稳定）| 默认 `pyramidal`（棱锥摩擦）|
| `impratio` | `100`（高求解器精度）| 默认 `1`（低精度）|

**修复**: 在 `Opendoge.xml` 添加 `<option cone="elliptic" impratio="100" />`。

> timestep 保留默认 0.002（部署控制频率 ~434Hz 与训练的 0.01 不同，统一会破坏实时比。训练通过多 substep 达到等效精度，部署高频单步本身已提供足够积分精度）。

---

## Round 2 修复清单

| # | 文件 | 修改内容 |
|---|------|----------|
| 9 | `test/deploy_mujoco.py` | RL 模式跳过 `rate_limit`，目标瞬时切换 |
| 10 | `docs/URDF/xml/Opendoge.xml` | 添加 `<option cone="elliptic" impratio="100" />` |

---

## Round 2 对齐后的最终状态

| 项目 | 训练 | 部署 | 状态 |
|------|------|------|:----:|
| 观测维度 | 49 | 49 | ✅ |
| projected_gravity | framequat 旋转 | framequat 旋转 | ✅ |
| PD 刚度/阻尼 | kp=20, joint damping=0.5 | kp=20, joint damping=0.5 | ✅ |
| joint frictionloss/armature | 0.2 / 0.01 | 0.2 / 0.01 | ✅ |
| action_scale | 0.25 | 0.25 | ✅ |
| 动作裁剪 | 无 | 无 | ✅ |
| 脚部接触参数 | friction 0.4/0.02/0.01, solref 0.01 1, condim 6 | 同 | ✅ |
| 相位 | `freq=1.2+1.3*cmd/0.6`, trot | 同 | ✅ |
| DEFAULT_POS | `{0,0.5,-1.3}` / `{0,0.7,-1.3}` | 同 | ✅ |
| 目标切换速度 | 瞬时 | ActiveRL 瞬时, 其他 rate_limit 3rad/s | ✅ |
| 物理选项 | `cone=elliptic impratio=100` | `cone=elliptic impratio=100` | ✅ |

---

## 验证 (Round 2)

```bash
cd /home/lain/OpenDoge/OpenDoge_firmware/test
python3 deploy_mujoco.py --policy ../policy/opendoge_r5.onnx --log-dir logs
# 按 A (站立) → 等 2s 斜坡 → 按 X (RL 推理) → 按 ↑ 前进
# 观察: 机器人应能持续稳定行走，不掉高、不震颤
```

---

## Round 3: C++/Python 控制回路对齐 + 配置去重 (2026-06-25)

Round 2 后 C++ 固件已模块化 (`controller.cpp`, `observer.cpp`, `safety.cpp`),
但 Python `deploy_mujoco.py` 仍有多处与 C++ 控制回路不一致的逻辑。

### Gap 11: Python build_observation 对速度额外缩放 0.5

**严重程度**: 🔴 致命

Python `build_observation()` 对 `joint_velocities * 0.5`,但 C++ `observer.cpp`
直接使用原始速度 (`logicalVelocity` = `direction * motor_velocity`)。训练观测中
也没有这个 0.5 缩放。

**修复**: 移除 `* 0.5`,j velocities 直接写入 obs[18:30]。

### Gap 12: LowGainTest 状态缺失

**严重程度**: 🟡 中等

C++ `RuntimeState` 有 `LowGainTest` 状态 (30% PD 增益, 保持 DEFAULT_POS),
Python 完全缺失。

**修复**: 添加 `RuntimeState.LowGainTest`, `OperatorCommand.low_gain_mode`,
键盘 L 键绑定, Ready↔LowGainTest 转换, 30% 增益控制行为。

### Gap 13: Python WaitFeedback 未使用

**严重程度**: 🟡 中等

Python `RuntimeState` 枚举中有 `WaitFeedback` 但从未使用 (初始化就进入 `Ready`)。
C++ 启动时进入 `WaitFeedback` 等待所有电机反馈就绪。

**修复**: 初始状态改为 `WaitFeedback`,MuJoCo 中首 tick 即转入 `Ready`。

### Gap 14: inference 仅 ActiveRL 运行

**严重程度**: 🔴 致命

C++ 在所有 active 状态 (ActiveRL, EnteringPosition, ActivePC) 均运行策略推理,
Python 仅在 ActiveRL 运行。

**修复**: 推理条件从 `ActiveRL and rl_inference` 改为 `runtime_state in
(ActiveRL, EnteringPosition, ActivePC) and command.active`。

### Gap 15: 推理失败处理不一致

**严重程度**: 🔴 致命

C++ 中 ActiveRL 推理失败降级到 ActivePC,EnteringPosition/ActivePC 失败进
DampingFault。Python 所有情况下都降级到 ActivePC。

**修复**: 区分 ActiveRL (降级) 和非 ActiveRL (进 DampingFault) 的处理路径。

### Gap 16: EnteringPosition 偏离检测目标不同

**严重程度**: 🔴 致命

C++ `updateStateMachine()` 检查 `abs(pos - default_pos)` 是否超过
`pc_startup_max_deviation`。Python `_check_startup_deviation()` 检查
`abs(pos - limited_target)`。

**修复**: 改为检查 `abs(pos - DEFAULT_POS)`,与 C++ 一致。

### Gap 17: EnteringPosition limited_target 初始化差异

**严重程度**: 🟡 中等

Python 在进入 EnteringPosition 时将 `limited_target` 重置为当前关节位置,
C++ 不重置 (保留之前的值)。

**修复**: 移除 Python 的 `limited_target = current_positions` 快照逻辑。

### Gap 18: rl_fallback_active 未在 ActivePC→ActiveRL 重置

**严重程度**: 🟢 低

C++ `updateStateMachine()` 在 ActivePC→ActiveRL 转换时重置
`rl_fallback_active = false`。Python 缺少此行。

**修复**: 添加 `self.rl_fallback_active = False`。

### Gap 19: LowGainTest 目标计算缺失

**严重程度**: 🟡 中等

C++ `updateTargets()` 在 LowGainTest 时强制 `logical_target = default_pos`。
Python 目标计算块未处理 LowGainTest。

**修复**: 添加 LowGainTest 分支,逻辑目标直接设为 DEFAULT_POS。

---

## 配置去重 (P0-2)

### 问题

`DeployConfig` (24 字段) 和 `SafetyConfig` (13 字段) 所有 13 个字段完全重复。
`main.cpp` 中有 13 行逐字段拷贝代码。`SafetyConfig` 从未独立加载,始终是
`DeployConfig` 的子集拷贝。

### 修复

- 删除 `SafetyConfig` 结构体
- `safetyFault()` 直接接受 `const DeployConfig&`
- `RuntimeState` 枚举和 `JointSafetyState` 从 `safety.hpp` 移至 `types.hpp`
  (同时解决了 P1 级别的 "RuntimeState 定义位置不当" 问题)
- `controller.hpp` 解除对 `safety.hpp` 的 include 依赖
- `main.cpp` 删除 13 行拷贝代码块
- 所有 `safety.X` 访问改为 `config.X`

---

## C++ 配置修复

### action_scale: 0.50 → 0.25

`src/opendoge_deploy/configs/opendoge_deploy.conf` 中的 `action_scale=0.50`
从未更新为训练值 0.25。CLAUDE.md Gap 3 声称已修复但修复仅应用于 Python 侧。
实机部署使用 0.50 (训练值的 2 倍)。

**修复**: C++ 配置文件 `action_scale=0.50` → `0.25`。

---

## 回归测试

新增 `test/deploy_gap_regression.py`: 纯函数回归测试 (无 MuJoCo/ONNX 依赖)。

测试覆盖:
- `rate_limit()` — 速率限制器语义
- `advance_phase()` — 步态相位公式、频率映射、相位折返
- `build_observation()` — **velocity 必须为原始值 (无 *0.5)**,字段顺序,维度
- Target computation — `action_scale=0.25`,RL 跳过 rate_limit,PC 使用 rate_limit,
  关节限位裁剪,LowGainTest 强制默认姿态
- PD 控制 — 阻尼/斜坡/Active/LowGainTest 四种模式的增益数值

运行: `python3 test/deploy_gap_regression.py`

---

## Round 3 对齐后的最终状态

| 项目 | C++ | Python | 状态 |
|------|-----|--------|:----:|
| 状态机 7 状态 | WaitFeedback/Ready/EnteringPosition/ActivePC/ActiveRL/LowGainTest/DampingFault | 同 | ✅ |
| LowGainTest | 30% 增益,强制 DEFAULT_POS | 同 | ✅ |
| WaitFeedback | 启动等待反馈 | 同 (MuJoCo 瞬切) | ✅ |
| 推理条件 | 所有 active 状态 | 同 | ✅ |
| 推理失败处理 | ActiveRL 降级,其他进 DampingFault | 同 | ✅ |
| 观测速度 | 原始值 (direction * motor_velocity) | 原始值 (joint_velocities) | ✅ |
| action_scale | 0.25 (C++ config 修复) | 0.25 | ✅ |
| 偏离检测 | abs(pos - DEFAULT_POS) | 同 | ✅ |
| rl_fallback_active 重置 | 每次离开 ActiveRL 时重置 | 同 | ✅ |
| LowGainTest 目标 | 强制 DEFAULT_POS | 同 | ✅ |
| SafetyConfig/DeployConfig 重复 | 已消除 (SafetyConfig 删除) | N/A | ✅ |
| RuntimeState 位置 | types.hpp | deploy_mujoco.py RuntimeState | ✅ |
| 回归测试 | C++ build + dry-run | deploy_gap_regression.py (53 项) | ✅ |
