# opendoge_deploy — C++ 实机部署模块

## 架构概述

`opendoge_deploy` 是非 ROS 的 C++ 实机 RL 部署运行时。完整链路：

```
daemons/imu_bridge/dm_imu_bridge.py             → /tmp/opendoge_imu.state      → opendoge_deploy 文件轮询
daemons/command_bridge/xbox_command_bridge.py  → /tmp/opendoge_command.state  → opendoge_deploy 文件轮询
opendoge_deploy (C++, SCHED_FIFO)
  → ONNX policy 推理 @ 100 Hz (CPU)
  → 目标位置计算 @ 200 Hz (PD + rate limit)
  → EL05 运控帧发送 @ 1000 Hz (q/dq/tau/kp/kd)
  → SocketCAN can0/can1/can2/can3
  → USB2CAN 信号转发板 → EL05 电机
```

## 文件职责

| 文件 | 职责 | 改动约束 |
|------|------|----------|
| `types.hpp` | 数据类型 (JointMap, MotorState, DeployConfig, ImuSample 等) + 工具函数 | 全局共享，新增字段需同步 `deploy_mujoco.py` |
| `policy.hpp/cpp` | 策略抽象层 (Policy 基类 + makePolicy 工厂) | 只加 backend，不改接口 |
| `onnx_policy.cpp` | ONNX Runtime 策略后端 | 无特殊限制 |
| `el05_socketcan.hpp/cpp` | EL05 私有 CAN 协议层 | 协议常量 (kComm*, kIndex*) 需与 `bringup/el05/el05_motor_menu.py` 保持一致 |
| `runtime_io.hpp/cpp` | 文件 I/O (配置解析、command.state、imu.state) | 格式见下方 |
| `main.cpp` | **主循环 + CLI + 状态机 + 观测构建 + 安全监控 + PD 控制 + JSON 状态输出** (god object) | **重构目标** |

## 当前问题

### main.cpp god object (1047行)

`main.cpp` 包含以下应独立模块化的逻辑：

1. **状态机** (L45-75 枚举定义, L766-842 转换逻辑) → 应拆为 `controller.cpp`
2. **观测构建** (L274-332, `buildObservation()`) → 应拆为 `observer.cpp`
3. **安全监控** (L350-465, `safetyFault()` + `JointSafetyState`) → 应拆为 `safety.cpp`
4. **步态相位** (L261-269, `advancePhase()`) → 应拆为 `gait.cpp`
5. **CLI 解析** (L96-177, `parseArgs()`) → 应拆为 `cli.cpp`
6. **JSON 状态输出** (L970-1028) → 应拆为 `status.cpp`
7. **PD 控制逻辑** (L887-943) → 应拆为 `controller.cpp`

### C++ 与 Python 控制逻辑重复

`test/deploy_mujoco.py` 中的 `DeployController` 类完整复现了 `main.cpp` 的控制回路：
- 状态机转换逻辑 — 两份实现
- 观测构建 (`buildObservation` vs `build_observation`) — 两份实现
- 步态相位 (`advancePhase` vs `advance_phase`) — 两份实现
- rate limiter — 两份实现

任何对控制逻辑的修改必须同步两份代码。当前通过 `test/CLAUDE.md` 记录 Gap 来手动追踪一致性。

### EL05 协议两份实现

| 位置 | 语言 | 用途 |
|------|------|------|
| `src/opendoge_deploy/src/el05_socketcan.cpp` | C++ | 生产部署 |
| `bringup/el05/el05_motor_menu.py` | Python | 硬件 bringup/调测 |
| `bringup/el05/protocol_selftest.py` | Python | 协议自检 |

CAN ID 构造、float→uint 映射范围 (P_MIN, V_MIN, T_MIN 等) 必须三处一致。

### 电机单独调参接口缺失

`opendoge_deploy` 运行时**没有**单电机调参能力。所有电机单独测试必须通过 `bringup/el05/el05_motor_menu.py` 的交互菜单完成。如果需要"在真实控制回路下测试单个电机响应"的功能，需要在 C++ 侧新增模式。

## 配置与数据格式

### command.state 格式

```
vx=0.0
vy=0.0
yaw_rate=0.0
active=false
estop=false
position_control=false
rl_inference=false
clear_fault=false
low_gain_mode=false
```

### imu.state 格式

```
wx=0.0
wy=0.0
wz=0.0
gx=0.0
gy=0.0
gz=-1.0
```

`wx/wy/wz` = 角速度 rad/s, `gx/gy/gz` = projected gravity (已取反 = -upvector)

### 部署配置 (opendoge_deploy.conf)

```
# 控制频率
inference_hz=100
target_hz=200
control_hz=1000

# PD 增益
kp=20.0
kd=0.3
safe_kd=2.0
action_scale=0.50

# 安全阈值
state_timeout_s=0.02
over_temperature_c=80.0
temp_warn_c=65.0
torque_threshold=3.0
torque_timeout_s=0.5
tracking_error_threshold=0.5
tracking_error_timeout_s=0.3
command_timeout_s=0.5
fall_gravity_z_threshold=0.3
fall_timeout_s=0.3
feedback_wait_timeout_s=5.0
command_smoothing_alpha=0.0

# 位置控制斜坡
pc_startup_ramp_s=2.0
pc_startup_max_deviation=0.25

# 故障轮询
fault_poll_hz=10.0

# 每关节校准 (direction offset lower upper max_position_step max_velocity max_torque max_kp max_kd)
joint.FL_hip_joint=1.0,0.0,-0.785,0.26,0.015,20.0,3.0,50.0,5.0
# ... 其余 11 关节
```

### Observation 格式 (49 维, 全部可部署, 无 privileged info)

```
gyro(3) + neg_gravity(3) + dof_pos_diff(12) + dof_vel(12)
+ last_action(12) + commands(3) + feet_phase(4)
```

## 与外部的关系

### 与 test/deploy_mujoco.py 的关系

`test/deploy_mujoco.py` 是 `main.cpp` 控制回路的 Python 级验证仿真器。修改 `main.cpp` 的控制逻辑后，必须同步更新 `deploy_mujoco.py`，反之亦然。关键同步点：

- 状态机转换规则
- `buildObservation()` / `build_observation()` 观测维度、顺序、归一化
- `advancePhase()` / `advance_phase()` 步态公式
- `rateLimit()` / `rate_limit()` 限速语义
- PD 增益逻辑 (阻尼/斜坡/Active/LowGain)
- `last_action` 的 clamp 语义 (RL 模式不 clamp)

### 与 UniLab 训练侧的关系

- `DeployConfig.kp/kd/action_scale` 必须匹配训练侧 `control_config`
- `defaultJointPosition()` 必须匹配训练侧 `scene_flat.xml` keyframe
- 观测结构必须与训练 actor 观测一致 (当前 49 维, 无 linvel)
- ONNX 模型中的 obs_normalizer 使用训练期 running mean/std
- XML 物理参数 (joint damping, foot friction, cone/impratio) 已对齐 → 见 `test/CLAUDE.md`

### 与 daemons / bringup 的关系

`daemons/` 是**实机硬件 I/O 守护进程**，负责将物理设备桥接为文件 IPC：

```
daemons/imu_bridge/dm_imu_bridge.py             → 串口读数 → /tmp/opendoge_imu.state
daemons/command_bridge/xbox_command_bridge.py  → HID 输入 → /tmp/opendoge_command.state
```

`opendoge_deploy` 仅通过文件轮询读取这些数据，不直接操作硬件。这是**有意设计**：
- 让 C++ 实时循环免于处理串口协议、HID 设备
- 守护进程可以独立重启而不影响主控制回路
- 文件格式简单，方便 dry-run 测试时手动注入数据

`bringup/el05/` 是 bringup/标定工具，在部署前使用，运行时不需要。

## 安全状态机

```
WaitFeedback → Ready ↔ EnteringPosition → ActivePC ↔ ActiveRL
                   ↕                      ↓ (fault)
              LowGainTest            DampingFault
```

故障触发条件 (进入 `DampingFault`):
- 电机反馈丢失/超时
- 电机故障位 (`fault` / `faultSta`) 非 0
- 电机过温 (> `over_temperature_c`)
- 持续力矩超限 (> `torque_timeout_s`)
- 持续跟踪误差 (> `tracking_error_timeout_s`)
- CAN 读写异常
- command 文件 `estop=true`
- IMU 倒地检测 (`projected_gravity.z < fall_gravity_z_threshold` 持续 > `fall_timeout_s`)
- IMU 持续无效 (> `imu_debounce_count` 帧)
- EnteringPosition 阶段关节偏差超限

故障恢复: `clear_fault=true` → 重新发送 stop(clear_fault=1) + motion_mode + enable → WaitFeedback

## 修改指南

1. **新增配置参数**: 在 `DeployConfig` + `SafetyConfig` (`types.hpp`) 中加字段 → `runtime_io.cpp` 的 `loadDeployConfig()` 增加解析 → `main.cpp` 传入对应的 safety 结构体
2. **新增状态**: 在 `RuntimeState` 枚举添加 → 所有 switch(state) 处增加处理
3. **修改观测格式**: `buildObservation()` + `deploy_mujoco.py` 的 `build_observation()` 同步修改，确保维度为 `kObsDim`
4. **修改 CAN 协议**: `el05_socketcan.cpp` + `bringup/el05/el05_motor_menu.py` 同步修改
5. **修改控制回路**: `main.cpp` 的控制块 (L887-943) + `deploy_mujoco.py` 的 `DeployController.step()` 同步修改
6. **新增 policy backend**: 实现 `Policy` 接口 → 在 `makePolicy()` 注册

## 电机映射 (不可变)

```
can0: FL_hip/motor 1, FL_thigh/motor 2, FL_calf/motor 3
can1: FR_hip/motor 4, FR_thigh/motor 5, FR_calf/motor 6
can2: RL_hip/motor 7, RL_thigh/motor 8, RL_calf/motor 9
can3: RR_hip/motor 10, RR_thigh/motor 11, RR_calf/motor 12
```

## 默认站立姿态 (必须匹配训练 keyframe)

```
前腿 (FL, FR): hip=0, thigh=0.5, calf=-1.3
后腿 (RL, RR): hip=0, thigh=0.7, calf=-1.3
```
