# 2026-06-22 UniLab Round 25 部署对齐

部署框架重构为匹配当前 UniLab 训练管线 (Round 25)：

- Observation 从 270 维 (45×6 frame_stack) 迁移到 **52 维单帧**
- Observation 排序匹配 `_compute_obs`: gyro + neg_gravity + diff + dof_vel + action + commands + phase + linvel
- 新增自适应步态相位计算 (cmd_speed → freq ∈ [1.2, 2.5] Hz)
- 新增 linvel 占位 (后续可替换为估计器)
- 移除命令缩放 (vx×2, vy×2, vyaw×0.25 → 原始值)
- PD 增益更新: kp 12→20, kd 0.5→0.3
- action_scale 更新: 0.30→0.50
- ONNX 验证输入维度更新: 270→52
- 策略模型: `policy/opendoge_r25.onnx` (UniLab Round 25, best 143.40 / final 109.98)
- 配置文件关节软限位更新为 URDF 物理约束

# 2026-06-08 OpenDoge 强化学习部署记录

本文固化 `/home/lain/OpenDoge/OpenDoge_firmware` 当前状态：OpenDoge 的实机强化学习部署主路径已经改为非 ROS 单进程运行时。

## 当前结论

CPU ONNX + SocketCAN + EL05 运控帧是主方向。强化学习模型部署、电机控制闭环和安全保护不需要 ROS，也不应强绑定 ROS。

已删除冗余 ROS 控制路径：

- `src/opendoge_bringup`
- `src/opendoge_control`
- `src/opendoge_rl_node`
- 空的 `src/motor_control_interface`
- 空的 `src/robot_joint_controller`
- URDF 中的 `ros2_control` / `MotorHardware` 插件声明

当前 `src/` 只保留：

- `opendoge_deploy`：非 ROS 实机部署主程序。

真实 URDF 使用 `/home/lain/OpenDoge/OpenDoge_description/URDF`。Firmware 内原先的占位 `opendoge_description` 也已删除，避免把简化描述误当成实机模型。

## 主部署架构

```text
opendoge_deploy
  -> CPU ONNX policy 100 Hz
  -> position target hold 200 Hz
  -> EL05 motion command 1000 Hz
  -> SocketCAN can0/can1/can2/can3
  -> USB 转 4 路 CAN2.0 模块
  -> EL05 motors
```

`opendoge_deploy` 不依赖：

- ROS
- rclcpp
- ros2_control
- controller_manager
- hardware_interface
- controller_interface

这样可以减少高频闭环中的调度层级，1000 Hz 电机循环不经过 ROS topic、executor 或 controller manager。

## 已实现内容

`src/opendoge_deploy` 已实现：

- 100 Hz policy inference 调度，与训练侧 200 Hz 仿真 dt、decimation=2 对齐。
- 200 Hz position target hold。
- 1000 Hz EL05 `q/dq/tau/kp/kd` 控制循环。
- `wait_feedback -> ready -> active -> damping_fault` 显式状态机。
- 4 路 SocketCAN：`can0..can3`。
- 参考 `mi_motor_demo_TB.py` 的启动流程：写入 `0x7005 run_mode=0`、使能、连续发送运控帧。
- EL05 29-bit extended CAN ID 打包。
- EL05 通信类型 2 反馈解析。
- `faultSta` 参数轮询和解析。
- 故障、高温、反馈超时、CAN 异常、急停进入阻尼模式。
- 每秒输出 loop/CAN 统计：控制 tick、推理 tick、target tick、最大控制延迟、missed deadline、CAN 收发和错误计数。
- 可选实时性设置：`--realtime` 尝试 `mlockall` + `SCHED_FIFO`，`--cpu N` 绑定 CPU。
- 非 ROS 命令文件输入：`vx/vy/yaw_rate/active/estop`。
- 非 ROS IMU 文件输入：`wx/wy/wz/gx/gy/gz`。
- 配置化关节方向、零点 offset、软限位和 position target 限速。
- 无硬件协议自检：`tools/el05/protocol_selftest.py`。
- vcan 启动脚本：`scripts/setup_vcan.sh`。
- 可选 ONNX Runtime 后端；未安装 ONNX Runtime 时仍可构建 dry-run 后端。

当前构建验证：

```bash
colcon build --symlink-install --packages-select opendoge_deploy
./install/opendoge_deploy/bin/opendoge_deploy --policy-backend none --duration-sec 1
```

## 电机和 CAN 映射

硬件使用 USB 转 4 路 CAN2.0 满速模块。每路 1 kHz 带 3 个 EL05 电机，带宽余量足够。

```text
can0 = 左前 FL: motor 1/2/3   = hip/thigh/calf
can1 = 右前 FR: motor 4/5/6   = hip/thigh/calf
can2 = 左后 RL: motor 7/8/9   = hip/thigh/calf
can3 = 右后 RR: motor 10/11/12 = hip/thigh/calf
```

关节顺序：

```text
FL_hip_joint, FL_thigh_joint, FL_calf_joint,
FR_hip_joint, FR_thigh_joint, FR_calf_joint,
RL_hip_joint, RL_thigh_joint, RL_calf_joint,
RR_hip_joint, RR_thigh_joint, RR_calf_joint
```

部署侧、真实 URDF 和训练侧统一使用 `hip/thigh/calf`，不使用 `knee`。

## RL Observation

训练仓库：`/home/lain/UniLab` (UniLab PPO pipeline)。

部署侧 observation 匹配 UniLab `_compute_obs` 输出（52 维单帧，无 frame stacking）：

```text
gyro(3)
neg_gravity(3) — IMU projected_gravity（已是"下"方向 = -upvector）
dof_pos_delta(12) — 关节位置 − default_angles
dof_vel(12)
last_action(12)
commands(3) — vx, vy, vyaw（原始值，无需缩放）
feet_phase(4) — 自适应步态相位 (FL/FR/RL/RR)
linvel(3) — 局部线速度（当前为零占位）
```

单帧 52 维，ONNX 输入维度为 `52`。ONNX 模型内部包含 obs_normalizer (Sub+Div)。

自适应相位计算：

```
cmd_speed = norm([vx, vy, vyaw])
freq = clamp(1.2 + 1.3 * cmd_speed / 0.6, 1.2, 2.5)
phase += ctrl_dt * freq  (mod 1.0)
feet_phase = [phase, (phase+0.5)%1, (phase+0.5)%1, phase]
```

默认站姿：

```text
[0.0, 0.6, -1.5] * 4
```

策略输出：

```text
target_position = default_joint_position + action * 0.50
kp = 20.0
kd = 0.3
```

## 滤波判断

当前部署代码不做 IMU 软件滤波，也不做电机输出低通滤波。

理由：

- 训练侧策略通常期望部署侧 observation 和 action 处理尽量一致。
- 低通滤波会引入相位滞后，可能影响步态闭环。
- 电机输出更适合做限幅、限速、限加速度，而不是无脑低通。

当前保留的处理：

- action clamp。
- action scale。
- target position clamp。
- 100 Hz 策略输出保持到 200 Hz target。
- 1000 Hz 电机循环消费最新 target。

IMU 如果后续实测性能差，建议在 IMU 驱动或硬件侧单独处理，再重新校验训练/部署一致性。

## 电机控制是否可行

可行。依据：

- `mi_motor_demo_TB.py` 已给出可用的 SocketCAN 控制参考。
- EL05 支持运控模式 `q/dq/tau/kp/kd`。
- 四路 CAN 分摊 12 电机，每路 3 电机，1 kHz 控制频率带宽足够。
- 当前 C++ 部署程序已经直接实现同类 CAN 打包、发送和反馈解析。

EL05 手册范围：

```text
position: -12.57..12.57 rad
velocity: -50..50 rad/s
torque:   -6..6 Nm
kp:       0..500
kd:       0..5
```

上机前仍必须确认：

- 关节方向符号。
- 机械零点 offset。
- 软件限位。
- 低增益单电机和单腿测试。
- ONNX 策略 action 是否和训练侧 replay 一致。

## 安全保护

触发以下任一条件时进入阻尼模式：

- 通信类型 2 反馈故障位非 0。
- `faultSta` 参数非 0。
- 电机高温。
- 反馈超时。
- CAN 打开、发送或接收异常。
- 命令输入 `estop=true`。
- 命令/IMU 输入解析失败。

故障输出以十六进制显示，并列出置位 bit，避免 `0x` 后接十进制数造成误判。

阻尼输出：

```text
position = 当前反馈位置
velocity = 0
torque = 0
kp = 0
kd = safe_kd
```

部署程序不自动执行通信类型 `0x06` 机械置零。置零会改变电机参考零点，必须由单电机工具人工确认后执行。

## ROS / ros2_control 处理

当前不走 ros2_control。

原因：

- RL 实机部署核心是一个实时控制程序，不需要 ROS 才能运行。
- 1000 Hz 电机控制不应穿过 ROS topic、controller manager 和 executor。
- 单进程内更容易控制线程优先级、ONNX Runtime 线程数、CAN 时间戳、故障闭锁和退出阻尼。

后续如果需要 ROS，只建议作为低频 bridge：

- 发布 debug state。
- 发布 observation/action 记录。
- 接收遥控或模式切换。
- 做可视化。

ROS bridge 不应参与 1000 Hz 电机控制闭环。

## 后续必须完成

1. 安装 ONNX Runtime C/C++，启用 `policy_backend=onnx`。
2. 用训练侧导出的 observation/action 测试向量做数值 replay。
3. 将 IMU 文件输入替换/接入真实非 ROS IMU 驱动，并确认坐标系、重力投影方向。
4. 将配置文件中的关节方向、零点 offset、软限位和最终限幅改成实测值。
5. 单电机验证 EL05 帧、反馈解析和阻尼。
6. 单腿低增益测试。
7. 12 电机低增益站姿测试。
8. 最后启用完整 RL walking policy。
