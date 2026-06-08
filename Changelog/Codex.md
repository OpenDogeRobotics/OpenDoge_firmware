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
  -> CPU ONNX policy 50 Hz
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

- 50 Hz policy inference 调度。
- 200 Hz position target hold。
- 1000 Hz EL05 `q/dq/tau/kp/kd` 控制循环。
- 4 路 SocketCAN：`can0..can3`。
- 参考 `mi_motor_demo_TB.py` 的启动流程：设置运控模式、使能、连续发送运控帧。
- EL05 29-bit extended CAN ID 打包。
- EL05 通信类型 2 反馈解析。
- 故障、高温、反馈超时、CAN 异常进入阻尼模式。
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

训练仓库：`/home/lain/OpenDoge/OpenDoge_train`。

部署侧 observation 按训练侧 OpenDoge 配置组织：

```text
commands(3)
base_ang_vel(3)
projected_gravity(3)
dof_pos_delta(12)
dof_vel(12)
last_action(12)
```

单帧 45 维，`frame_stack=6`，ONNX 输入维度为 `270`。

默认站姿：

```text
[0.0, 0.6, -1.5] * 4
```

策略输出：

```text
target_position = default_joint_position + action * 0.30
kp = 12.0
kd = 0.5
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
- 50 Hz 策略输出保持到 200 Hz target。
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
- 电机高温。
- 反馈超时。
- CAN 打开、发送或接收异常。
- 后续如解析到 `faultSta`，也应纳入同一故障闭锁。

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
3. 接入非 ROS IMU 输入，并确认坐标系、重力投影方向。
4. 补齐关节方向、零点 offset、软限位和最终限幅。
5. 单电机验证 EL05 帧、反馈解析和阻尼。
6. 单腿低增益测试。
7. 12 电机低增益站姿测试。
8. 最后启用完整 RL walking policy。
