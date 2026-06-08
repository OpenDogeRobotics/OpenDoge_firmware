# 2026-06-08 强化学习部署状态记录

本文固化当前对 `/home/lain/OpenDoge/OpenDoge_firmware` 的判断：它目前可以作为
OpenDoge 强化学习部署框架的骨架，但还不是完整的真机闭环控制栈。

## 总结

当前工作区更接近 ROS 2 bringup + RL 节点骨架，尚未形成完整的实机强化学习控制链路。

直接使用 CPU ONNX 推理是合理方向，但仓库里还没有 ONNX 后端。当前更关键的缺口是：
策略输出到 EL05 电机 CAN 运控帧之间的正式电机控制路径还没有打通。

## 当前已有内容

- ROS 2 workspace 结构，包含 description、bringup、control 配置、RL 节点和 EL05/USB2CAN 工具。
- `opendoge_rl_node` 订阅：
  - `/joint_state`
  - `/imu`
  - `/joy`
  - `/cmd_vel`
- `opendoge_rl_node` 发布：
  - `/joint_target`
  - `/rl_observation`
  - `/rl_action`
- 已有 RL 频率参数：
  - `inference_rate_hz=50`
  - `publish_rate_hz=200`
  - `control_rate_hz=500`
- `rl_node.cpp` 中已经实现 observation 构造。
- EL05 Python 调试工具中已有 RobStride 私有 29-bit 扩展 CAN 协议的 SocketCAN 打包和解析参考。

## 作为实机 RL 部署框架还缺什么

### 策略运行时

- 还没有 CPU ONNX 后端。
- `policy_backend=none` 只保持默认关节位置。
- `policy_backend=linear_csv` 只是管线测试后端。
- `policy_backend=rknn` 目前只是占位。

建议下一步：

- 新增 `policy_backend=onnx`，使用 ONNX Runtime C++。
- 启动时加载 `policy_path`。
- 预分配输入/输出 tensor。
- 用训练侧导出的测试向量校验部署侧 action 数值。

### 电机硬件接口

当前配置引用的硬件插件不在此 workspace 中：

- 配置插件：`motor_control_interface/MotorHardware`
- 引用位置：
  - `src/opendoge_control/config/ros2_control.yaml`
  - `src/opendoge_description/urdf/opendoge_apx.urdf.xacro`

建议下一步：

- 实现或引入 `hardware_interface::SystemInterface` 插件。
- 增加 C++ SocketCAN 支持。
- 将 Python EL05 工具中的帧打包/解析逻辑迁移到生产级 C++ 代码。
- 实现 `read()` 电机反馈和 `write()` 电机命令。

### 控制器

当前配置引用的控制器不在此 workspace 中：

- 配置控制器：`robot_joint_controller/RobotJointControllerGroup`
- 引用位置：`src/opendoge_bringup/config/controllers.yaml`

建议下一步：

- 实现或引入该控制器，或者替换为命令语义清晰的标准/自定义控制器。

### 命令语义

当前命令链路语义不完整：

- `ros2_control.yaml` 只声明了 `effort` command interface。
- EL05 运控模式需要 `q/dq/tau/kp/kd`。
- `opendoge_rl_node` 当前发布 `sensor_msgs/msg/JointState`：
  - `position` = 目标位置
  - `velocity` = 0
  - `effort` = `kp`
- `kd` 和 torque 命令没有通过 `/joint_target` 传下去。

建议下一步：

- 不要依赖 `JointState.effort` 偷塞 `kp`。
- 定义真实的 `q/dq/tau/kp/kd` 命令通路。
- 中期建议使用 ros2_control 多 command interface：
  - `position`
  - `velocity`
  - `effort`
  - `kp`
  - `kd`

### 标定和安全

仍然缺少或只是 placeholder 的内容：

- 真实 motor id。
- CAN 通道映射。
- 关节方向符号。
- 关节零点 offset。
- 软件限位。
- torque、velocity、Kp、Kd 限幅。
- 温度/故障位解析和处理策略。
- 硬件层 watchdog。
- 命令超时后的安全回退。
- CAN error 和 bus-off 恢复。
- RL 节点本地 mode flag 之外的完整急停链路。

## 滤波状态

### IMU / 陀螺仪滤波

当前 RL 节点没有对 IMU 数据做滤波。

当前行为：

- quaternion 直接从 `/imu` 拷贝。
- angular velocity 直接从 `/imu` 拷贝。
- RPY 直接由内部保存的 quaternion 计算。
- observation 做了 clamp，但 clamp 不是滤波。

建议：

- 优先使用 IMU 驱动提供的融合 quaternion 和硬件/驱动 DLPF。
- 如果角速度噪声明显，可以对 angular velocity 加轻量一阶低通。
- 截止频率不能太低，否则相位延迟会影响步态稳定。
- 必须和训练分布一致：实机滤波、仿真观测噪声和延迟要匹配。

### 电机输出滤波

当前 RL 节点没有真正的电机输出滤波。

当前行为：

- action 做 clamp。
- action 做 scale。
- target position clamp 到关节限位。
- 50 Hz 策略输出被保持，并以 200 Hz 发布。
- 没有插值、限速、限加速度或低通平滑。

建议：

- 增加目标位置变化率限制。
- 在 controller/hardware 层增加 velocity、torque、Kp、Kd 限幅。
- 可以考虑将 50 Hz 策略目标短插值到 500 Hz 电机控制目标。
- 不建议随意加重低通，除非训练时也包含同等延迟；否则可能 destabilize gait。

## CPU ONNX 推理计划

CPU ONNX 推理可行，前提是实测推理时间稳定低于 50 Hz 策略周期的 20 ms。

建议实现步骤：

1. 给 `opendoge_rl_node` 增加 ONNX Runtime 依赖。
2. 增加 `policy_backend=onnx`。
3. 节点初始化时只加载一次 ONNX 模型。
4. observation 构造必须和训练环境完全一致。
5. 只在 `inference_rate_hz` 节奏下运行推理。
6. 沿用 action clamp 和 scale。
7. 发布 debug action 和 observation，方便与训练侧 replay 对齐。
8. 实测推理延迟和端到端控制延迟。

建议先用 CPU ONNX 跑通并完成数值校验，再考虑 RKNN/NPU 优化。

## 电机控制路径

预期生产路径：

```text
RL node -> target command -> ros2_control controller -> MotorHardware
  -> SocketCAN(can0/can1/...) -> USB2CAN 信号转发板 -> EL05 CAN bus
```

EL05 工具已经确认底层方向：

- 使用 SocketCAN raw socket。
- 使用 29-bit extended CAN ID。
- 使用 RobStride 私有协议。
- 运控命令携带 `q/dq/tau/kp/kd`。

当前仓库状态：

- Python EL05 工具可以用于单电机 bringup 和协议参考。
- ROS 生产电机路径必须补齐 C++ hardware/controller 后才完整。

## ros2_control 和延迟

当前设计意图是使用 `ros2_control`：

- `bringup.launch.py` 启动 `controller_manager/ros2_control_node`。
- launch 会 spawn `robot_joint_controller`。
- `ros2_control.yaml` 设置 `update_rate: 500`。

延迟判断：

- `ros2_control` 会引入调度和 controller 周期延迟。
- 500 Hz 下一个控制周期是 2 ms。
- 正确实现时，这个量级通常可以接受。
- 更大的风险通常是 Linux 调度抖动、ROS topic 排队、USB2CAN/CAN 总线序列化和总线负载。

建议：

- 500 Hz 高频电机 read/write 放在 controller/hardware loop 内部。
- 不要通过普通 ROS topic 发送 500 Hz 逐电机命令。
- ROS topic 只承载 50/200 Hz 策略级 target。
- 在改架构前先用时间戳实测 jitter。
- 如果 CAN 负载过高，拆分多路 CAN，或在可接受范围内降低反馈/命令频率。

## 建议推进顺序

1. 在 `opendoge_rl_node` 实现 CPU ONNX 后端。
2. 增加训练导出 replay 测试，校验 observation/action 数值一致。
3. 明确定义 `q/dq/tau/kp/kd` 命令语义。
4. 将 EL05 协议代码迁移到 C++。
5. 实现 `motor_control_interface/MotorHardware`。
6. 实现或替换 `robot_joint_controller`。
7. 增加 hardware/controller watchdog 和安全限幅。
8. 先验证单电机 SocketCAN。
9. 再验证单腿。
10. 再以低增益、低 action scale 验证 12 电机。
11. 最后再启用完整 RL walking policy。

## 当前结论

CPU ONNX 是合适的下一步。当前仓库还没有完整实时电机控制链路。最关键的工程工作是：
生产级 EL05 C++ 硬件接口、明确的 `q/dq/tau/kp/kd` 命令链路、安全/watchdog 层，以及训练侧
observation 与部署侧 ONNX 推理之间的数值一致性验证。
