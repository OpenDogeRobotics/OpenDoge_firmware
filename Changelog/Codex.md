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
  - `control_rate_hz=1000`
- `rl_node.cpp` 中已经按 OpenDoge_train 的单帧 45 维 observation 格式组织部署侧输入：
  - command(3)
  - base angular velocity(3)
  - projected gravity(3)
  - dof position delta(12)
  - dof velocity(12)
  - last action(12)
- `frame_stack=6`，部署侧 ONNX 输入维度为 `45 * 6 = 270`。
- EL05 Python 调试工具中已有 RobStride 私有 29-bit 扩展 CAN 协议的 SocketCAN 打包和解析参考。
- 已有 USB 转 4 路 CAN2.0 满速模块参考，`mi_motor_demo_TB.py` 中明确使用 `can0`、`can1`、`can2`、`can3`，每路配置为 `bitrate 1000000`。
- 当前电机 ID 和关节顺序已经明确为 1 到 12：
  - `can0` 左前：`1=FL_hip_joint`，`2=FL_thigh_joint`，`3=FL_calf_joint`
  - `can1` 右前：`4=FR_hip_joint`，`5=FR_thigh_joint`，`6=FR_calf_joint`
  - `can2` 左后：`7=RL_hip_joint`，`8=RL_thigh_joint`，`9=RL_calf_joint`
  - `can3` 右后：`10=RR_hip_joint`，`11=RR_thigh_joint`，`12=RR_calf_joint`
- 固件侧关节名已和 `/home/lain/OpenDoge/OpenDoge_description/URDF`、`/home/lain/OpenDoge/OpenDoge_train` 对齐，统一使用 `hip/thigh/calf`。

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

- `ros2_control.yaml` 已改成 `position/velocity/effort/kp/kd` 多 command interface。
- EL05 运控模式需要 `q/dq/tau/kp/kd`。
- `opendoge_rl_node` 当前发布 `sensor_msgs/msg/JointState`：
  - `position` = 目标位置
  - `velocity` = 0
  - `effort` = `kp`
- `kd` 和 torque 命令没有通过 `/joint_target` 传下去。

建议下一步：

- 保持 `/joint_target` 作为 200 Hz 策略级 position target。
- 由 `robot_joint_controller` 或后续自定义 controller 将 `/joint_target` 转为 `position/velocity/effort/kp/kd`。
- `MotorHardware` 负责以 1000 Hz 将五元组打包成 EL05 运控帧。

### 标定和安全

已知信息：

- 电机 ID 为 `1..12`，顺序为左前、右前、左后、右后，每条腿内部为 `hip/thigh/calf`。
- 硬件有 USB 转 4 路 CAN2.0 满速模块，适合按“一条 CAN 总线带一条腿 3 个电机”的方式组织。
- 每路 1 kHz 带 3 个电机的带宽余量充足，CAN 总线本身不应成为当前架构的主要限制。
- CAN 通道到腿的实际映射已经确认：
  - `can0=左前`
  - `can1=右前`
  - `can2=左后`
  - `can3=右后`
- 根据 EL05 手册，运控模式量程为：
  - position: `-12.57..12.57 rad`
  - velocity: `-50..50 rad/s`
  - torque: `-6..6 Nm`
  - Kp: `0..500`
  - Kd: `0..5`

仍然缺少或需要实机确认的内容：

- 关节方向符号。
- 关节零点 offset。
- 软件限位。
- 上机安全限幅的最终数值，例如 position rate limit、torque limit、velocity limit。
- RL 节点本地 mode flag 之外的完整急停链路。

必须加入的保护策略：

- `MotorHardware` 一旦解析到电机反馈故障，立即强制进入阻尼模式。
- 触发阻尼模式的条件至少包括：
  - 通信类型 2 反馈中的故障位非 0。
  - 通信类型 21 或参数 `0x3022 faultSta` 中的故障位非 0。
  - 电机过温。
  - 三相电流故障或过流。
  - 堵转过载。
  - 过压、欠压。
  - 驱动芯片故障。
  - 编码器未标定、位置初始化故障、硬件识别故障。
  - state timeout、command timeout、CAN bus-off。
- 阻尼模式建议输出：
  - `position = 当前反馈位置`
  - `velocity = 0`
  - `torque = 0`
  - `kp = 0`
  - `kd = safe_kd`

## 滤波状态

### IMU / 陀螺仪滤波

当前 RL 节点不对 IMU 数据做软件滤波。

当前行为：

- quaternion 直接从 `/imu` 拷贝。
- angular velocity 直接从 `/imu` 拷贝。
- projected gravity 由内部保存的 quaternion 计算。
- observation 做了 clamp，但 clamp 不是滤波。

当前决定：

- 固件代码不加 IMU 软件滤波。
- 如果后续发现 IMU 性能不足，单独在 IMU 驱动或硬件侧处理。

### 电机输出滤波

当前 RL 节点不做软件低通滤波。

当前行为：

- action 做 clamp。
- action 做 scale。
- target position clamp 到关节限位。
- 50 Hz 策略输出被保持，并以 200 Hz 发布。
- 不做低通平滑。

建议：

- 增加目标位置变化率限制。
- 在 controller/hardware 层增加 velocity、torque、Kp、Kd 限幅。
- 1000 Hz 电机控制循环消费最新 200 Hz position target。
- 不加滤波；如需更平滑，优先做限速/限加速度，而不是低通。

## CPU ONNX 推理计划

CPU ONNX 推理可行，前提是实测推理时间稳定低于 50 Hz 策略周期的 20 ms。

建议实现步骤：

1. 给 `opendoge_rl_node` 增加 ONNX Runtime 依赖。
2. 增加 `policy_backend=onnx`。
3. 节点初始化时只加载一次 ONNX 模型。
4. observation 构造必须和 OpenDoge_train 完全一致。
5. 只在 `inference_rate_hz` 节奏下运行推理。
6. 沿用 action clamp 和 scale。
7. 发布 debug action 和 observation，方便与训练侧 replay 对齐。
8. 实测推理延迟和端到端控制延迟。

建议先用 CPU ONNX 跑通并完成数值校验，再考虑 RKNN/NPU 优化。

## 电机控制路径

预期生产路径：

```text
RL node -> target command -> ros2_control controller -> MotorHardware
  -> SocketCAN(can0/can1/can2/can3) -> USB 转 4 路 CAN2.0 模块 -> EL05 CAN bus
```

EL05 工具已经确认底层方向：

- 使用 SocketCAN raw socket。
- 使用 4 路 CAN2.0：`can0`、`can1`、`can2`、`can3`，参考 `mi_motor_demo_TB.py`。
- 使用 29-bit extended CAN ID。
- 使用 RobStride 私有协议。
- 运控命令携带 `q/dq/tau/kp/kd`。
- 每路 CAN 带一条腿的 3 个电机，关节顺序为 `hip/thigh/calf`。
- 已确认电机映射：
  - `can0`: 左前 `1/2/3`
  - `can1`: 右前 `4/5/6`
  - `can2`: 左后 `7/8/9`
  - `can3`: 右后 `10/11/12`

当前仓库状态：

- Python EL05 工具可以用于单电机 bringup 和协议参考。
- ROS 生产电机路径必须补齐 C++ hardware/controller 后才完整。

## ros2_control 和延迟

当前设计意图是使用 `ros2_control`：

- `bringup.launch.py` 启动 `controller_manager/ros2_control_node`。
- launch 会 spawn `robot_joint_controller`。
- `ros2_control.yaml` 设置 `update_rate: 1000`。

延迟判断：

- `ros2_control` 会引入调度和 controller 周期延迟。
- 1000 Hz 下一个控制周期是 1 ms。
- 正确实现时，这个量级通常可以接受。
- 在 4 路 CAN2.0、每路 3 个电机、每路 1 kHz 余量充足的硬件条件下，CAN 总线带宽不是当前最主要风险。
- 更大的风险通常是 Linux 调度抖动、ROS topic 排队、USB2CAN 驱动实现、SocketCAN 收发调度、协议解析和安全闭环。

建议：

- 1000 Hz 高频电机 read/write 放在 controller/hardware loop 内部。
- 不要通过普通 ROS topic 发送 1000 Hz 逐电机命令。
- ROS topic 只承载 50/200 Hz 策略级 target。
- 在改架构前先用时间戳实测 jitter。
- 硬件接口应原生支持 4 路 CAN 并行收发，不要把 12 个电机挤到单路 CAN。
- 即使带宽充足，也要记录每路 CAN 的发送周期、反馈周期、丢帧、超时和 bus error。

推荐线程/循环设计：

- `opendoge_rl_node`: 50 Hz ONNX 推理，200 Hz 发布 position target。
- `robot_joint_controller`: 200 Hz 接收 target，1000 Hz 输出 `q/dq/tau/kp/kd` 到 command interfaces。
- `MotorHardware`: 1000 Hz 主循环，4 路 CAN 并行发送 12 个电机运控帧，并非阻塞接收反馈。
- 每路 CAN 独立统计 send/recv timestamp、timeout、fault、bus error。
- ONNX Runtime 限制线程数，避免和 1000 Hz 控制线程抢占；控制线程优先级高于推理线程。

## 建议推进顺序

1. 在 `opendoge_rl_node` 实现 CPU ONNX 后端。
2. 增加训练导出 replay 测试，校验 observation/action 数值一致。
3. 明确定义 `q/dq/tau/kp/kd` 命令语义。
4. 将 EL05 协议代码迁移到 C++。
5. 实现 `motor_control_interface/MotorHardware`，按 4 路 CAN、每路 3 电机组织配置。
6. 实现或替换 `robot_joint_controller`。
7. 增加 hardware/controller watchdog、安全限幅和故障进入阻尼模式。
8. 先验证单电机 SocketCAN。
9. 再验证单腿。
10. 再以低增益、低 action scale 验证 12 电机。
11. 最后再启用完整 RL walking policy。

## 当前结论

CPU ONNX 是合适的下一步。当前仓库还没有完整实时电机控制链路。最关键的工程工作是：
生产级 EL05 C++ 硬件接口、明确的 `q/dq/tau/kp/kd` 命令链路、安全/watchdog 层，以及训练侧
observation 与部署侧 ONNX 推理之间的数值一致性验证。
