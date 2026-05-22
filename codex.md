# OpenDoge Firmware 后续修改计划

日期：2026-05-22

## 1. 当前判断

- EL05 使用说明书中的默认电机协议是 RobStride 私有 CAN 协议：
  - CAN 2.0
  - 1 Mbps
  - 29-bit 扩展帧
  - 8 Byte 数据区
  - 关键通信类型：1 运控控制、2 状态反馈、3 使能、4 停止、6 设置零位、17 读参数、18 写参数、21 故障反馈、22 保存参数、24 主动上报、25 协议切换
- EL05 也支持 CANopen 和 MIT 协议，但当前 `RobotCommand` 的字段 `q/dq/tau/kp/kd` 与私有协议的运控模式最匹配。
- 当前 `/home/lain/OpenDoge/OpenDoge_firmware/src` 里没有可见的电机协议打包/解析实现。
- 当前 ROS2 配置只是指向外部硬件插件 `motor_control_interface/MotorHardware`：
  - `can_interface: can0`
  - `master_id: 253`
  - `update_rate: 500`
- `motor_control` 目录在 README 中被描述为目录联接，但当前 `src` 下实际不存在，因此不能确认当前系统真正发送的是哪种电机协议。
- `opendoge_rl_node` 目前只是占位逻辑：超时发安全指令，正常时保持当前位置，还没有接入策略推理。

结论：当前仓库可见源码不足以满足 EL05 真机电机控制需求。需要补齐或接入 RobStride/EL05 CAN 驱动，并把它挂到 ros2_control 硬件接口中。

## 2. 推荐协议路线

优先选择 EL05 私有协议的运控模式，而不是 CANopen 或 MIT：

- 与现有 `robot_msgs/msg/MotorCommand` 的 `q/dq/tau/kp/kd` 最直接匹配。
- 单帧可以同时下发目标位置、速度、前馈力矩、Kp、Kd，适合四足 500 Hz 控制。
- 状态反馈帧可直接解析位置、速度、力矩、温度和故障位。
- 后续 RL 策略输出也通常是 PD 目标或关节力矩，私有运控模式改造成本最低。

协议映射建议：

- `q` -> 目标角度，范围 `[-12.57, 12.57] rad`，编码到 uint16。
- `dq` -> 目标角速度，范围 `[-50, 50] rad/s`，编码到 uint16。
- `tau` -> 前馈力矩，EL05 范围 `[-6, 6] Nm`，放入 29-bit CAN ID 的数据区 2。
- `kp` -> `0..500`，编码到 uint16。
- `kd` -> `0..5`，编码到 uint16。
- 反馈位置、速度、力矩、温度从通信类型 2 解析。

## 3. 实施任务

### 3.1 补齐电机驱动包

新增或接入一个明确的 EL05/RobStride 驱动包，建议包名：

- `opendoge_el05_driver`，或
- 恢复 README 中提到的 `motor_control` 目录联接。

驱动需要实现：

- SocketCAN 打开和关闭。
- `can0` 配置检查，要求 1 Mbps。
- 29-bit 扩展帧发送和接收。
- 通信类型 1 运控控制帧打包。
- 通信类型 2 状态反馈帧解析。
- 通信类型 3 使能。
- 通信类型 4 停止和清故障。
- 通信类型 6 设置机械零位。
- 通信类型 17/18 参数读写，至少支持：
  - `0x7005 run_mode`
  - `0x7016 loc_ref`
  - `0x7017 limit_spd`
  - `0x7018 limit_cur`
  - `0x7019 mechPos`
  - `0x701B mechVel`
- 故障位解析和日志输出。
- 单电机通信自测工具。

可参考本机已有代码：

- `/home/lain/qxzn/combat_system_ws/src/framework/02_real_hardware/actuators/qx_robstride_motor_driver`

但接入 OpenDoge 时需要确认 EL05 参数范围，不要直接沿用其他 RobStride 型号的 `torque_max/kp_max/kd_max`。

### 3.2 实现 ros2_control 硬件接口

目标：让 `motor_control_interface/MotorHardware` 在当前仓库内可构建、可追踪。

需要实现或确认：

- `on_init()` 读取：
  - `can_interface`
  - `master_id`
  - 每个 joint 的 `motor_id`
  - 每个 joint 的方向系数、零点偏置、力矩限制
- `on_activate()`：
  - 打开 SocketCAN
  - 停止电机
  - 设置/确认运控模式
  - 使能电机
  - 读取初始状态
- `read()`：
  - 接收并解析通信类型 2 状态反馈
  - 更新 `position/velocity/effort`
  - 记录温度、故障位和在线状态
- `write()`：
  - 将 `RobotCommand` 或 ros2_control command interface 转成 EL05 运控帧
  - 对 `q/dq/tau/kp/kd` 做范围限制
  - 按 12 个电机顺序发送
- `on_deactivate()`：
  - 发送停止帧
  - 关闭 CAN

当前配置只声明了 `effort` command interface，但 RL 节点发布的是 `q/dq/tau/kp/kd`。这里需要二选一：

- 方案 A：保留现有 `robot_joint_controller/RobotJointControllerGroup`，由它把 `RobotCommand` 传给硬件层，硬件层发送完整运控帧。
- 方案 B：改成标准 ros2_control 多 command interface：`position/velocity/effort/kp/kd`，减少自定义控制器依赖。

短期建议走方案 A，改动小；中期再考虑标准化。

### 3.3 修正机器人配置

需要更新：

- `src/opendoge_description/urdf/opendoge_apx.urdf.xacro`
  - 替换 placeholder motor_id。
  - 增加每个关节的方向系数。
  - 增加零点 offset。
- `src/opendoge_control/config/ros2_control.yaml`
  - 明确电机 ID 映射。
  - 明确每个关节限幅。
  - 增加 CAN 协议参数，例如 `protocol: robstride_private`。
- `src/opendoge_bringup/config/controllers.yaml`
  - 保证关节顺序和 URDF、策略配置完全一致。

### 3.4 上机前工具

先做独立工具，不直接跑整机：

- `el05_scan`：扫描/确认 1..12 电机 ID。
- `el05_enable_one --id N`：单电机使能。
- `el05_stop_one --id N`：单电机停止。
- `el05_read_state --id N`：读取单电机状态。
- `el05_set_zero --id N`：设置零点，默认要求二次确认。
- `el05_jog --id N --amp 0.05 --kp 5 --kd 0.2`：小幅往返测试方向。

这些工具用于先确认 CAN 连接、ID、方向、零点和反馈量纲。

### 3.5 安全策略

必须加入：

- 命令超时保护：超过指定周期没有收到新命令，发送停止或低刚度保持。
- 状态超时保护：某个电机状态超时，整机进入 passive。
- 上电默认不自动大力矩站立。
- 初始测试力矩限制为额定/峰值的低比例。
- `kp/kd/tau/q/dq` 全部限幅。
- 电机故障位非 0 时停止对应电机，并上报 ROS 日志。
- 禁止运行中切换控制模式。手册明确说明需要先发送停止命令再切换模式。

## 4. 验收顺序

1. 编译通过：
   - `colcon build --symlink-install`
2. CAN 接口确认：
   - `ip link show can0`
   - `candump can0` 能看到 EL05 反馈。
3. 单电机通信：
   - 能使能、停止、读状态。
   - 位置/速度/力矩/温度解析合理。
4. 单电机小幅控制：
   - `q` 正方向正确。
   - `kp/kd/tau` 作用符合预期。
   - 停止命令可靠。
5. 12 电机 passive：
   - 所有电机在线。
   - `/robot_joint_controller/state` 频率稳定。
6. 低刚度站立或悬空测试：
   - 先悬空，再落地。
   - 检查抖动、反向、限位和温升。
7. 接入 RL：
   - 在 `rl_node.cpp` 中替换保持当前位置占位逻辑。
   - 先限幅输出，再逐步放开策略。

## 5. 当前风险点

- `motor_control_interface/MotorHardware` 源码不在当前 `src`，构建时可能找不到插件。
- 当前 motor_id 是 placeholder，不能直接上机。
- 当前 command interface 只有 `effort`，但实际 EL05 运控需要 `q/dq/tau/kp/kd`。
- EL05 私有协议、MIT 协议、CANopen 协议不能混用；电机当前协议状态必须先确认。
- 手册中的不同协议帧格式不同：
  - 私有协议：29-bit 扩展帧。
  - MIT：11-bit 标准帧。
  - CANopen：标准 CANopen 对象字典。
- 如果电机已被切到 MIT 或 CANopen，需要先切回私有协议并重新上电，或驱动必须按当前协议实现。

## 6. 下一步建议

优先做最小闭环：

1. 把 `qx_robstride_motor_driver` 中的私有协议打包/解析逻辑迁移成 OpenDoge 可构建的驱动库。
2. 新增单电机 CLI 测试工具。
3. 实现 `motor_control_interface/MotorHardware` 或替换当前插件名。
4. 用单电机确认 EL05 帧格式、方向和单位。
5. 再接入 12 电机 ros2_control。

完成最小闭环后，再处理 RL 策略推理和整机步态。

## 7. USB2CAN 正式信号转发板记录

用户确认：这块 USB2CAN 板不仅用于调试，也是正式机器人从主控到 EL05 电机 CAN 总线的信号转发板。

已将用户提供的 USB2CAN 示例复制到：

- `tools/usb2can/mi_motor_demo_TB.py`

并补充说明：

- `tools/usb2can/README.md`

检查结果：

- 示例使用 `python-can` + `socketcan`，对应 Linux 下的 `can0/can1/...` 网络设备；这应视为正式通信链路接口，而不是临时调试接口。
- 示例发送 29-bit 扩展帧，方向上可作为 EL05 私有协议参考。
- 当前 Python 环境未安装 `python-can`，`import can` 会失败。
- 当前 `src` 下仍没有可见的正式 USB2CAN/SocketCAN 驱动接口包，也没有可见的 `motor_control_interface/MotorHardware` 源码。
- 示例参数范围和 EL05 手册不完全一致，不能直接用于真机闭环控制。
- 示例的 `set_motion_mode()` 没有按 EL05 手册写入 `0x7005 run_mode`，需要在 OpenDoge 专用工具中重写。

正式链路目标：

```text
ROS2 / ros2_control -> 电机硬件接口 -> SocketCAN(can0/can1/...) -> USB2CAN 信号转发板 -> EL05 CAN 总线
```

因此后续实现 `motor_control_interface/MotorHardware` 时，应把 `can_interface` 明确解释为这块 USB2CAN 信号转发板在 Linux 下暴露的 SocketCAN 网络设备名。

## 8. 当前控制框架缺口清单

按当前 `/home/lain/OpenDoge/OpenDoge_firmware` 可见内容判断，这个工作区还不是一个完整的真机控制框架。它现在更接近 ROS2 bringup 骨架：有 URDF、ros2_control 配置、controller 配置和一个 RL 占位节点，但缺少从策略命令到 EL05 电机 CAN 帧的闭环实现。

### 8.1 构建依赖缺口

当前 `src` 下缺少以下关键包源码：

- `motor_control_interface`：`ros2_control.yaml` 和 URDF 都引用了 `motor_control_interface/MotorHardware`，但当前仓库没有这个硬件插件实现。
- `robot_joint_controller`：`controllers.yaml` 配置了 `robot_joint_controller/RobotJointControllerGroup`，但当前仓库没有控制器源码。
- `robot_msgs`：`opendoge_rl_node` 依赖 `robot_msgs/msg/RobotCommand`、`RobotState`、`MotorCommand`、`MotorState`，但当前仓库没有消息定义。
- `dm_imu` 或等价 IMU 驱动：README 里写了需要 `/imu`，但当前 `src` 下没有 IMU 驱动包。

影响：

- 直接 `colcon build` 很可能因找不到这些包失败。
- 即使系统环境里装过这些包，当前仓库也无法自洽复现和审查真机控制链路。

### 8.2 电机硬件接口缺口

当前 `opendoge_control` 是配置包，不是硬件驱动包。缺少：

- `hardware_interface::SystemInterface` 实现。
- SocketCAN 打开、关闭和错误恢复。
- USB2CAN 信号转发板的 `can0/can1/...` 通道管理。
- EL05 私有协议帧打包和解析。
- 电机使能、停止、清错、零点设置、参数读写。
- 12 电机批量发送和反馈接收调度。
- 通信超时、丢帧统计、总线错误统计。

必须补齐后，`ros2_control_node` 才能真正控制电机。

### 8.3 协议层缺口

已确认 USB2CAN 示例发送 29-bit 扩展帧，但当前正式控制框架还缺少完整 EL05 协议层：

- 通信类型 1：运控模式 `q/dq/tau/kp/kd` 控制帧。
- 通信类型 2：电机状态反馈帧。
- 通信类型 3：电机使能。
- 通信类型 4：停止运行和清故障。
- 通信类型 6：机械零位。
- 通信类型 17/18：参数读写。
- 通信类型 21：故障反馈。
- 通信类型 22：保存参数。
- 通信类型 24：主动上报配置。
- 通信类型 25：协议切换。

还需要明确电机当前到底处于私有协议、MIT 协议还是 CANopen 协议。驱动不能假设协议状态正确。

### 8.4 控制接口不匹配

当前配置存在接口层不一致：

- `ros2_control.yaml` 每个关节只声明 `command_interfaces: [effort]`。
- URDF 里每个关节也只声明 `<command_interface name="effort"/>`。
- 但 RL 节点发布的 `RobotCommand` 包含 `q/dq/tau/kp/kd`。
- EL05 运控模式也需要完整 `q/dq/tau/kp/kd`，不是单一 `effort`。

需要决定接口方案：

- 短期：保留 `robot_joint_controller/RobotJointControllerGroup`，让它通过自定义 `RobotCommand` 向硬件层传完整命令。
- 中期：改成 ros2_control 多 command interface，例如 `position`、`velocity`、`effort`、`kp`、`kd`。

在方案确定前，控制链路的命令语义是不完整的。

### 8.5 机器人参数和标定缺口

当前 URDF 中电机 ID 明确是 placeholder，缺少实机必需参数：

- 每个关节对应的真实 `motor_id`。
- 每个关节所在 CAN 通道，例如 `can0` 或 `can1`。
- 每个关节方向系数。
- 每个关节零点偏置。
- 机械限位和软件限位。
- 电机力矩限制、速度限制、温度限制。
- 齿轮/连杆传动比例，如果关节输出和电机反馈不是同一坐标。
- 初始站立姿态和安全 passive 姿态。

没有这些参数不能安全上机。

### 8.6 USB2CAN 正式链路缺口

USB2CAN 板已确认是正式信号转发板，但框架里还缺少生产级链路管理：

- CAN 口启动脚本或 launch 前置动作。
- 多通道分配配置。
- `bitrate 1000000` 固化配置。
- USB2CAN 插拔后的设备名稳定策略。
- `can0/can1` 对应哪条腿或哪组电机的配置。
- 总线 off/error-passive 状态检测和恢复策略。
- `python-can` 只用于工具层；正式硬件接口建议用 C++ SocketCAN。

### 8.7 控制器和状态发布缺口

当前只有 controller 配置，没有控制器实现。需要确认或补齐：

- `/robot_joint_controller/command` 订阅逻辑。
- `/robot_joint_controller/state` 发布逻辑。
- 命令数组长度检查。
- 关节顺序映射。
- 命令限幅。
- 命令超时后进入安全输出。
- 状态中 position/velocity/effort 的单位定义。
- 是否需要 `joint_state_broadcaster` 发布标准 `/joint_states`。

如果继续使用自定义 `robot_joint_controller`，它必须和硬件层共享同一套关节顺序与电机映射。

### 8.8 安全和故障处理缺口

当前 RL 节点有简单超时回退，但完整安全链路还缺少：

- 硬件层命令 watchdog。
- 电机状态 watchdog。
- 总线通信 watchdog。
- E-stop 或软件急停入口。
- 上电默认 passive。
- 运行中禁止切模式。
- 故障位解析后的停机策略。
- 过温、过压、欠压、堵转、未标定等保护。
- 关节软限位。
- 力矩、速度、位置、Kp、Kd 全链路限幅。
- 控制频率异常检测。

这些逻辑应放在硬件层和控制器层，而不能只依赖 RL 节点。

### 8.9 RL 和运动控制缺口

`opendoge_rl_node` 当前只是占位：

- 没有加载 TorchScript/ONNX/其他策略模型。
- 没有 observation 构造。
- 没有动作缩放和限幅。
- 没有站起、趴下、passive、RL、fault 等 FSM。
- 没有 IMU 坐标系处理。
- 没有关节默认姿态、动作 offset、动作比例。
- 没有策略频率和硬件频率之间的插值或保持策略。

因此目前不能认为 RL 控制已经接入真机。

### 8.10 测试和验收工具缺口

当前缺少分层测试工具：

- `el05_single_motor_tool.py` 或 C++ 等价工具。
- 单电机 enable/stop/read-state/zero/jog。
- 12 电机扫描和 ID 校验。
- vcan/mock CAN 协议单元测试。
- ros2_control fake hardware 测试。
- 控制器命令限幅测试。
- 上机前 checklist 脚本。
- 日志记录和故障复盘工具。

建议优先补单电机工具，再补硬件插件。

### 8.11 配置和部署缺口

当前还缺少正式部署所需内容：

- `requirements.txt` 或 rosdep 说明，至少包括 `python-can` 工具依赖。
- CAN 口 bringup 脚本，例如 `scripts/setup_can.sh`。
- USB2CAN 板 udev 规则或设备命名说明。
- launch 顺序：CAN up -> IMU -> ros2_control -> controller spawner -> RL node。
- 参数文件：电机映射、方向、零点、限幅、CAN 通道。
- 日志目录和运行配置。
- 真机/仿真配置分离。

### 8.12 优先补齐顺序

建议按下面顺序推进：

1. 补齐 `robot_msgs`、`robot_joint_controller`、`motor_control_interface` 的源码或替代实现，先让工作区可构建。
2. 新增正式 USB2CAN 链路的单电机工具，验证 `can0` 到 EL05 的使能、停止、读状态和小幅 jog。
3. 实现 EL05 私有协议 C++ 驱动库。
4. 实现 `motor_control_interface/MotorHardware`，打通 `read()` 和 `write()`。
5. 明确 12 电机 ID、CAN 通道、方向、零点、限幅配置。
6. 接入 controller，验证 `/robot_joint_controller/command` 到电机动作。
7. 加入硬件层 watchdog、故障处理和软限位。
8. 最后接入 RL 策略和 FSM。

当前最关键的第一缺口是：缺少正式电机硬件接口和它依赖的 EL05/USB2CAN 协议实现。

## 9. 工具和工作区整理记录

用户确认：EL05 是灵足 RobStride/RS 电机，OpenDoge 本项目没有使用 LK/领控电机。

因此后续所有硬件工具和正式驱动只保留 EL05/RobStride 路线。`pd02_motor_driver_test` 只作为交互式菜单和上机流程参考，不引入它的 LK/Lingkong 分支、依赖或电机模型。

已新增：

- `.gitignore`
  - 忽略 `build/`、`install/`、`log/`、`__pycache__/` 等工作区生成物。
- `requirements.txt`
  - 记录供应商 Python 示例所需的 `python-can`。
  - OpenDoge 新增的 EL05 菜单工具使用 Python 内置 SocketCAN，不强依赖 `python-can`。
- `scripts/setup_can.sh`
  - 启动正式 USB2CAN 信号转发板暴露的 SocketCAN 设备。
  - 默认用法：`sudo ./scripts/setup_can.sh can0 1000000`。
- `tools/README.md`
  - 说明工具目录结构和“只做 EL05/RobStride，不做 LK”的规则。
- `tools/el05/README.md`
  - 说明 OpenDoge EL05 交互式菜单的使用方式和安全注意事项。
- `tools/el05/el05_motor_menu.py`
  - 面向 EL05/RobStride 的交互式菜单。
  - 直接使用 Linux raw SocketCAN，不依赖当前缺失的 ROS 包。
  - 当前菜单包含：
    - 列出电机 ID。
    - 读取 `0x7019 mechPos` 和 `0x701B mechVel` 参数。
    - 监听通信类型 2 反馈帧。
    - 使能电机。
    - 停止电机。
    - 清除故障。
    - 写入 `0x7005 run_mode = 0` 运控模式。
    - 单电机小幅 jog。
    - 设置机械零位。

已更新：

- `README.md`
  - 按当前真实工作区状态重新整理。
  - 明确当前缺失的 ROS 包：`motor_control_interface`、`robot_joint_controller`、`robot_msgs`、IMU 驱动。
  - 明确正式电机链路：

```text
ROS2 / ros2_control -> MotorHardware -> SocketCAN(can0/can1/...) -> USB2CAN 信号转发板 -> EL05 CAN 总线
```

当前工具层已经能支撑“正式 USB2CAN 链路 + 单电机 EL05 验证”的下一步工作；ROS 正式闭环仍需补齐 C++ `MotorHardware` 和控制器/消息包。

## 10. 构建验证记录

已执行：

```bash
colcon list
```

当前可枚举 ROS 包：

- `opendoge_bringup`
- `opendoge_control`
- `opendoge_description`
- `opendoge_rl_node`

已修正：

- `src/opendoge_description/CMakeLists.txt`
  - 移除不必要的 `find_package(xacro REQUIRED)` 构建期依赖。
  - 移除不存在的 `meshes` 目录安装项。

已验证通过：

```bash
colcon build --symlink-install --packages-select opendoge_description opendoge_control opendoge_bringup
```

结果：

- `opendoge_description` 通过。
- `opendoge_control` 通过。
- `opendoge_bringup` 通过。

仍然失败：

```bash
colcon build --symlink-install --packages-select opendoge_rl_node
```

失败原因：

- 缺少 `robot_msgs` 包，CMake 找不到 `robot_msgsConfig.cmake`。

因此当前工作区已经整理到“配置/描述/bringup 可构建，RL 节点等待消息包补齐”的状态。下一步若要全量构建，必须先补 `robot_msgs`，或者临时禁用/替换 `opendoge_rl_node` 的自定义消息依赖。
