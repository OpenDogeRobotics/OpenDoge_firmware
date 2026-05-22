# OpenDoge Firmware ROS2 工作区

适用：Linux / ROS2 Humble，四足机器人 **OpenDoge / opendoge-apx**。

当前仓库是 ROS2 控制工作区骨架，已包含机器人描述、ros2_control 配置、bringup 配置、RL 占位节点和 EL05/USB2CAN 硬件工具。正式电机硬件接口、控制器和消息包仍需补齐，详见 [codex.md](codex.md)。

## 目录结构

```
OpenDoge_firmware/
  README.md
  codex.md                         # 后续计划、缺口和协议记录
  docs/                            # EL05 手册、原理图等参考文档
  requirements.txt                  # Python 参考工具依赖
  scripts/
    setup_can.sh                    # USB2CAN SocketCAN 启动脚本
  src/
    opendoge_description/           # URDF/Xacro
    opendoge_control/               # ros2_control 配置
    opendoge_bringup/               # launch 和 controller 配置
    opendoge_rl_node/               # RL 占位节点
  tools/
    el05/                           # OpenDoge EL05/RobStride 交互式硬件工具
    usb2can/                        # USB2CAN 示例和参考说明
  build/ install/ log/              # colcon 生成，git 忽略
```

## 当前缺失的 ROS 包

当前 `src` 下还缺少这些真机闭环必需包：

- `motor_control_interface`：应提供 `motor_control_interface/MotorHardware`。
- `robot_joint_controller`：当前 `controllers.yaml` 引用了 `robot_joint_controller/RobotJointControllerGroup`。
- `robot_msgs`：`opendoge_rl_node` 依赖 `RobotCommand`、`RobotState`、`MotorCommand`、`MotorState`。
- `dm_imu` 或等价 IMU 驱动：需要发布 `/imu`。

在这些包补齐前，`colcon build` 可能无法完整通过，`ros2_control_node` 也不能真正控制 EL05。

## 硬件链路

OpenDoge 使用 EL05 灵足 RobStride/RS 电机，不使用 LK/领控电机。正式电机链路是：

```text
ROS2 / ros2_control -> MotorHardware -> SocketCAN(can0/can1/...) -> USB2CAN 信号转发板 -> EL05 CAN 总线
```

启动 CAN：

```bash
sudo ./scripts/setup_can.sh can0 1000000
```

EL05 交互式菜单：

```bash
./tools/el05/el05_motor_menu.py --channel can0 --master-id 0xfd
```

## 构建

```bash
cd /home/lain/OpenDoge/OpenDoge_firmware
colcon build --symlink-install
source install/setup.bash
```

当前已验证可先构建配置/描述/bringup 包：

```bash
colcon build --symlink-install --packages-select opendoge_description opendoge_control opendoge_bringup
source install/setup.bash
```

`opendoge_rl_node` 当前会因缺少 `robot_msgs` 构建失败。补齐 `robot_msgs` 后再执行全量构建。

## 运行 bringup

```bash
ros2 launch opendoge_bringup bringup.launch.py
```

默认加载：
- `robot_state_publisher`：从 xacro 生成 `robot_description`
- `ros2_control_node`：加载 `ros2_control.yaml` 与 `controllers.yaml`
- `spawner robot_joint_controller`

## 快速验收

```bash
ros2 param get /robot_state_publisher robot_description
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic hz /robot_joint_controller/state
```

Passive/低增益测试示例（需先读 state，数组长度=12）：

```bash
ros2 topic echo /robot_joint_controller/state --once
ros2 topic pub --once /robot_joint_controller/command robot_msgs/msg/RobotCommand "
motor_command:
  - {q: 0.0, dq: 0.0, tau: 0.0, kp: 0.0, kd: 2.0}
  - {q: 0.0, dq: 0.0, tau: 0.0, kp: 0.0, kd: 2.0}
  # ... 共12项，按关节顺序补齐
"
```

## 与现有代码的融合要点

- 硬件接口：`ros2_control.yaml` 指向 `motor_control_interface/MotorHardware`，需要在本工作区补齐或接入该插件。
- 电机协议：EL05 走 RobStride 私有 CAN 2.0 29-bit 扩展帧，优先使用运控模式 `q/dq/tau/kp/kd`。
- USB2CAN：`can_interface` 应解释为正式 USB2CAN 信号转发板暴露的 SocketCAN 设备名，例如 `can0`。
- 关节顺序：URDF、`controllers.yaml`、`ros2_control.yaml`、策略 `policy/opendoge_apx/base.yaml` 必须同序。
- IMU：确认 `/imu` 的 `frame_id` 等于 `imu_link`，若不一致，在 IMU 驱动或 RL 节点做转换。

## 常见可调项

- `opendoge_apx.urdf.xacro`：惯量、连杆长度、IMU 安装位姿。
- `ros2_control.yaml`：`update_rate`、CAN 接口、master id、关节接口类型。
- `controllers.yaml`：如需 joint_state_broadcaster，可追加后在 launch 中增加 spawner。
- `tools/el05/el05_motor_menu.py`：单电机上机前验证。

## 上机前检查清单

- motor_id 与电机实际布线一致。
- `can0/can1` 与 USB2CAN 通道和腿部布线一致。
- 关节正方向、零点 offset、软限位确认。
- 力矩量纲一致（N·m 或驱动器单位）；初始 `torque_limits` 设置为额定的 10%-20%。
- `/robot_joint_controller/state` 频率稳定。
- `/imu` 频率与 frame 正确。
- 先 Passive，再单关节微动，确认方向和振荡情况。

