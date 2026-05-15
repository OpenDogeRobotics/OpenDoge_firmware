# opendoge-apx ROS2 运控工作区使用手册

适用：Windows/PowerShell，ROS2 Humble，四足机器人 **opendoge-apx**。已集成 `motor_control` 硬件接口与 `dm_imu` 驱动，提供描述、控制与 bringup。

## 目录结构
```
opendoge_ws/
  src/
    motor_control/        # 现有电机驱动（目录联接）
    dm_imu/               # 现有 IMU 驱动（目录联接）
    opendoge_description/ # URDF/Xacro 与 meshes
    opendoge_control/     # ros2_control 配置
    opendoge_bringup/     # 启动与 controller 配置
  install/ build/ log/    # colcon 生成
```

## 先决条件
- 已安装 ROS2 Humble，含 `ros2_control`、`controller_manager`、`robot_state_publisher`、`xacro`。
- CAN 接口连通，`motor_control` 插件能找到 `can_interface`，并设置正确的 `master_id`、motor_id。
- IMU 驱动可发布 `/imu`（`header.frame_id` 需与 URDF 的 `imu_link` 对齐）。

## 首次准备
1. 在 `ROS_WS` 下已创建目录联接（已完成）：
   ```powershell
   cd C:\Users\com01\Desktop\rl_sar\ROS_WS\opendoge_ws\src
   cmd /c mklink /J motor_control ..\motor_control
   cmd /c mklink /J dm_imu ..\DM-IMU-ROS2\ros2_ws\src\dm_imu
   ```
2. 如需调整：
   - URDF：`src/opendoge_description/urdf/opendoge_apx.urdf.xacro`
   - ros2_control 插件与关节列表：`src/opendoge_control/config/ros2_control.yaml`
   - 控制器关节顺序：`src/opendoge_bringup/config/controllers.yaml`

## 构建
```powershell
cd C:\Users\com01\Desktop\rl_sar\ROS_WS\opendoge_ws
colcon build --symlink-install
# 打开新终端后:
call install\setup.bat
```

## 运行 bringup
```powershell
ros2 launch opendoge_bringup bringup.launch.py
```
默认加载：
- `robot_state_publisher`：从 xacro 生成 `robot_description`
- `ros2_control_node`：加载 `ros2_control.yaml` 与 `controllers.yaml`
- `spawner robot_joint_controller`

## 快速验收（对应 DOCS/08 Step1-4）
```powershell
ros2 param get /robot_state_publisher robot_description
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic hz /robot_joint_controller/state
```
Passive/低增益测试示例（需先读 state，数组长度=12）：
```powershell
ros2 topic echo /robot_joint_controller/state --once
ros2 topic pub --once /robot_joint_controller/command robot_msgs/msg/RobotCommand "
motor_command:
  - {q: 0.0, dq: 0.0, tau: 0.0, kp: 0.0, kd: 2.0}
  - {q: 0.0, dq: 0.0, tau: 0.0, kp: 0.0, kd: 2.0}
  # ... 共12项，按关节顺序补齐
"
```

## 与现有代码的融合要点
- 硬件接口：`ros2_control.yaml` 指向 `motor_control_interface/MotorHardware`，需把 `motor_id`、`can_interface`、`master_id` 替换为真实值，保持与 `motor_control` 内部 mapping 一致。
- 关节顺序：URDF、`controllers.yaml`、`ros2_control.yaml`、策略 `policy/opendoge_apx/base.yaml` 必须同序。
- IMU：确认 `/imu` 的 `frame_id` 等于 `imu_link`，若不一致，在 IMU 驱动或 RL 节点做转换。
- 策略配置：`policy/opendoge_apx/base.yaml` 与 `policy/opendoge_apx/robot_lab/config.yaml` 已放置，占位参数需结合真实模型与量纲调整。

## 常见可调项
- `opendoge_apx.urdf.xacro`：惯量、连杆长度、IMU 安装位姿。
- `ros2_control.yaml`：`update_rate`、关节 effort 限制、接口类型。
- `controllers.yaml`：如需 joint_state_broadcaster，可追加后在 launch 中增加 spawner。

## 上机前检查清单
- motor_id 与电机实际布线一致；正方向确认。
- 力矩量纲一致（N·m 或驱动器单位）；初始 `torque_limits` 设置为额定的 10%-20%。
- `/robot_joint_controller/state` 频率稳定；`/imu` 频率与 frame 正确。
- 先 Passive，再单关节微动，确认方向和振荡情况。

更多细节可参考仓库 `DOCS/08_hands_on_tutorial.md`、`DOCS/07_troubleshooting.md`。 

