# USB2CAN 信号转发板示例

本目录用于存放正式机器人 USB2CAN 信号转发板的硬件 bringup、链路验证与单电机测试脚本。

这块板不是临时调试板，而是 OpenDoge 真机从主控到 EL05 电机 CAN 总线的正式信号转发板。生产驱动应围绕这条链路实现：

```text
ROS2 / ros2_control -> 电机硬件接口 -> SocketCAN(can0/can1/...) -> USB2CAN 信号转发板 -> EL05 CAN 总线
```

## 已放置文件

- `mi_motor_demo_TB.py`：供应商/上位机示例脚本，使用 `python-can` 通过 Linux SocketCAN 发送 29-bit 扩展 CAN 帧。

原始文件仍保留在仓库根目录 `mi_motor_demo_TB.py`，避免误删用户提供的样例。

## 板卡接口检查

这个示例对应的底层接口是：

- Python 包：`python-can`
- CAN backend：`socketcan`
- Linux 网络设备：`can0`、`can1`、`can2`、`can3`
- CAN 配置：`bitrate 1000000`
- 帧格式：29-bit extended CAN frame

当前系统检查结果：

- 当前 Python 环境未安装 `python-can`，`import can` 会失败。
- 当前 `OpenDoge_firmware/src` 没有面向这块正式 USB2CAN 信号转发板的 Python 或 C++ SocketCAN 驱动包。
- 当前 `OpenDoge_firmware/src` 也没有可见的 `motor_control_interface/MotorHardware` 源码。
- 现有配置只在 `ros2_control.yaml` 中声明 `can_interface: can0`，但实际 CAN 收发实现不在当前仓库可见源码内。

## 使用前准备

安装依赖：

```bash
python3 -m pip install python-can
```

打开 CAN 口：

```bash
sudo ip link set down can0
sudo ip link set can0 type can bitrate 1000000 loopback off
sudo ip link set up can0
```

观察总线：

```bash
candump can0
```

## 重要注意

这个脚本只能作为正式 USB2CAN 信号转发板的链路验证和私有扩展帧格式参考，不能直接作为 OpenDoge 真机驱动：

- 脚本默认控制电机 ID `6` 和 `7`，不匹配 OpenDoge 12 电机映射。
- 脚本主循环会持续来回运动电机，上机前必须先改成单电机、小幅、可退出的测试工具。
- 脚本里的范围更像小米/其他示例参数：
  - torque：`[-12.5, 12.5]`
  - position：`[-12.0, 12.0]`
  - velocity：`[-30.0, 30.0]`
- EL05 手册私有运控模式建议按 EL05 范围重新确认：
  - torque：`[-6, 6] Nm`
  - position：`[-12.57, 12.57] rad`
  - velocity：`[-50, 50] rad/s`
  - kp：`[0, 500]`
  - kd：`[0, 5]`
- `set_motion_mode()` 当前发送通信类型 18 但未写入 `0x7005 run_mode` 参数，按 EL05 手册需要补齐参数写入帧，不能假定它已经正确切换模式。

## 后续接入建议

1. 保留本脚本作为原始参考。
2. 新增一个面向正式 USB2CAN 信号转发板的 OpenDoge 专用 `el05_single_motor_tool.py`，提供明确 CLI：
   - `--channel can0`
   - `--motor-id N`
   - `enable`
   - `stop`
   - `zero`
   - `jog`
   - `read-state`
3. 再把经过验证的打包/解析逻辑迁移到 C++ `motor_control_interface/MotorHardware` 或新的 EL05 ROS2 硬件包中。
4. 正式驱动不应绕过这块板另起通信路径；应通过 Linux SocketCAN 设备名统一配置，例如 `can0`、`can1`。
