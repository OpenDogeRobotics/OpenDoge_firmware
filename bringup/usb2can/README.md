# USB2CAN 信号转发板示例

本目录用于存放正式机器人 USB2CAN 信号转发板的硬件 bringup、链路验证与参考脚本。

这块板是 OpenDoge 真机从主控到 EL05 电机 CAN 总线的正式信号转发板。生产链路是：

```text
opendoge_deploy -> SocketCAN(can0/can1/can2/can3) -> USB2CAN 信号转发板 -> EL05 CAN 总线
```

## 已放置文件

- `mi_motor_demo_TB.py`：供应商/上位机示例脚本，使用 `python-can` 通过 Linux SocketCAN 发送 29-bit 扩展 CAN 帧。

原始文件仍保留在仓库根目录 `mi_motor_demo_TB.py`，避免误删用户提供的样例。

## 板卡接口

- Python 包：`python-can`
- CAN backend：`socketcan`
- Linux 网络设备：`can0`、`can1`、`can2`、`can3`
- CAN 配置：`bitrate 1000000`
- 帧格式：29-bit extended CAN frame

## OpenDoge 映射

```text
can0 = 左前 FL: motor 1/2/3
can1 = 右前 FR: motor 4/5/6
can2 = 左后 RL: motor 7/8/9
can3 = 右后 RR: motor 10/11/12
```

每条腿内部顺序为 `hip/thigh/calf`。

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

`mi_motor_demo_TB.py` 是协议和链路参考，不是 OpenDoge 的生产部署程序：

- 脚本中的默认电机 ID 不一定匹配 OpenDoge 12 电机映射。
- 脚本主循环可能持续运动电机，上机前必须改成单电机、小幅、可退出测试。
- 生产部署使用 `src/opendoge_deploy` 的 C++ SocketCAN 实现。
- 部署程序不会自动执行机械置零；置零必须人工确认关节处于目标零位后再做。

EL05 手册运控模式范围：

```text
torque:   -6..6 Nm
position: -12.57..12.57 rad
velocity: -50..50 rad/s
kp:       0..500
kd:       0..5
```
