# USB2CAN 信号转发板

本目录用于存放 OpenDoge USB2CAN 硬件 bringup、链路验证与参考脚本。

## 硬件架构

```
Orange Pi 5 → USB 2.0 Hub (外部供电) → 4× CANable/candleLight (gs_usb) → can0/can1/can2/can3
                                         → 3× CH340 USB 串口 (备用/调试)
```

生产链路：

```text
opendoge_deploy -> SocketCAN(can0/can1/can2/can3) -> USB2CAN 转发板 -> EL05 CAN 总线
```

### USB-CAN 适配器

| 属性 | 值 |
|------|-----|
| 型号 | CANable / candleLight (gs_usb) |
| USB VID/PID | `1d50:606f` |
| 内核驱动 | `gs_usb` |
| CAN 通道数 | 4 (can0-can3) |
| 供电要求 | **USB Hub 必须外部供电** (Orange Pi 5 USB 口供电不足) |

启动 CAN 接口：

```bash
sudo modprobe gs_usb can can_raw
for i in 0 1 2 3; do
    sudo ip link set down can$i
    sudo ip link set can$i type can bitrate 1000000 loopback off
    sudo ip link set up can$i
done
```

## 已放置文件

| 文件 | 用途 | 状态 |
|------|------|------|
| `mi_motor_demo_TB.py` | **小米电机**供应商参考脚本, 仅作 `python-can` + SocketCAN 链路参考 | 📦 保留不动 |
| `../hardware/motor/scan_motors_readonly.py` | EL05 只读电机扫描 (不使能, 不写参数) | ✅ 生产使用 |
| `../hardware/motor/el05_motor_menu.py` | EL05 交互式电机调测菜单 | ✅ 生产使用 |
| `../hardware/motor/protocol_selftest.py` | EL05 协议自检 | ✅ 生产使用 |

> ⚠️ **重要**: `mi_motor_demo_TB.py` 是**小米电机**协议，与 OpenDoge 使用的 **EL05 (RobStride/灵足)** 电机不同。
> 两个协议 CAN 帧结构相似但参数范围不同 (EL05: P_MIN=-12.57, 小米: P_MIN=-12.5)。
> 该文件**保留不动**，仅作历史参考。

## EL05 电机协议

详见 `docs/EL05使用说明书2600428.pdf`。关键参数：

```text
torque:   -6..6 Nm
position: -12.57..12.57 rad
velocity: -50..50 rad/s
kp:       0..500
kd:       0..5
```

### 出厂默认值

**新电机 CAN ID 为 127 (0x7F)**，不是 OpenDoge 标准 ID 1-12。
需要用 Windows 上位机 **EDULITE-TOOL** (灵足官网下载) 逐个改 ID。

### 只读扫描 (安全操作)

```bash
python3 hardware/motor/scan_motors_readonly.py
```

此脚本：
- 使用 `COMM_GET_DEVICE_ID` (0x00) 广播发现电机 (包括出厂 ID 127)
- 使用 `COMM_READ_PARAM` (0x11) 读取位置/速度/模式/故障
- **不发送任何控制命令** (不使能、不置零、不写参数、不控制运动)

### 交互式调测 (谨慎操作)

```bash
./hardware/motor/el05_motor_menu.py --channel can3 --master-id 0xfd
```

此工具可执行使能、停止、置零等**写操作**，使用前确保电机处于安全状态。

## OpenDoge CAN 通道映射

```text
can0 = 左前 FL: motor 1/2/3  (hip/thigh/calf)
can1 = 右前 FR: motor 4/5/6
can2 = 左后 RL: motor 7/8/9
can3 = 右后 RR: motor 10/11/12
```

## 安装依赖

```bash
python3 -m pip install python-can
```

## 观察总线

```bash
candump can0          # 实时 CAN 流量
ip -details link show can0  # CAN 接口状态 (ERROR-ACTIVE = 正常)
```
