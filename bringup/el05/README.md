# EL05 Interactive Motor Tools

`el05_motor_menu.py` is an interactive hardware menu for OpenDoge EL05 (RobStride/灵足) motors.

It is intentionally EL05 / RobStride only. The PD02 tool package was used only as a reference for menu shape and operator workflow; OpenDoge does not use LK/Lingkong motors.

> ⚠️ `bringup/usb2can/mi_motor_demo_TB.py` 是**小米电机**的供应商参考脚本，与 EL05 协议不同，保留不动仅作链路参考。

## 电机出厂默认值

新电机 **CAN ID 默认为 127 (0x7F)**，不是 OpenDoge 标准 ID 1-12。
需要使用 Windows 上位机 **EDULITE-TOOL** (灵足官网 robstride.com 下载) 逐个改为标准 ID。

先用只读工具发现所有电机 (含出厂 ID):
```bash
python3 bringup/scan_motors_readonly.py --all-ids
```

## Prerequisites

Bring up the production USB2CAN signal forwarding board as a Linux SocketCAN interface:

```bash
sudo modprobe gs_usb can can_raw
sudo ./scripts/setup_can.sh can0 1000000
```

From the workspace root:

```bash
./bringup/el05/el05_motor_menu.py --channel can0 --master-id 0xfd
```

Optional custom motor ID order (factory default ID included):

```bash
./bringup/el05/el05_motor_menu.py --channel can3 --ids 10,11,12,127
```

## Menu

- `list motors`: show current joint-name to motor-id order.
- `read mech position/velocity params`: read `0x7019 mechPos` and `0x701B mechVel` without enabling the motor.
- `listen feedback frames`: print communication type 2 feedback frames already present on the bus.
- `read faultSta`: read `0x3022 faultSta` as uint32 and print set bits.
- `enable selected`: send communication type 3.
- `stop selected`: send communication type 4.
- `clear fault selected`: send communication type 4 with clear-fault flag.
- `set motion mode selected`: write `0x7005 run_mode = 0`, after stop.
- `small jog one motor`: enable one motor and send small EL05 motion-control frames.
- `set mechanical zero selected`: stop and send communication type 6.

## Safety Notes

- Start with a single suspended motor before testing a loaded leg.
- Do not use `small jog` on multiple motors; the tool intentionally restricts it to one motor.
- Mechanical zero changes the motor reference. Use it only after confirming the joint is physically in the intended zero pose.
- This tool is for bringup and validation. The production deployment path is the non-ROS C++ `opendoge_deploy` runtime.
