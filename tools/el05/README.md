# EL05 Interactive Motor Tools

`el05_motor_menu.py` is an interactive hardware menu for OpenDoge EL05 motors.

It is intentionally EL05 / RobStride only. The PD02 tool package was used only as a reference for menu shape and operator workflow; OpenDoge does not use LK/Lingkong motors.

## Prerequisites

Bring up the production USB2CAN signal forwarding board as a Linux SocketCAN interface:

```bash
sudo ./scripts/setup_can.sh can0 1000000
```

From the workspace root:

```bash
./tools/el05/el05_motor_menu.py --channel can0 --master-id 0xfd
```

Optional custom motor ID order:

```bash
./tools/el05/el05_motor_menu.py --channel can0 --ids 1,2,3,4,5,6,7,8,9,10,11,12
```

## Menu

- `list motors`: show current joint-name to motor-id order.
- `read mech position/velocity params`: read `0x7019 mechPos` and `0x701B mechVel` without enabling the motor.
- `listen feedback frames`: print communication type 2 feedback frames already present on the bus.
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
- This tool is for bringup and validation. The production ROS path still needs a C++ `motor_control_interface/MotorHardware` implementation.
