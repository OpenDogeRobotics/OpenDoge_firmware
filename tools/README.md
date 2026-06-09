# OpenDoge Hardware Tools

This directory contains hardware bringup and validation tools for the real OpenDoge robot.

## Layout

- `el05/`: OpenDoge-specific EL05 / RobStride motor tools over the production USB2CAN signal forwarding board.
- `joystick/`: Xbox-compatible joystick to `command.state` bridge for non-ROS deployment.
- `usb2can/`: vendor/reference USB2CAN examples and notes.

## Current Rule

OpenDoge uses EL05 RobStride/RS motors. Do not add LK/Lingkong motor paths to this project unless the hardware changes.

The production motor path is:

```text
opendoge_deploy -> SocketCAN(can0/can1/can2/can3) -> USB2CAN signal forwarding board -> EL05 CAN bus
```

Use tools in `el05/` for OpenDoge bringup. Keep `usb2can/mi_motor_demo_TB.py` as the protocol reference sample.
