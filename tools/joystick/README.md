# OpenDoge Joystick Command Bridge

This directory provides a non-ROS Xbox-compatible joystick bridge for real OpenDoge deployment.

It reads Linux joystick events from `/dev/input/js0` and writes the command file consumed by:

```bash
./install/opendoge_deploy/bin/opendoge_deploy --command-file /tmp/opendoge_command.state
```

## Controls

```text
Left stick Y: vx forward/back
Left stick X: vy left/right
Right stick X: yaw_rate
A: active=true
B: active=false
START: toggle active
X or BACK: estop=true
Y: clear local estop output for the next run
```

Optional `--require-rb` makes RB a deadman switch. When RB is released, the bridge writes `active=false`.

## Run

Start the bridge:

```bash
./tools/joystick/xbox_command_bridge.py --output /tmp/opendoge_command.state --require-rb
```

In another terminal, run deployment with the same command file:

```bash
./install/opendoge_deploy/bin/opendoge_deploy \
  --policy-backend onnx \
  --policy-path /home/lain/OpenDoge/OpenDoge_firmware/gen52_model4800.onnx \
  --command-file /tmp/opendoge_command.state \
  --imu-file /tmp/opendoge_imu.state
```

Conservative default command limits:

```text
max_vx=0.6 m/s
max_vy=0.4 m/s
max_yaw_rate=1.0 rad/s
```

Tune with `--max-vx`, `--max-vy`, and `--max-yaw-rate` after bench testing.
