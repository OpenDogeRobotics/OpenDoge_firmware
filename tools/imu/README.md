# DM-IMU Bridge

`dm_imu_bridge.py` converts DM-IMU-L1 data to the OpenDoge `imu.state` file consumed by `opendoge_deploy`.

Output format:

```text
wx=...
wy=...
wz=...
gx=...
gy=...
gz=...
```

The bridge uses gyroscope data for `w*` and quaternion attitude for projected gravity `g*`.

## USB / RS485 Active Frames

Configure the IMU to output gyro and quaternion frames, then run:

```bash
./tools/imu/dm_imu_bridge.py --source serial --device /dev/ttyUSB0 --baud 921600 --output /tmp/opendoge_imu.state
```

## CAN Frames

Use a SocketCAN interface dedicated to the IMU or a bus where ID collision is impossible:

```bash
./tools/imu/dm_imu_bridge.py --source can --can can4 --can-id 0x01 --output /tmp/opendoge_imu.state
```

## Axis Alignment

The default assumes the IMU coordinate frame is mounted the same as the robot base frame:

```text
robot x = imu x
robot y = imu y
robot z = imu z
```

If the physical mounting differs, use:

```bash
--axis-map xyz --axis-signs 1,1,1
```

`--axis-map` selects which IMU axes become robot x/y/z. `--axis-signs` flips the mapped axes.
