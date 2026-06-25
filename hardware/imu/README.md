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

The deployment path uses the USB virtual serial active frame protocol:

```text
55 AA ID 02 gyro_x gyro_y gyro_z CRC16 0A
55 AA ID 04 quat_w quat_x quat_y quat_z CRC16 0A
```

Configure the IMU to output gyro and quaternion frames, then run:

```bash
./hardware/imu/dm_imu_bridge.py --device /dev/ttyUSB0 --baud 921600 --output /tmp/opendoge_imu.state
```

To configure the module through USB quick commands before reading:

```bash
./hardware/imu/dm_imu_bridge.py --device /dev/ttyUSB0 --baud 921600 --configure-usb --output /tmp/opendoge_imu.state
```

Add `--check-crc` to validate USB serial CRC16 before accepting frames.

## CAN Frames

Use a SocketCAN interface dedicated to the IMU or a bus where ID collision is impossible:

```bash
./hardware/imu/dm_imu_bridge.py --source can --can can4 --can-id 0x01 --output /tmp/opendoge_imu.state
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
