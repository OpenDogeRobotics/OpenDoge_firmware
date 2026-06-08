# OpenDoge Firmware

适用：Linux 实机部署环境，四足机器人 **OpenDoge**。

当前仓库的主线是非 ROS 单进程强化学习部署：`opendoge_deploy` 直接管理 CPU ONNX 推理、4 路 SocketCAN、EL05 电机运控帧和安全阻尼。ROS / ros2_control 控制链已经从本仓库移除，避免实机路径被 controller manager、topic queue 或 ROS executor 额外耦合。

详细状态记录见 [Changelog/Codex.md](Changelog/Codex.md)。

## 目录结构

```text
OpenDoge_firmware/
  README.md
  Changelog/Codex.md
  docs/                            # EL05 手册、原理图等参考文档
  requirements.txt                  # Python 参考工具依赖
  scripts/
    setup_can.sh                    # USB2CAN SocketCAN 启动脚本
  src/
    opendoge_deploy/                # 非 ROS 实机部署主程序
  tools/
    el05/                           # EL05/RobStride 交互式硬件工具
    usb2can/                        # USB2CAN 示例和参考说明
  build/ install/ log/              # colcon 生成，git 忽略
```

## 主部署链路

```text
opendoge_deploy
  -> CPU ONNX policy 50 Hz
  -> position target hold 200 Hz
  -> EL05 q/dq/tau/kp/kd control 1000 Hz
  -> SocketCAN can0/can1/can2/can3
  -> USB 转 4 路 CAN2.0 模块
  -> EL05 motors
```

`opendoge_deploy` 不依赖 ROS、rclcpp、ros2_control、hardware_interface、controller_interface 或 controller_manager。

## 电机映射

每条 CAN 总线带一条腿 3 个电机：

```text
can0: 左前 FL, motor 1/2/3  = hip/thigh/calf
can1: 右前 FR, motor 4/5/6  = hip/thigh/calf
can2: 左后 RL, motor 7/8/9  = hip/thigh/calf
can3: 右后 RR, motor 10/11/12 = hip/thigh/calf
```

部署侧、真实 URDF 和训练侧统一使用关节名：

```text
FL_hip_joint, FL_thigh_joint, FL_calf_joint,
FR_hip_joint, FR_thigh_joint, FR_calf_joint,
RL_hip_joint, RL_thigh_joint, RL_calf_joint,
RR_hip_joint, RR_thigh_joint, RR_calf_joint
```

## 构建

```bash
cd /home/lain/OpenDoge/OpenDoge_firmware
colcon build --symlink-install --packages-select opendoge_deploy
source install/setup.bash
```

如果要启用 ONNX 后端，需要安装 ONNX Runtime C/C++ 运行库，并让 CMake 能找到 `onnxruntime_cxx_api.h` 和 `libonnxruntime.so`。未找到 ONNX Runtime 时，`opendoge_deploy` 仍会构建，但只支持 `none` / `linear_csv` 后端。

## 运行

dry-run，不打开 CAN、不发送电机帧：

```bash
./install/opendoge_deploy/bin/opendoge_deploy --policy-backend none --duration-sec 2
```

实机运行前启动四路 CAN：

```bash
sudo ./scripts/setup_can.sh can0 1000000
sudo ./scripts/setup_can.sh can1 1000000
sudo ./scripts/setup_can.sh can2 1000000
sudo ./scripts/setup_can.sh can3 1000000
```

实机阻尼/站姿安全测试：

```bash
./install/opendoge_deploy/bin/opendoge_deploy --real --enable --policy-backend none
```

ONNX 策略运行：

```bash
./install/opendoge_deploy/bin/opendoge_deploy --real --enable --policy-backend onnx --policy-path /path/to/opendoge.onnx
```

`--enable` 会参考 `mi_motor_demo_TB.py` 的流程发送运控模式设置和电机使能。部署程序不会自动执行机械置零。

## 安全策略

`opendoge_deploy` 在以下情况进入阻尼模式：

- 电机反馈故障位非 0。
- 电机温度超过阈值。
- 电机反馈超时。
- CAN 打开、发送或接收异常。

阻尼输出语义：

```text
q = 当前反馈位置
dq = 0
tau = 0
kp = 0
kd = safe_kd
```

## 上机前检查

- 电机 ID、CAN 通道、腿部布线一致。
- 关节正方向和机械零点 offset 已标定。
- 真实 URDF 使用 `/home/lain/OpenDoge/OpenDoge_description/URDF`，不要使用 firmware 内的旧占位描述。
- 软件限位、position rate limit、velocity limit、torque limit 已设置为保守值。
- EL05 单电机、单腿测试通过后再启用 12 电机。
- ONNX observation/action 数值已和训练侧 replay 对齐。
- IMU 坐标系和重力投影方向确认后再启用完整行走策略。
