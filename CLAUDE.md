# OpenDoge Firmware — 四足机器人实机部署仓库

## 目录结构与职责

```
OpenDoge_firmware/
├── src/opendoge_deploy/     ← C++ 实机部署运行时 (生产代码)
│   ├── main.cpp              ← 主循环编排
│   ├── cli.cpp/hpp           ← CLI 参数解析
│   ├── controller.cpp/hpp    ← 状态机 + PD 控制回路
│   ├── safety.cpp/hpp        ← 安全监控 (力矩/跟踪/倒地/温度)
│   ├── observer.cpp/hpp      ← 观测构建 + 步态相位
│   ├── status.cpp/hpp        ← JSON 状态输出
│   ├── el05_socketcan.cpp/hpp ← EL05 CAN 协议层
│   ├── policy.cpp/hpp        ← 策略抽象层
│   ├── onnx_policy.cpp       ← ONNX Runtime backend
│   └── runtime_io.cpp/hpp    ← 文件 I/O
│
├── daemons/                  ← 运行时 I/O 适配守护进程 (部署必须)
│   ├── imu_bridge/           ← DM-IMU-L1 → /tmp/imu.state
│   └── command_bridge/       ← Xbox 手柄 → /tmp/command.state
│
├── bringup/                  ← 硬件 bringup/标定/验证 (部署前使用)
│   ├── el05/                 ← 电机交互菜单、标定、协议自检
│   └── usb2can/              ← vendor 参考示例
│
├── web_tools/                ← Web 控制台 (独立工具)
├── test/                     ← sim2sim 部署管线验证仿真
│   ├── deploy_mujoco.py      ← 复现 C++ 控制回路 (必须与 main.cpp 同步)
│   ├── calc_zero_offset.py   ← URDF 零位 → 趴伏补偿角
│   └── CLAUDE.md             ← 训练↔部署 Gap 追踪记录
│
├── scripts/                  ← Shell 编排 (CAN 启动、整机启动、systemd)
├── docs/URDF/                ← 机器人描述 (URDF + MuJoCo XML)
└── policy/                   ← ONNX 策略模型
```

## 核心不变量

### 1. EL05 协议一致性

EL05/RobStride CAN 协议在以下三处实现，参数必须一致：

| 位置 | 语言 | 用途 |
|------|------|------|
| `src/opendoge_deploy/src/el05_socketcan.cpp` | C++ | 生产部署 |
| `bringup/el05/el05_motor_menu.py` | Python | 硬件调测 |
| `bringup/el05/protocol_selftest.py` | Python | 协议自检 |

同步项：CAN ID 构造 (`buildExtId`)、float↔uint 映射范围 (P_MIN=-12.57, P_MAX=12.57, V_MIN=-50, V_MAX=50, T_MIN=-6, T_MAX=6, KP_MAX=500, KD_MAX=5)、comm_type 常量 (0x01-0x12)、参数索引 (0x7005, 0x7019, 0x701B, 0x3022)。

### 2. 控制回路同步 (C++ ↔ Python)

`src/opendoge_deploy/src/main.cpp` 和 `test/deploy_mujoco.py` 实现相同的控制回路。任何修改必须双向同步：

- 状态机转换规则 (`RuntimeState` enum, `updateStateMachine` / `DeployController.step`)
- 观测构建 (`buildObservation` / `build_observation`) — 49 维格式不可变
- 步态相位 (`advancePhase` / `advance_phase`) — 公式不可变
- rate limiter 语义 (RL 模式跳过, PC 模式 3 rad/s)
- PD 增益逻辑 (阻尼/斜坡/Active/LowGain)
- `last_action` 的 clamp 语义

### 3. 电机映射 (不可变)

```
can0: FL_hip/1, FL_thigh/2, FL_calf/3
can1: FR_hip/4, FR_thigh/5, FR_calf/6
can2: RL_hip/7, RL_thigh/8, RL_calf/9
can3: RR_hip/10, RR_thigh/11, RR_calf/12
```

### 4. 默认站立姿态 (必须匹配训练 keyframe)

```
前腿: hip=0, thigh=0.5, calf=-1.3
后腿: hip=0, thigh=0.7, calf=-1.3
```

### 5. 观测格式 (49 维，无 privileged info)

```
gyro(3) + neg_gravity(3) + dof_pos_diff(12) + dof_vel(12)
+ last_action(12) + commands(3) + feet_phase(4)
```

## Daemons 与 deploy 的接口

`opendoge_deploy` 不直接操作硬件。IMU 和手柄通过 `daemons/` 中的 Python 守护进程桥接为文件 IPC：

```
daemons/imu_bridge/dm_imu_bridge.py           → /tmp/opendoge_imu.state
daemons/command_bridge/xbox_command_bridge.py → /tmp/opendoge_command.state
```

C++ deploy 通过 `runtime_io.cpp` 的 `readImuFile()` / `readCommandFile()` 以 200 Hz 轮询这些文件。这是有意设计：
- 实时循环 (`SCHED_FIFO`) 免于处理串口/HID I/O
- 守护进程可独立重启
- dry-run 测试可直接写入文件注入假数据

### DM-IMU-L1 状态

| 属性 | 值 |
|------|-----|
| 硬件 | DM-Tech DM-IMU-L1 |
| USB ID | `6877:4d55` |
| 串口 | `/dev/ttyUSBx` (自动检测) |
| 桥接脚本 | `daemons/imu_bridge/dm_imu_bridge.py` |
| 输出文件 | `/tmp/opendoge_imu.state` |
| 数据格式 | `wx wy wz gx gy gz` (角速度 rad/s, projected gravity 已取反) |
| 当前状态 | ✅ 正常运行, `gz ≈ -1.0` (竖直), gyro 噪声 < 0.02 rad/s |

验证命令：
```bash
cat /tmp/opendoge_imu.state       # 查看实时 IMU 数据
lsusb | grep 6877:4d55             # 确认 IMU 已连接
```

### Xbox 2.4G 手柄状态

| 属性 | 值 |
|------|-----|
| 硬件 | 2.4G XBOX 360 For Windows (克隆 dongle) |
| USB ID | `413d:2104` |
| 驱动 | xboxdrv (用户态, 需剥离内核 hid-generic) |
| 设备节点 | `/dev/input/js0` → `Xbox Gamepad (userspace driver)` |
| 桥接脚本 | `daemons/command_bridge/xbox_command_bridge.py` |
| 输出文件 | `/tmp/opendoge_command.state` |
| 当前状态 | ✅ 正常运行 |

**轴/按键映射** (xboxdrv 默认, 已验证):

| 物理输入 | js 事件 | 功能 |
|----------|---------|------|
| 左摇杆 Y | axis1 | vx (前后速度) |
| 左摇杆 X | axis0 | vy (侧向速度) |
| 右摇杆 X | axis2 | yaw_rate (转向角速度) |
| A | btn0 | 激活 + 位置控制模式 |
| B | btn1 | 失能 |
| X | btn2 | 进入 RL 推理 |
| Y | btn3 | 退出 RL 推理 |
| BACK | btn6 | 切换 low_gain_mode |
| START | btn7 | 切换使能 |
| RB | btn5 | 死手开关 (`--require-rb`) |

**systemd 服务** (开机自启):

```
opendoge-xboxdrv.service   (system) → xboxdrv, Type=simple, Restart=always
opendoge-joystick.service  (user)   → bridge, Type=simple, Restart=always
```

服务链：`opendoge-xboxdrv` → `/dev/input/js0` → `opendoge-joystick` → `/tmp/opendoge_command.state`

**稳定性机制**:
- xboxdrv 崩溃 → systemd `Restart=always`, 5s 后自动拉起
- js0 消失 → bridge 捕获 `DeviceLostError`, 写安全中性命令, 自动重连
- USB 热插拔 → `ExecStartPre` 等待 dongle/js0 出现 (最多 30s)
- 无限重启次数 (`StartLimitIntervalSec=0`)

日常管理：
```bash
systemctl status opendoge-xboxdrv              # xboxdrv 状态
systemctl --user status opendoge-joystick      # bridge 状态
journalctl --user -u opendoge-joystick -f      # bridge 实时日志
watch -n 0.2 cat /tmp/opendoge_command.state   # 监控手柄命令
sudo journalctl -u opendoge-xboxdrv -f         # xboxdrv 实时日志
```

安装/重装：
```bash
bash scripts/install_services.sh
```

## 构建与测试

```bash
# 构建
export ONNXRUNTIME_ROOT=$(realpath build/deps/onnxruntime)
colcon build --symlink-install --packages-select opendoge_deploy
source install/setup.bash

# dry-run (无硬件)
./install/opendoge_deploy/bin/opendoge_deploy --policy-backend none --duration-sec 2

# MuJoCo 仿真验证
cd test && python3 deploy_mujoco.py --mode idle --no-render --duration 2

# vcan 无硬件 CAN 测试
sudo ./scripts/setup_vcan.sh can0 can1 can2 can3
./install/opendoge_deploy/bin/opendoge_deploy --real --enable --allow-missing-imu --policy-backend none --duration-sec 1
```

## 修改指南

1. **新增配置参数**: `types.hpp` → `runtime_io.cpp` → `main.cpp`
2. **修改控制回路**: `controller.cpp` + `deploy_mujoco.py` 同步
3. **修改 CAN 协议**: `el05_socketcan.cpp` + `bringup/el05/el05_motor_menu.py` + `protocol_selftest.py` 同步
4. **新增 policy backend**: 实现 `Policy` 接口，在 `makePolicy()` 注册
5. **XML 物理参数对齐**: 固件 `Opendoge.xml` 的物理参数必须对齐 UniLab 训练 `opendoge.xml`+`scene_flat.xml`，追踪记录见 `test/CLAUDE.md`

## 相关仓库

- UniLab 训练框架: `../UniLab/`
- OpenDoge 硬件设计: `../OpenDoge_hardware/`
- 模块详细文档: `src/opendoge_deploy/CLAUDE.md`
