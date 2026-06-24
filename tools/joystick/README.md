# OpenDoge Joystick Bridge

Xbox 手柄 → `/tmp/opendoge_command.state` 桥接工具，供 `opendoge_deploy` 运行时读取。

## 文件清单

| 文件 | 用途 |
|------|------|
| `xbox_command_bridge.py` | **主桥接程序** — 读取 `/dev/input/js0`，写入命令文件 |
| `opendoge_joystick.py` | 共享库 — `LinuxJoystick` / `XboxCommandMapper` / `atomic_write` |
| `start_joystick_bridge.sh` | 启动器 — 先拉起 xboxdrv，再启动桥接 |
| `diag_joystick.py` | 诊断工具 — 检测 xboxdrv/js0 是否正常工作 |
| `gamepad_tester.html` | 网页手柄测试页 — 浏览器中查看手柄状态 |

## 快速开始

### 1. 确保 xboxdrv 运行

```bash
# 启动 xboxdrv（需 sudo，detach kernel driver）
sudo xboxdrv --device-by-id 413d:2104 --type xbox360 --detach-kernel-driver --silent &

# 确认 js0 已出现
ls -la /dev/input/js0
cat /sys/class/input/js0/device/name    # 应显示 "Xbox Gamepad (userspace driver)"
```

> 如果 USB ID 不同，用 `lsusb` 查找手柄的 `xxxx:xxxx` 替换 `413d:2104`。

### 2. 运行桥接

```bash
./tools/joystick/xbox_command_bridge.py --require-rb
```

或使用启动器脚本（自动处理 xboxdrv）：

```bash
./tools/joystick/start_joystick_bridge.sh --require-rb
```

### 3. 验证

```bash
# 观察命令文件实时更新
watch -n 0.2 cat /tmp/opendoge_command.state
```

按下手柄 A 键后应看到 `active=true`、`position_control=true`，推动摇杆应看到 vx/vy/yaw 数值变化。

## 手柄映射

```
  Left stick   -> vx (前后) / vy (左右)
  Right stick  -> yaw_rate (转向)

  A            -> 启动机器人 + 位置控制模式 (position_control)
  B            -> 机器人失能 / 电机保护 (active=false)
  X            -> 进入 RL 推理状态 (rl_inference)
  Y            -> 退出 RL 推理状态
  BACK         -> 急停 (estop)
  START        -> 切换使能
  RB (hold)    -> 死手开关 (--require-rb)
```

状态组合：

| 手柄操作 | active | position_control | rl_inference | estop |
|----------|--------|------------------|--------------|-------|
| 空闲 | false | false | false | false |
| 按 A | true | true | false | false |
| 按 A 后按 X | true | false | true | false |
| RL 中按 Y | true | true | false | false |
| 按 B | false | false | false | false |
| 按 BACK | false | false | false | **true** |

## 命令行参数

```
xbox_command_bridge.py [选项]

  --device PATH         手柄设备路径 (默认: /dev/input/js* 中第一个)
  --output PATH         命令文件输出路径 (默认: /tmp/opendoge_command.state)
  --rate-hz HZ          更新频率 (默认: 100)
  --deadzone VAL        摇杆死区 (默认: 0.08)
  --max-vx VAL          前进最大速度 m/s (默认: 0.6)
  --max-vy VAL          侧向最大速度 m/s (默认: 0.4)
  --max-yaw-rate VAL    转向最大角速度 rad/s (默认: 1.0)
  --require-rb          要求按住 RB 才输出有效命令 (死手)
  --quiet               不打印周期性状态
  --axis-vx N / --axis-vy N / --axis-yaw N   自定义轴号
  --btn-a N / --btn-b N / --btn-x N / --btn-y N ...  自定义按键号
```

## systemd 服务（开机自启）

桥接已固化为 systemd 服务，开机自动运行，无需手动启动。

### 安装

```bash
# 一键安装（推荐）
./scripts/install_services.sh

# 或手动安装用户级桥接服务（无需 sudo）
mkdir -p ~/.config/systemd/user
cp scripts/opendoge-joystick.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now opendoge-joystick
loginctl enable-linger   # 确保用户服务在开机时启动
```

### 日常管理

```bash
# 状态
systemctl --user status opendoge-joystick

# 实时日志
journalctl --user -u opendoge-joystick -f

# 重启 / 停止
systemctl --user restart opendoge-joystick
systemctl --user stop opendoge-joystick
```

### 服务架构

```
opendoge-xboxdrv.service (系统级, root)     opendoge-joystick.service (用户级, orangepi)
         │                                              │
         │ 提供 /dev/input/js0                            │ 读取 js0
         │                                              │ 写入 /tmp/opendoge_command.state
         ▼                                              ▼
     [xboxdrv] ──── /dev/input/js0 ──── [xbox_command_bridge.py]
                                                │
                                                ▼
                                    /tmp/opendoge_command.state
                                                │
                                                ▼
                                    [opendoge_deploy]
```

> **注意**: `opendoge-xboxdrv.service` 是系统级 service（需 sudo 安装一次），负责在开机时启动 xboxdrv。如果 xboxdrv 已通过其他方式运行，只需安装 `opendoge-joystick.service` 即可。

### 与 start_robot.sh 的集成

`start_robot.sh` 的 `start_joystick_bridge()` 函数会优先检测 systemd 服务：

- 如果 `opendoge-joystick.service` 已在运行 → **跳过**，打印提示
- 如果未运行 → 回退到直接 spawn 桥接进程（兼容未安装 systemd 服务的场景）

## 诊断

如果手柄不响应：

```bash
# 1. 检查 xboxdrv
pgrep -la xboxdrv

# 2. 检查 js0
ls -la /dev/input/js0
cat /sys/class/input/js0/device/name

# 3. 运行诊断工具
./tools/joystick/diag_joystick.py

# 4. 检查桥接日志
journalctl --user -u opendoge-joystick -n 50

# 5. 手动测试 — 直接读取 js0 事件
cat /dev/input/js0 | xxd    # 按手柄按键应看到数据
```

### 常见问题

| 症状 | 原因 | 解决 |
|------|------|------|
| `No joystick found` | js0 不存在 | 启动 xboxdrv 或检查 USB 连接 |
| 按键无反应 | xboxdrv 未正确启动 | `sudo killall xboxdrv && sudo xboxdrv --device-by-id 413d:2104 --type xbox360 --detach-kernel-driver --silent &` |
| USB ID 不匹配 | 手柄/dongle 的 USB ID 不是 `413d:2104` | `lsusb` 查找正确 ID，通过环境变量 `XBOX_USB_ID=xxxx:xxxx` 覆盖 |
| 命令文件不更新 | 桥接进程挂了 | `systemctl --user restart opendoge-joystick` |
| RB 按住才能动 | 正常的 — `--require-rb` 死手保护 | 去掉 `--require-rb` 关闭此行为 |
