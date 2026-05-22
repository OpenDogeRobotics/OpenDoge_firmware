# opendoge_rl_node

RK3588 侧强化学习推理桥接节点。节点订阅电机 `joint_state`、`/imu`、`/joy` 和 `/cmd_vel`，构造策略 observation，以 50 Hz 运行策略，以 200 Hz 发布 POS 目标到 `/joint_target`。底层电机控制器仍按 500 Hz 执行，消费最新 `joint_target`。

## 通信接口

- 输入：`/joint_state` (`sensor_msgs/msg/JointState`)，电机位置/速度反馈。
- 输入：`/imu` (`sensor_msgs/msg/Imu`)，机身角速度和姿态。
- 输入：`/joy` (`sensor_msgs/msg/Joy`)，手柄死区、使能、模式切换和急停。
- 输入：`/cmd_vel` (`geometry_msgs/msg/Twist`)，可选速度指令。
- 输出：`/joint_target` (`sensor_msgs/msg/JointState`)，12 关节 POS 目标。
- 调试：`/rl_observation`、`/rl_action` (`std_msgs/msg/Float64MultiArray`)。

默认参数在 `config/opendoge_rl.yaml`。`policy_backend` 当前支持：

- `none`：构建和联调模式，发布默认站姿。
- `linear_csv`：轻量 CSV 线性策略，用于验证 observation/action 管线。
- `rknn`：RK3588 RKNN 接入预留，安装 Rockchip `rknnrt` 后把后端替换为真实 NPU 推理。

## 构建
```bash
cd /home/lain/OpenDoge/OpenDoge_firmware
colcon build --symlink-install --packages-select opendoge_rl_node
source install/setup.bash
```

## 运行
```bash
ros2 launch opendoge_rl_node rl_node.launch.py
```

覆盖 RKNN 参数示例：

```bash
ros2 launch opendoge_rl_node rl_node.launch.py policy_backend:=rknn policy_path:=/home/lain/policy/opendoge.rknn
```

## 手柄结构

- `joy_button_deadman`：按住后摇杆才会写入速度指令。
- `joy_button_standby`：进入低幅站立输出。
- `joy_button_running`：进入 RL 行走输出。
- `joy_button_passive`：回到 passive。
- `joy_button_estop`：进入 fault，持续输出安全目标。

轴映射由 `joy_axis_x`、`joy_axis_y`、`joy_axis_yaw` 配置，速度上限由 `max_cmd_x/y/yaw` 限制。
