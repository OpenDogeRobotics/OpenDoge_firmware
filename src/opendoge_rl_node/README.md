# opendoge_rl_node (C++) 最小真机桥接示例

功能：订阅 `/imu`（sensor_msgs/Imu）和 `/robot_joint_controller/state`（robot_msgs/RobotState），发布 `/robot_joint_controller/command`（robot_msgs/RobotCommand）。当前策略推理为占位（零输出+保持姿态），保留 FSM 钩子与超时回退安全逻辑。

## 参数（launch 可传）
- `policy_path`：策略文件路径（占位）。
- `num_dofs`：关节数，默认 12。
- `update_rate_hz`：控制循环频率，默认 500。
- `timeout_state_ms` / `timeout_imu_ms`：超时阈值，超时回退安全指令。
- `safe_kp` / `safe_kd` / `safe_tau`：安全/占位输出的 PD 与力矩。
- `command_topic` / `state_topic` / `imu_topic`：话题名，默认 `/robot_joint_controller/command`、`/robot_joint_controller/state`、`/imu`。

## 构建
```powershell
cd C:\Users\com01\Desktop\rl_sar\ROS_WS\opendoge_ws
colcon build --symlink-install --packages-select opendoge_rl_node
call install\setup.bat
```

## 运行
```powershell
ros2 launch opendoge_rl_node rl_node.launch.py policy_path:=/path/to/policy.pt
```

## 后续接入策略
- 在 `rl_node.cpp` 中替换 `publishHoldPositionCommand` 的占位逻辑，接入 RL_SAR 推理（TorchScript/ONNX/自定义库）。
- 可扩展 FSM：增加状态枚举与切换条件（如 Passive/GetUp/RL/Fault），在 `controlLoop` 内分支调用不同控制策略。

