# OpenDoge MuJoCo 仿真测试

```bash
cd /home/lain/OpenDoge/OpenDoge_firmware && python3 test/deploy_mujoco.py --mode idle
```

cd /home/lain/OpenDoge/OpenDoge_firmware && python3 test/deploy_mujoco.py --mode rl --policy policy/opendoge_r5.onnx


## 测试脚本

| 脚本 | 用途 | 用法 |
|---|---|---|
| `deploy_mujoco.py` | OpenDoge MuJoCo 仿真，复现 deploy 控制回路（状态机、PD 控制、RL 推理） | `python3 test/deploy_mujoco.py --mode idle` |
| `calc_zero_offset.py` | 计算 URDF 零位 → 平地趴伏的补偿角度 | `python3 test/calc_zero_offset.py` |

## 依赖

```bash
pip install mujoco numpy
```

## deploy_mujoco.py 用法

```bash
python3 test/deploy_mujoco.py                          # 位置控制模式
python3 test/deploy_mujoco.py --mode idle              # 待机模式 (阻尼趴伏)
python3 test/deploy_mujoco.py --mode rl                # RL 推理模式 (模拟)
python3 test/deploy_mujoco.py --duration 10            # 运行 10 秒
python3 test/deploy_mujoco.py --no-render              # 无渲染 (headless)
python3 test/deploy_mujoco.py --cmd 0.3 0 0            # 静态速度命令
```

### 键盘控制 (渲染窗口)

| 按键 | 功能 |
|---|---|
| A / Space | 激活位置控制 (EnteringPosition → ActivePC) |
| B | 停用 (→ Ready, 阻尼) |
| X | 切换到 RL 推理模式 |
| Y | 切换回位置控制模式 |
| Backspace | 急停 (estop) |
| Esc / Q | 退出 |

## 碰撞体配置 (Opendoge.xml)

| Geom | contype | conaffinity | 说明 |
|---|---|---|---|
| base_link mesh | `1` | `1` | 机身碰撞 — 趴伏时撑在地面 |
| hip mesh | 继承默认 `1` | `1` | 髋关节 mesh 碰撞 |
| thigh mesh | `0` | `0` | 视觉 mesh — 趴伏时悬空，不碰地 |
| calf mesh | 继承默认 `1` | `1` | 小腿 mesh 碰撞 |
| 脚底球体 | `1` | `1` | 足端碰撞球 (r=0.015) |
| 脚视觉 mesh | `0` | `0` | 足端视觉 mesh |

默认 `<geom contype="1" conaffinity="1"/>`，只对 thigh 显式关闭碰撞。

## 补偿角 (URDF 零位 → 平地趴伏)

全 mesh 碰撞 (thigh 除外), 关节失能仅阻尼 kd=2.0, 自由落体 10-15s 稳态实测:

```
DEFAULT_POS = np.array([
    0.230,  1.079, -2.681,   # FL
   -0.230,  1.079, -2.681,   # FR
    0.231,  1.090, -2.681,   # RL
   -0.230,  1.084, -2.681,   # RR
])
```

| 关节 | FL | FR | RL | RR |
|---|---|---|---|---|
| hip | +0.230 | -0.230 | +0.231 | -0.230 |
| thigh | +1.079 | +1.079 | +1.090 | +1.084 |
| calf | -2.681 | -2.681 | -2.681 | -2.681 |

- Calf 全部触及关节下限 -2.68 rad (mesh 碰撞硬约束)
- Thigh 接近但未及上限 +1.134
- 碰撞配置变更后需重跑 `calc_zero_offset.py` 验证
