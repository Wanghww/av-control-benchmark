# 第四周：接入 CARLA 仿真

## 当前进度

第一阶段第四周完成，第一阶段全部完成。

---

## 已完成内容

### 新增文件结构

```
python_prototype/carla_bridge/
├── __init__.py
├── carla_env.py            ← CARLA 连接、车辆控制
├── waypoint_trajectory.py  ← 从地图提取路点生成轨迹
└── run_carla_sim.py        ← CARLA 仿真主程序

python_prototype/results/
├── bicycle_pure_pursuit.png   ← 第三周：自行车模型仿真结果
└── carla_pure_pursuit.png     ← 第四周：CARLA 仿真结果
```

---

### 系统架构

```
Windows（第二台电脑）
  └── CARLA 0.9.15（仿真服务器，TCP 172.22.0.1:2000）
        ↕ 同步模式 tick（0.05s/步）
WSL2 Ubuntu（第二台电脑）
  └── run_carla_sim.py
        ├── CarlaEnv       → 读取车辆状态 [x, y, psi, v]
        ├── PurePursuitController → 计算转角 delta
        ├── PIDController  → 计算加速度 accel
        └── 发送控制指令 (steer, throttle, brake)
```

### CARLA 坐标系处理

CARLA 使用左手坐标系（Y 轴朝南），与标准数学坐标系（Y 轴朝北）相反。正确的转换方式：

| 量 | CARLA 原始 | 转换方式 | 含义 |
|----|-----------|---------|------|
| y | tf.location.y（朝南为正） | `y = -y_carla` | 转为朝北为正 |
| yaw | 顺时针角度（度） | `psi = -radians(yaw)` | 转为逆时针弧度 |
| steer | +1=右转 | `steer = -delta / MAX_STEER` | Pure Pursuit delta>0=左转 |

**关键经验：Y 轴取反和转向取反必须同时满足，缺一会导致 U 形掉头。**

### 轨迹生成改进

使用车头前向量点积确保路点方向与车头一致，避免找到对向车道：

```python
fwd = start_transform.get_forward_vector()
for dist in [3, 6, 10, 15, 20]:
    loc = start_transform.location + carla.Location(x=fwd.x*dist, y=fwd.y*dist)
    wp = carla_map.get_waypoint(loc)
    dot = fwd.x * wp.transform.get_forward_vector().x + ...
    if dot > 0:  # 方向一致
        break
```

### Pure Pursuit 改进

新增 `min_idx` 参数，防止最近点向后跳导致车辆追赶错误目标：

```python
delta, v_ref, nearest_idx = pure_pursuit.compute(state, trajectory, min_idx=nearest_idx)
```

---

## 仿真结果

| 指标 | 值 | 达标要求 |
|------|-----|---------|
| RMSE | **0.79 m** | < 1.0 m ✓ |
| 最大误差 | 3.14 m（Step 0 生成偏差） | — |
| 仿真时长 | 30s（600步 × 0.05s） | — |
| 稳态误差（Step 50后） | 0.2~1.0 m | — |

---

## 第一阶段总结

| 产出 | 验证方式 | 结果 |
|------|---------|------|
| WSL2 + SSH + VS Code 环境 | VS Code 左下角 SSH: wsl-second | ✓ |
| 动态自行车模型（RK4 积分） | 直行 1s x=5.00m | ✓ |
| PID 纵向控制器 | 速度跟踪稳定 | ✓ |
| Pure Pursuit 横向控制器 | S 弯 RMSE=0.33m | ✓ |
| CARLA 仿真跑通 | Town03 RMSE=0.79m | ✓ |

---

## 第二阶段预告

用 C++ 重写动态自行车模型和控制器，性能提升约 10-50 倍，为后续 MPC 实时求解做准备。
