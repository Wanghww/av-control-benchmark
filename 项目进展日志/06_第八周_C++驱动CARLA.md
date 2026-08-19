# 第八周：C++ 控制器驱动 CARLA 仿真

## 当前进度

第二阶段第八周完成，**第二阶段全部完成**。

---

## 已完成内容

### 新增文件

```
cpp_controllers/src/carla_interface.cpp     ← C++ 控制器进程（stdin/stdout 通信）
python_prototype/carla_bridge/run_carla_cpp.py  ← Python CARLA 驱动脚本
python_prototype/results/carla_cpp_pure_pursuit.png  ← 仿真结果图表
```

---

### 系统架构

```
Python（run_carla_cpp.py）          C++（carla_interface）
  ├── CarlaEnv 读取车辆状态           ├── 从 trajectory.csv 加载轨迹
  ├── 状态写入 stdin ─────────────→  ├── Pure Pursuit 计算 delta
  ├── 从 stdout 读取控制指令 ←──────  ├── PID 计算 accel
  └── 发送给 CARLA 车辆              └── 输出 throttle steer brake
```

通信格式：
- **Python → C++（stdin）：** `x y theta v\n`
- **C++ → Python（stdout）：** `throttle steer brake\n`

---

### 仿真结果

| 指标 | Python 控制器 | C++ 控制器 |
|------|-------------|-----------|
| RMSE | 0.794 m | **0.794 m** |
| 最大误差 | 3.14 m | 3.14 m |

两者结果**完全一致**，证明 C++ 实现与 Python 版逻辑等价。

---

## 第二阶段完整总结

| 产出 | 验证方式 | 结果 |
|------|---------|------|
| CMake 工程编译 | `cmake .. && make -j4` | ✓ |
| 单元测试 4 个 | `./test_controllers` | 4/4 PASSED ✓ |
| C++ 仿真 RMSE | `./run_simulation` | 0.108 m ✓ |
| C++ 驱动 CARLA | `run_carla_cpp.py` | RMSE=0.794 m ✓ |
| Stanley 控制器 | 编译通过 | ✓ |

---

## 第三阶段预告

将控制器封装为 ROS2 节点，实现标准的自动驾驶软件架构：
- 发布 `/cmd_vel` 控制话题
- 订阅 `/odom` 里程计话题
- 用 ROS2 bag 记录仿真数据
- 与 ROS2 生态（rviz2、rosbag2）完整集成
