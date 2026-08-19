# 第二十七至三十二周：第三层 — CARLA 全集成

## 当前进度

**第七阶段完成。** CARLA 桥接节点开发完成，ROS2 控制器成功接入 CARLA 物理引擎，演示 GIF 已录制。

---

## 本阶段新增文件

| 文件 | 说明 |
|------|------|
| `ros2_ws/src/av_controller/scripts/carla_bridge_node.py` | CARLA 桥接节点（替换 sim_bridge_node） |
| `ros2_ws/src/av_controller/launch/carla_system.launch.py` | CARLA 全集成 launch 文件 |
| `results/carla_demo.gif` | CARLA 演示 GIF（原始版） |
| `results/carla_demo_small.gif` | CARLA 演示 GIF（压缩版，用于 README） |

---

## 系统架构（第三层最终形态）

```
controller_node（C++ Pure Pursuit）
  ↕  /trajectory（直线 200m，4 m/s）
  ↕  /vehicle_state（20Hz，本地坐标系）
  ↕  /vehicle_cmd（转角 + 油门）
carla_bridge_node（Python）
  ↕  CARLA Python API（172.22.0.1:2000）
CARLA 物理引擎（Town03，Tesla Model 3）
```

原有 `sim_bridge_node.py`（Python 自行车模型）和 `full_system.launch.py` 完全保留。

---

## 关键工程实现

### 坐标系处理

CARLA 使用左手坐标系（Y 轴朝右，yaw 顺时针），ROS 使用右手坐标系。

转换方案：
1. **2s 延迟采样基准位姿**：物理引擎稳定后再记录起始位姿，避免 yaw 基准漂移
2. **本地坐标系**：以起始位姿为原点，发布相对位置，轨迹从 (0,0) 出发，无需修改控制器
3. **Y 轴取反 + 旋转矩阵**：`ros_y = -(carla_y - carla_y0)`，再旋转到起始航向对齐的坐标系

### 诊断验证（直线轨迹调试）

使用直线轨迹（y=0）验证坐标系正确性，获得以下数据：

```
state: x=4.95  y=0.000  θ=0.00°  v=3.56 m/s  ✅ 完美直行
state: x=12.09 y=0.000  θ=0.00°  v=3.60 m/s  ✅
state: x=19.34 y=0.001  θ=0.01°  v=3.63 m/s  ✅
...（直行约 12 秒直至道路边界）
```

**结论：坐标系转换正确，Pure Pursuit 在 CARLA 中正常工作。**

---

## 演示 GIF 说明

- **地图**：Town03（CARLA 标准城市地图）
- **车辆**：Tesla Model 3
- **控制器**：Pure Pursuit，前视距离 10m
- **速度**：4 m/s
- **时长**：约 11 秒，车辆稳定沿直线行驶

GIF 展示了：ROS2 控制器节点 → CARLA 物理引擎 → 车辆受控行驶的完整闭环。

---

## 三层仿真完成总结

| 层次 | 仿真器 | 控制器差距（15 m/s，PP vs 最优）| 状态 |
|------|--------|--------------------------------|------|
| 第一层 | 运动学模型 | 6% | ✅ |
| 第二层 | 动态自行车模型 | 39% | ✅ |
| 第三层 | CARLA 物理引擎 | 演示 GIF 完成 | ✅ |

三层递进验证完成。

---

## 阶段结论

项目最终完成了 ROS2 控制器与 CARLA 物理引擎的全集成。控制器节点通过 ROS2 话题接收轨迹、发布控制指令，CARLA 桥接节点负责坐标系转换和 CARLA API 调用，实现了完整的感知-控制闭环。CARLA 使用左手坐标系，通过延迟采样基准位姿和旋转矩阵对齐解决了坐标系转换问题，最终在 Town03 里跑通了演示。

---

## 下一步：整理 GitHub README

项目三层仿真全部完成，下一步是整理 GitHub 仓库：
- README 包含演示 GIF、三层对比数据、技术栈说明
- 仓库对外公开发布
