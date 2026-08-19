# 第七阶段：第三层 — ROS2 + CARLA 全集成（第 27-32 周）

## 本阶段目标

将 ROS2 控制器节点连接真实 CARLA 仿真环境，替换掉当前的 Python 自行车模型桥接节点，实现完整的"感知-规划-控制"软件架构闭环。

**里程碑：CARLA 中的车辆通过 ROS2 话题接受控制指令，跟踪双移线轨迹，录制演示 GIF。**

---

## 为什么这一步重要

### 当前架构（第二层）

```
轨迹发布节点 → /trajectory → 控制器节点 → /vehicle_cmd → Python仿真桥接节点
                                                              ↓
                                                      (动态自行车模型)
```

Python 自行车模型是我们自己写的，参数理想、无噪声、无延迟，相当于"开卷考试"。

### 目标架构（第三层）

```
轨迹发布节点 → /trajectory → 控制器节点 → /vehicle_cmd → CARLA 桥接节点
                                                              ↓
                                                      (CARLA 物理引擎)
                                                        真实车辆动力学
                                                        传感器噪声
                                                        执行延迟
```

CARLA 有真实的车辆物理模型（Unreal Engine PhysX），比任何手写模型都更接近真实车辆。

### 三层横向对比价值

| 层次 | 仿真器 | 模型复杂度 | 结果可信度 |
|------|--------|-----------|-----------|
| 第一层 | Python 运动学模型 | 最简 | 算法正确性验证 |
| 第二层 | Python 动态模型 | 中等 | 参数调优 |
| **第三层** | **CARLA 物理引擎** | **最高** | **接近真实** |

三层对比本身就是工程方法论的体现。

---

## 第二十七至二十八周：CARLA 桥接节点开发

### 27.1 替换 sim_bridge_node.py

删除当前基于 Python 自行车模型的 `sim_bridge_node.py`，新建 `carla_bridge_node.py`：

```python
#!/usr/bin/env python3
"""
CARLA-ROS2 桥接节点
- 从 /vehicle_cmd 话题接收控制指令，发送给 CARLA 车辆
- 从 CARLA 读取车辆状态，发布到 /vehicle_state 话题
"""
import sys
sys.path.append('/opt/carla-simulator/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg')

import carla
import rclpy
from rclpy.node import Node
from av_control_msgs.msg import VehicleCmd, VehicleState

class CarlaBridgeNode(Node):
    def __init__(self):
        super().__init__('carla_bridge_node')

        # 连接 CARLA 服务器
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        # 在 CARLA 中生成车辆
        blueprint = self.world.get_blueprint_library().find('vehicle.tesla.model3')
        spawn_point = self.world.get_map().get_spawn_points()[0]
        self.vehicle = self.world.spawn_actor(blueprint, spawn_point)

        # 订阅控制指令
        self.cmd_sub = self.create_subscription(
            VehicleCmd, '/vehicle_cmd', self.cmd_callback, 10)

        # 发布车辆状态（20Hz）
        self.state_pub = self.create_publisher(VehicleState, '/vehicle_state', 10)
        self.timer = self.create_timer(0.05, self.publish_state)

        self.get_logger().info('CARLA 桥接节点已启动')

    def cmd_callback(self, msg: VehicleCmd):
        """将 ROS2 控制指令转换为 CARLA VehicleControl"""
        control = carla.VehicleControl(
            throttle=float(msg.throttle),
            steer=float(msg.steer),
            brake=float(msg.brake),
        )
        self.vehicle.apply_control(control)

    def publish_state(self):
        """读取 CARLA 车辆状态并发布"""
        transform = self.vehicle.get_transform()
        velocity  = self.vehicle.get_velocity()
        v = (velocity.x**2 + velocity.y**2)**0.5

        msg = VehicleState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x     = transform.location.x
        msg.y     = -transform.location.y      # CARLA Y 轴与 ROS 相反
        msg.theta = -transform.rotation.yaw * 3.14159 / 180.0
        msg.v     = v
        self.state_pub.publish(msg)

    def destroy(self):
        if self.vehicle:
            self.vehicle.destroy()


def main():
    rclpy.init()
    node = CarlaBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()
```

### 27.2 轨迹与 CARLA 坐标系对齐

CARLA 使用左手坐标系（Y 轴朝右），ROS 使用右手坐标系（Y 轴朝左）。

轨迹发布节点需要：
1. 在 CARLA 地图中选取一段合适的直道
2. 将 ISO 3888 双移线轨迹的起点对齐到 CARLA 起始位姿
3. 坐标系转换：ROS `(x, y)` → CARLA `(x, -y)`

---

## 第二十九至三十周：场景录制与数据采集

### 29.1 录制演示 GIF

CARLA 无头模式下通过 `spectator` 相机录制俯视视角：

```python
# 设置跟随相机（俯视角）
spectator = world.get_spectator()
# 在控制循环中更新相机位置，跟随车辆
transform = vehicle.get_transform()
spectator.set_transform(carla.Transform(
    transform.location + carla.Location(z=50),
    carla.Rotation(pitch=-90)
))
```

使用 `ffmpeg` 将截图序列合成 GIF：

```bash
ffmpeg -r 20 -i frame_%04d.png -vf "scale=800:-1" -loop 0 demo.gif
```

### 29.2 采集与记录对比数据

在 CARLA 中对 Pure Pursuit / LQR / MPC 各跑 3 次双移线：
- 记录 RMSE、最大误差、计算耗时
- 与第一层（运动学模型）、第二层（动态模型）结果并排对比

---

## 第三十一至三十二周：三层对比图表与项目整理

### 31.1 三层对比总图

最终核心图表：**三层仿真 × 四种控制器 × 速度** 的完整对比矩阵。

```
图表标题：Controller Benchmark Across Simulation Fidelity

子图1：运动学模型（第一层）速度-RMSE
子图2：动态模型（第二层）速度-RMSE
子图3：CARLA（第三层）速度-RMSE

结论：随仿真精度提升，控制器差距逐渐拉开
```

### 31.2 GitHub 最终发布

```
README.md 包含：
- 演示 GIF（CARLA 中车辆跟踪轨迹）
- 三层对比图
- 五控制器 RMSE 数据表（含计算耗时）
- 系统架构图
- 安装与运行说明
```

---

## 本阶段完成标准

| 产出 | 验证标准 |
|------|----------|
| CARLA 桥接节点 | `/vehicle_state` 话题 20Hz，数据正常 |
| 全系统联调 | CARLA 中车辆跟踪双移线，目视正常 |
| 演示 GIF | < 10MB，清晰展示 MPC 跟踪效果 |
| 三层对比图 | 三层仿真数据完整，结论清晰 |
| GitHub README | 公开仓库，结构完整 |

---

## 阶段结论

项目采用三层递进的仿真验证方式：运动学模型快速验证算法逻辑，动态模型调优参数并体现控制器差异，最终接入 CARLA 物理引擎验证真实性。三层仿真精度递增，控制器差距随之扩大。这个框架与工业界"纯软件仿真 → 半实物仿真 → 实车"的验证流程在逻辑上是一致的。
