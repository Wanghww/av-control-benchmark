#!/usr/bin/env python3
"""
CARLA-ROS2 桥接节点（第七阶段）
替换 sim_bridge_node.py，接入真实 CARLA 物理引擎

功能：
  - 连接 CARLA 服务器，生成 Tesla Model 3
  - 以起始位姿为原点，发布 /vehicle_state（本地坐标系，20Hz）
  - 接收 /vehicle_cmd，转换后发给 CARLA 执行器
  - 启动 1s 后发布双移线参考轨迹到 /trajectory

坐标系处理：
  CARLA 使用左手坐标系（Y 轴向右，yaw 顺时针为正）
  ROS 使用右手坐标系（Y 轴向左，yaw 逆时针为正）
  转换：ros_y = -carla_y，ros_yaw = -carla_yaw_deg * pi/180

运行前提：
  - CARLA 0.9.15 已在 Windows 启动（CarlaUE4.exe）
  - WSL2 可通过 localhost:2000 访问 CARLA
"""

import sys
import math
import numpy as np

# CARLA Python API（根据实际安装路径调整）
_CARLA_EGG = (
    '/opt/carla-simulator/PythonAPI/carla/dist/'
    'carla-0.9.15-py3.10-linux-x86_64.egg'
)
if _CARLA_EGG not in sys.path:
    sys.path.append(_CARLA_EGG)

try:
    import carla
except ImportError:
    print('[ERROR] 找不到 CARLA Python API，请确认路径：', _CARLA_EGG)
    sys.exit(1)

import rclpy
from rclpy.node import Node
from av_control_msgs.msg import VehicleCmd, VehicleState, Trajectory


class CarlaBridgeNode(Node):
    # Tesla Model 3 最大方向盘转角（约 70°）
    MAX_STEER_RAD = math.radians(70.0)

    def __init__(self):
        super().__init__('carla_bridge_node')

        # ── 参数声明 ──────────────────────────────────────────────
        self.declare_parameter('carla_host',   '172.22.0.1')
        self.declare_parameter('carla_port',   2000)
        self.declare_parameter('spawn_index',  0)
        self.declare_parameter('target_speed', 8.0)
        self.declare_parameter('town',         'Town03')

        host        = self.get_parameter('carla_host').get_parameter_value().string_value
        port        = self.get_parameter('carla_port').get_parameter_value().integer_value
        spawn_idx   = self.get_parameter('spawn_index').get_parameter_value().integer_value
        self.speed_ = self.get_parameter('target_speed').get_parameter_value().double_value
        town        = self.get_parameter('town').get_parameter_value().string_value

        # ── 连接 CARLA ────────────────────────────────────────────
        self.get_logger().info(f'正在连接 CARLA {host}:{port} ...')
        self.client_ = carla.Client(host, port)
        self.client_.set_timeout(60.0)   # load_world 需要较长超时
        self.get_logger().info(f'加载地图 {town}（首次可能需要 30-60s）...')
        self.world_  = self.client_.load_world(town)
        self.client_.set_timeout(15.0)   # 恢复正常超时
        self.get_logger().info(f'当前地图：{self.world_.get_map().name}')

        # 重置为异步模式（防止上次异常退出留下同步模式，导致 CARLA 卡死）
        settings = self.world_.get_settings()
        settings.synchronous_mode    = False
        settings.fixed_delta_seconds = 0.0
        self.world_.apply_settings(settings)
        self.get_logger().info('CARLA 异步模式（ROS2 timer 驱动 20Hz 控制循环）')

        # ── 清理残留车辆（上次异常退出可能留下车辆）────────────────
        old_actors = self.world_.get_actors().filter('vehicle.*')
        if old_actors:
            self.get_logger().info(f'清理 {len(old_actors)} 个残留车辆')
            for a in old_actors:
                a.destroy()

        # ── 生成车辆 ──────────────────────────────────────────────
        bp = self.world_.get_blueprint_library().find('vehicle.tesla.model3')
        spawn_points = self.world_.get_map().get_spawn_points()
        if spawn_idx >= len(spawn_points):
            spawn_idx = 0
            self.get_logger().warn(f'spawn_index 超出范围，已重置为 0')
        spawn_tf = spawn_points[spawn_idx]

        self.vehicle_ = self.world_.spawn_actor(bp, spawn_tf)
        # 新生成车辆默认手动驾驶，不需要 set_autopilot(False)

        # 起始位姿初始化为 None，在物理稳定后（2s 定时器）再采样
        self._ox   = None
        self._oy   = None
        self._oyaw = None

        # ── ROS2 话题 ─────────────────────────────────────────────
        self.state_pub_ = self.create_publisher(VehicleState, '/vehicle_state', 10)
        self.traj_pub_  = self.create_publisher(Trajectory,    '/trajectory',    10)

        self.cmd_sub_ = self.create_subscription(
            VehicleCmd, '/vehicle_cmd', self._cmd_callback, 10)

        # 20Hz 控制循环（起始位姿未采样前跳过）
        self.timer_ = self.create_timer(0.05, self._control_step)

        # 2s 后采样起始位姿并发布轨迹（给 CARLA 物理引擎充分稳定时间）
        self._traj_sent = False
        self.create_timer(2.0, self._init_and_publish)

        self.get_logger().info('CARLA 桥接节点已启动，2s 后采样基准位姿...')

    # ── 2s 后：采样基准位姿 + 发布轨迹 ──────────────────────────────────
    def _init_and_publish(self):
        if self._traj_sent:
            return

        # 物理引擎已稳定，现在采样真实起始位姿
        init_tf = self.vehicle_.get_transform()
        self._ox   = init_tf.location.x
        self._oy   = init_tf.location.y
        self._oyaw = -math.radians(init_tf.rotation.yaw)

        self.get_logger().info(
            f'基准位姿已采样：CARLA ({self._ox:.1f}, {self._oy:.1f}) '
            f'yaw_ros={math.degrees(self._oyaw):.1f}°'
        )

        msg = self._make_dlc_trajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.traj_pub_.publish(msg)
        self._traj_sent = True
        self.get_logger().info(f'轨迹已发布：{len(msg.x)} 个路点，速度 {self.speed_} m/s')

    def _make_dlc_trajectory(self) -> Trajectory:
        """直线轨迹：车辆沿本地 +X 方向直行 200m
        对 CARLA 演示 GIF 来说已足够——只需展示车辆在真实物理引擎中受控行驶
        量化对比数据已在第一、二层仿真中完成
        """
        msg = Trajectory()
        for i in range(400):   # 0~199.5m，步长 0.5m
            msg.x.append(float(i * 0.5))
            msg.y.append(0.0)
            msg.yaw.append(0.0)
            msg.v_ref.append(float(self.speed_))
        self.get_logger().info(f'直线轨迹已生成（400 点，200m，速度 {self.speed_} m/s）')
        return msg

    # ── 每帧步进（异步模式，CARLA 自动运行，不需要 tick）────────────────
    def _control_step(self):
        if self._ox is None:   # 基准位姿未采样，跳过
            return
        self._update_spectator()
        self._publish_state()

    def _update_spectator(self):
        """让 CARLA 俯视视角跟随车辆（俯角 70°，高度 30m）"""
        tf = self.vehicle_.get_transform()
        spectator = self.world_.get_spectator()
        spectator.set_transform(carla.Transform(
            tf.location + carla.Location(z=30),
            carla.Rotation(pitch=-70, yaw=tf.rotation.yaw)
        ))

    # ── 接收控制指令 ──────────────────────────────────────────────────
    def _cmd_callback(self, msg: VehicleCmd):
        steer = float(np.clip(msg.delta / self.MAX_STEER_RAD, -1.0, 1.0))

        ctrl = carla.VehicleControl()
        ctrl.steer = steer
        if msg.accel >= 0.0:
            ctrl.throttle = float(np.clip(msg.accel / 3.0, 0.0, 1.0))
            ctrl.brake    = 0.0
        else:
            ctrl.throttle = 0.0
            ctrl.brake    = float(np.clip(-msg.accel / 5.0, 0.0, 1.0))

        self.vehicle_.apply_control(ctrl)

    # ── 发布车辆状态 ──────────────────────────────────────────────────
    def _publish_state(self):
        tf  = self.vehicle_.get_transform()
        vel = self.vehicle_.get_velocity()

        # CARLA 世界坐标 → ROS 本地坐标系
        cx   = tf.location.x
        cy   = tf.location.y
        cyaw = -math.radians(tf.rotation.yaw)   # CARLA顺时针→ROS逆时针

        # 相对于起始位姿的偏移（Y 轴取反）
        dx  = cx - self._ox
        dy  = -(cy - self._oy)
        dth = cyaw - self._oyaw

        # 旋转到起始航向对齐的局部坐标系
        c = math.cos(-self._oyaw)
        s = math.sin(-self._oyaw)
        lx =  c * dx - s * dy
        ly =  s * dx + c * dy

        # 角度归一化到 (-π, π]
        while dth >  math.pi: dth -= 2 * math.pi
        while dth < -math.pi: dth += 2 * math.pi

        v = math.sqrt(vel.x**2 + vel.y**2)

        # 每 2 秒打印一次诊断信息
        if not hasattr(self, '_log_count'): self._log_count = 0
        self._log_count += 1
        if self._log_count % 40 == 0:
            self.get_logger().info(
                f'state: x={lx:.2f} y={ly:.3f} θ={math.degrees(dth):.2f}° v={v:.2f}m/s')

        out = VehicleState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.x     = lx
        out.y     = ly
        out.theta = dth
        out.v     = v
        self.state_pub_.publish(out)

    # ── 清理 ──────────────────────────────────────────────────────────
    def destroy(self):
        if hasattr(self, 'vehicle_') and self.vehicle_:
            self.vehicle_.destroy()
            self.get_logger().info('CARLA 车辆已销毁')
        self.get_logger().info('CARLA 资源已释放')


def main():
    rclpy.init()
    node = CarlaBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
