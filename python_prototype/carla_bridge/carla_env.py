import carla
import numpy as np


class CarlaEnv:
    """CARLA 环境桥接：连接、生成车辆、读状态、发控制指令"""

    MAX_STEER_ANGLE = np.radians(70)  # Tesla Model 3 最大方向盘转角

    def __init__(self, host='172.22.0.1', port=2000, town='Town03'):
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.load_world(town)

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)

        self.vehicle   = None
        self.history_x = []
        self.history_y = []

    def spawn_vehicle(self, spawn_index=0):
        bp = self.world.get_blueprint_library().find('vehicle.tesla.model3')
        spawn_point = self.world.get_map().get_spawn_points()[spawn_index]
        self.vehicle = self.world.spawn_actor(bp, spawn_point)
        self.vehicle.set_autopilot(False)
        for _ in range(10):
            self.world.tick()
        return spawn_point

    def get_state(self):
        """返回控制器需要的状态量 [x, y, psi, v]"""
        tf  = self.vehicle.get_transform()
        vel = self.vehicle.get_velocity()

        x   =  tf.location.x
        y   = -tf.location.y              # CARLA Y轴朝南取反，转为标准朝北坐标系
        psi = -np.radians(tf.rotation.yaw)
        v   = np.sqrt(vel.x**2 + vel.y**2)

        self.history_x.append(x)
        self.history_y.append(y)
        return np.array([x, y, psi, v])

    def apply_control(self, delta, accel):
        """
        delta: 前轮转角 (rad)，正值向左
        accel: 加速度 (m/s²)，正值加速，负值制动
        """
        # CARLA steer=+1 为右转，Pure Pursuit delta>0 为左转，需取反
        steer = float(np.clip(-delta / self.MAX_STEER_ANGLE, -1.0, 1.0))

        ctrl = carla.VehicleControl()
        ctrl.steer = steer
        if accel >= 0:
            ctrl.throttle = float(np.clip(accel / 3.0, 0.0, 1.0))
            ctrl.brake    = 0.0
        else:
            ctrl.throttle = 0.0
            ctrl.brake    = float(np.clip(-accel / 5.0, 0.0, 1.0))

        self.vehicle.apply_control(ctrl)
        self.world.tick()

    def destroy(self):
        if self.vehicle:
            self.vehicle.destroy()
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)
