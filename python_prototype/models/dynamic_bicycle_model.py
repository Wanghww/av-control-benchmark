import numpy as np


class DynamicBicycleModel:
    """
    动态自行车模型（线性轮胎模型）
    状态量: [x, y, psi, v]
    内部状态: beta（质心侧偏角）, r（横摆角速度）
    控制量: [delta（前轮转角 rad）, a（加速度 m/s²）]
    """

    def __init__(self, dt=0.05):
        self.dt = dt

        # 车辆参数（参考普通轿车）
        self.m  = 1500.0      # 整备质量 (kg)
        self.Iz = 2500.0      # 绕 z 轴转动惯量 (kg·m²)
        self.lf = 1.2         # 质心到前轴距离 (m)
        self.lr = 1.5         # 质心到后轴距离 (m)
        self.L  = self.lf + self.lr

        # 轮胎参数
        self.Cf = 80000.0     # 前轮侧偏刚度 (N/rad)
        self.Cr = 80000.0     # 后轮侧偏刚度 (N/rad)

        # 约束
        self.max_steer = np.radians(30)
        self.max_speed = 20.0
        self.max_accel = 3.0

        # 状态
        self.x    = 0.0
        self.y    = 0.0
        self.psi  = 0.0
        self.v    = 0.0
        self.beta = 0.0
        self.r    = 0.0

    def _derivatives(self, state, delta, v):
        """计算状态导数，供 RK4 使用。state = [x, y, psi, beta, r]"""
        x, y, psi, beta, r = state
        m, Iz, lf, lr = self.m, self.Iz, self.lf, self.lr
        Cf, Cr = self.Cf, self.Cr

        dx    = v * np.cos(psi + beta)
        dy    = v * np.sin(psi + beta)
        dpsi  = r
        dbeta = (-(Cf + Cr) / (m * v) * beta
                + (-(Cf * lf - Cr * lr) / (m * v**2) - 1.0) * r
                + Cf / (m * v) * delta)
        dr    = (-(Cf * lf - Cr * lr) / Iz * beta
                - (Cf * lf**2 + Cr * lr**2) / (Iz * v) * r
                + Cf * lf / Iz * delta)

        return np.array([dx, dy, dpsi, dbeta, dr])

    def update(self, delta, a):
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        a     = np.clip(a,    -self.max_accel,  self.max_accel)

        # 低速用运动学近似，避免 v≈0 时数值问题
        if self.v < 0.5:
            self.x   += self.v * np.cos(self.psi) * self.dt
            self.y   += self.v * np.sin(self.psi) * self.dt
            self.psi += self.v / self.L * np.tan(delta) * self.dt
            self.psi  = self._normalize_angle(self.psi)
            self.v    = np.clip(self.v + a * self.dt, 0.0, self.max_speed)
            return

        # RK4 积分（v 在一步内视为常数，标准工程近似）
        state = np.array([self.x, self.y, self.psi, self.beta, self.r])
        v = self.v
        dt = self.dt

        k1 = self._derivatives(state,              delta, v)
        k2 = self._derivatives(state + dt/2 * k1,  delta, v)
        k3 = self._derivatives(state + dt/2 * k2,  delta, v)
        k4 = self._derivatives(state + dt    * k3,  delta, v)

        state_new = state + dt / 6 * (k1 + 2*k2 + 2*k3 + k4)

        self.x, self.y, self.psi, self.beta, self.r = state_new
        self.psi = self._normalize_angle(self.psi)
        self.v   = np.clip(self.v + a * self.dt, 0.0, self.max_speed)

    def get_state(self):
        return np.array([self.x, self.y, self.psi, self.v])

    def set_state(self, x, y, psi, v):
        self.x, self.y, self.psi, self.v = x, y, psi, v
        self.beta = 0.0
        self.r    = 0.0

    @staticmethod
    def _normalize_angle(angle):
        while angle >  np.pi: angle -= 2 * np.pi
        while angle < -np.pi: angle += 2 * np.pi
        return angle
