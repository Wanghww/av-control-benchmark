class PIDController:
    """纵向速度 PID 控制器"""

    def __init__(self, kp=1.0, ki=0.1, kd=0.05, dt=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self._error_sum  = 0.0
        self._last_error = 0.0

    def compute(self, v_ref, v_current):
        """
        v_ref:     目标速度 (m/s)
        v_current: 当前速度 (m/s)
        返回:      加速度指令 (m/s²)
        """
        error = v_ref - v_current

        self._error_sum += error * self.dt
        error_diff = (error - self._last_error) / self.dt

        output = self.kp * error + self.ki * self._error_sum + self.kd * error_diff

        self._last_error = error
        return output

    def reset(self):
        self._error_sum  = 0.0
        self._last_error = 0.0
