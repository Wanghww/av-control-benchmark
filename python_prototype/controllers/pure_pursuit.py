import numpy as np


class PurePursuitController:
    """横向 Pure Pursuit 路径跟踪控制器"""

    def __init__(self, wheelbase=2.7, k=0.1, ld_min=2.0):
        self.L      = wheelbase
        self.k      = k        # 前视距离增益：速度越快看得越远
        self.ld_min = ld_min   # 最小前视距离 (m)

    def compute(self, state, trajectory, min_idx=0):
        """
        state:      [x, y, theta, v]
        trajectory: N×4 数组，每行 [x, y, yaw, v_ref]
        min_idx:    最小搜索起点，防止最近点向后跳
        返回:       (转角 delta rad, 目标速度 v_ref, nearest_idx)
        """
        x, y, theta, v = state

        ld = max(self.k * v + self.ld_min, self.ld_min)

        # 只在 min_idx 之后搜索最近点，防止轨迹弯曲时向后跳
        search_start = max(min_idx, 0)
        dists = np.linalg.norm(trajectory[search_start:, :2] - np.array([x, y]), axis=1)
        nearest_idx = search_start + int(np.argmin(dists))

        # 找前视点（距离 >= ld 的第一个点）
        target_idx = nearest_idx
        for i in range(nearest_idx, len(trajectory)):
            if np.linalg.norm(trajectory[i, :2] - np.array([x, y])) >= ld:
                target_idx = i
                break

        tx, ty = trajectory[target_idx, :2]
        alpha = np.arctan2(ty - y, tx - x) - theta
        alpha = self._normalize_angle(alpha)

        delta = np.arctan2(2 * self.L * np.sin(alpha), ld)
        v_ref = trajectory[target_idx, 3]

        return delta, v_ref, nearest_idx

    @staticmethod
    def _normalize_angle(angle):
        while angle >  np.pi: angle -= 2 * np.pi
        while angle < -np.pi: angle += 2 * np.pi
        return angle
