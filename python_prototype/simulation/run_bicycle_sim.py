import sys
import os
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from models.dynamic_bicycle_model import DynamicBicycleModel
from controllers.pid_controller import PIDController
from controllers.pure_pursuit import PurePursuitController
from utils.trajectory_generator import generate_s_curve


def run():
    trajectory = generate_s_curve(length=100, amplitude=10, speed=6.0)

    car          = DynamicBicycleModel(dt=0.05)
    pid          = PIDController(kp=1.5, ki=0.1, kd=0.05, dt=0.05)
    pure_pursuit = PurePursuitController(wheelbase=2.7, k=0.3, ld_min=3.0)

    car.set_state(trajectory[0, 0], trajectory[0, 1], trajectory[0, 2], 0.0)

    history_x      = []
    history_y      = []
    lateral_errors = []

    nearest_idx = 0
    for _ in range(1000):
        state = car.get_state()
        delta, v_ref, nearest_idx = pure_pursuit.compute(state, trajectory, min_idx=nearest_idx)
        accel = pid.compute(v_ref, state[3])
        car.update(delta, accel)

        history_x.append(car.x)
        history_y.append(car.y)

        dists = np.linalg.norm(trajectory[:, :2] - np.array([car.x, car.y]), axis=1)
        lateral_errors.append(dists[nearest_idx])

        if nearest_idx >= len(trajectory) - 10:
            break

    rmse = np.sqrt(np.mean(np.array(lateral_errors) ** 2))
    print(f'RMSE: {rmse:.4f} m')
    print(f'Max error: {max(lateral_errors):.4f} m')

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    axes[0].plot(trajectory[:, 0], trajectory[:, 1], 'b--', lw=2, label='Reference')
    axes[0].plot(history_x, history_y, 'r-', lw=1.5, label='Actual')
    axes[0].set_xlabel('X (m)')
    axes[0].set_ylabel('Y (m)')
    axes[0].set_title('S-Curve Tracking (Dynamic Bicycle Model)')
    axes[0].legend()
    axes[0].set_aspect('equal')
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(lateral_errors, 'g-', lw=1.2)
    axes[1].axhline(rmse, color='r', linestyle='--', label=f'RMSE: {rmse:.3f} m')
    axes[1].set_xlabel('Step')
    axes[1].set_ylabel('Lateral Error (m)')
    axes[1].set_title('Lateral Tracking Error')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    os.makedirs(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results'), exist_ok=True)
    out_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results', 'bicycle_pure_pursuit.png')
    plt.savefig(out_path, dpi=150)
    print(f'Plot saved to {out_path}')


if __name__ == '__main__':
    run()
