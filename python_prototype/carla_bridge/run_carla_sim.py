import sys
import os
import carla
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from controllers.pid_controller import PIDController
from controllers.pure_pursuit import PurePursuitController
from carla_bridge.carla_env import CarlaEnv
from carla_bridge.waypoint_trajectory import get_waypoint_trajectory


def run():
    env = CarlaEnv(host='172.22.0.1', port=2000, town='Town03')

    try:
        print('Spawning vehicle...')
        spawn_tf = env.spawn_vehicle(spawn_index=0)

        print('Generating reference trajectory...')
        trajectory = get_waypoint_trajectory(env.world, spawn_tf, length=150, speed=3.0)

        pid          = PIDController(kp=1.5, ki=0.1, kd=0.05)
        pure_pursuit = PurePursuitController(wheelbase=2.87, k=0.3, ld_min=3.0)  # Model 3 轴距

        lateral_errors = []
        print('Starting simulation...')

        spectator = env.world.get_spectator()
        nearest_idx = 0
        for step in range(600):  # 30 秒（0.05s × 600）
            state        = env.get_state()
            delta, v_ref, nearest_idx = pure_pursuit.compute(state, trajectory, min_idx=nearest_idx)
            accel        = pid.compute(v_ref, state[3])
            env.apply_control(delta, accel)

            # 视角跟随车辆（俯视 30m 高）
            tf = env.vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                tf.location + carla.Location(z=30),
                carla.Rotation(pitch=-90)
            ))

            dists = np.linalg.norm(trajectory[:, :2] - state[:2], axis=1)
            lateral_errors.append(dists[nearest_idx])

            if step % 50 == 0:
                print(f'  Step {step:3d}: v={state[3]:.1f} m/s  lateral_err={lateral_errors[-1]:.3f} m')

            if nearest_idx >= len(trajectory) - 10:
                print(f'  Reached end of trajectory at step {step}.')
                break

    finally:
        env.destroy()

    rmse = np.sqrt(np.mean(np.array(lateral_errors) ** 2))
    print(f'\nRMSE: {rmse:.4f} m')
    print(f'Max error: {max(lateral_errors):.4f} m')

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    axes[0].plot(trajectory[:, 0], trajectory[:, 1], 'b--', lw=2, label='Reference')
    axes[0].plot(env.history_x, env.history_y, 'r-', lw=1.5, label='Actual')
    axes[0].set_xlabel('X (m)')
    axes[0].set_ylabel('Y (m)')
    axes[0].set_title('CARLA Trajectory Tracking (Town03)')
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
    out_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results', 'carla_pure_pursuit.png')
    plt.savefig(out_path, dpi=150)
    print(f'Plot saved to {out_path}')


if __name__ == '__main__':
    run()
