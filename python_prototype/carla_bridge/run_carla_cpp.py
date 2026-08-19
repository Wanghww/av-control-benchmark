import sys
import os
import subprocess
import carla
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_bridge.carla_env import CarlaEnv
from carla_bridge.waypoint_trajectory import get_waypoint_trajectory

CARLA_INTERFACE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', 'cpp_controllers', 'build', 'carla_interface'
)
TRAJ_CSV = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    'trajectory.csv'
)


def save_trajectory(trajectory, path):
    with open(path, 'w') as f:
        f.write('x,y,yaw,v_ref\n')
        for row in trajectory:
            f.write(f'{row[0]},{row[1]},{row[2]},{row[3]}\n')


def run():
    env = CarlaEnv(host='172.22.0.1', port=2000, town='Town03')

    try:
        print('Spawning vehicle...')
        spawn_tf = env.spawn_vehicle(spawn_index=0)

        print('Generating trajectory...')
        trajectory = get_waypoint_trajectory(env.world, spawn_tf, length=150, speed=3.0)
        save_trajectory(trajectory, TRAJ_CSV)

        print('Starting C++ controller process...')
        cpp_proc = subprocess.Popen(
            [CARLA_INTERFACE, TRAJ_CSV],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1
        )

        spectator   = env.world.get_spectator()
        lateral_errors = []
        nearest_idx = 0
        print('Simulation running...')

        for step in range(600):
            state = env.get_state()

            # 发送状态给 C++（格式：x y theta v）
            cpp_proc.stdin.write(f'{state[0]} {state[1]} {state[2]} {state[3]}\n')
            cpp_proc.stdin.flush()

            # 读取控制指令（格式：throttle steer brake）
            line = cpp_proc.stdout.readline().strip()
            if not line:
                break
            throttle, steer, brake = map(float, line.split())

            # 直接发送给 CARLA
            ctrl = carla.VehicleControl()
            ctrl.throttle = throttle
            ctrl.steer    = steer
            ctrl.brake    = brake
            env.vehicle.apply_control(ctrl)
            env.world.tick()

            # 记录误差
            dists = np.linalg.norm(trajectory[:, :2] - state[:2], axis=1)
            nearest_idx = int(np.argmin(dists[nearest_idx:]) + nearest_idx)
            lateral_errors.append(dists[nearest_idx])

            # 视角跟随
            tf = env.vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                tf.location + carla.Location(z=30),
                carla.Rotation(pitch=-90)
            ))

            if step % 50 == 0:
                print(f'  Step {step:3d}: v={state[3]:.1f} m/s  err={lateral_errors[-1]:.3f} m')

            if nearest_idx >= len(trajectory) - 10:
                print(f'  Reached end at step {step}.')
                break

        cpp_proc.stdin.close()
        cpp_proc.wait()

    finally:
        env.destroy()

    rmse = np.sqrt(np.mean(np.array(lateral_errors) ** 2))
    print(f'\nRMSE: {rmse:.4f} m')
    print(f'Max error: {max(lateral_errors):.4f} m')

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    axes[0].plot(trajectory[:, 0], trajectory[:, 1], 'b--', lw=2, label='Reference')
    axes[0].plot(env.history_x, env.history_y, 'r-', lw=1.5, label='Actual (C++)')
    axes[0].set_xlabel('X (m)')
    axes[0].set_ylabel('Y (m)')
    axes[0].set_title('CARLA Tracking with C++ Controller')
    axes[0].legend()
    axes[0].set_aspect('equal')
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(lateral_errors, 'g-', lw=1.2)
    axes[1].axhline(rmse, color='r', linestyle='--', label=f'RMSE: {rmse:.3f} m')
    axes[1].set_xlabel('Step')
    axes[1].set_ylabel('Lateral Error (m)')
    axes[1].set_title('Lateral Tracking Error (C++ Controller)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    os.makedirs(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'results'), exist_ok=True)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'results', 'carla_cpp_pure_pursuit.png')
    plt.savefig(out, dpi=150)
    print(f'Plot saved to {out}')


if __name__ == '__main__':
    run()
