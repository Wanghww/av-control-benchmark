import numpy as np
import matplotlib.pyplot as plt
import os

csv_path = os.path.join(os.path.dirname(__file__), '..', 'build', 'results', 'simulation_output.csv')
data = np.loadtxt(csv_path, delimiter=',', skiprows=1)

steps         = data[:, 0]
history_x     = data[:, 1]
history_y     = data[:, 2]
lateral_errors = data[:, 5]

# 生成对应的 S 形参考轨迹（与 main.cpp 一致）
N, length = 300, 100.0
ref_x = np.array([length * i / N for i in range(N)])
ref_y = 10.0 * np.sin(2 * np.pi * ref_x / length)

rmse = np.sqrt(np.mean(lateral_errors ** 2))
print(f'RMSE: {rmse:.4f} m')
print(f'Max error: {np.max(lateral_errors):.4f} m')

fig, axes = plt.subplots(1, 2, figsize=(14, 5))

axes[0].plot(ref_x, ref_y, 'b--', lw=2, label='Reference')
axes[0].plot(history_x, history_y, 'r-', lw=1.5, label='Actual (C++)')
axes[0].set_xlabel('X (m)')
axes[0].set_ylabel('Y (m)')
axes[0].set_title('S-Curve Tracking (C++ Kinematic Model)')
axes[0].legend()
axes[0].set_aspect('equal')
axes[0].grid(True, alpha=0.3)

axes[1].plot(steps, lateral_errors, 'g-', lw=1.2)
axes[1].axhline(rmse, color='r', linestyle='--', label=f'RMSE: {rmse:.3f} m')
axes[1].set_xlabel('Step')
axes[1].set_ylabel('Lateral Error (m)')
axes[1].set_title('Lateral Tracking Error')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

plt.tight_layout()
out_path = os.path.join(os.path.dirname(__file__), '..', 'build', 'results', 'cpp_pure_pursuit.png')
plt.savefig(out_path, dpi=150)
print(f'Plot saved to {out_path}')
