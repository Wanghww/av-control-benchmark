"""
多控制器基准对比可视化
读取 build/results/ 下的 CSV 文件，生成四张对比图：
  1. S 弯轨迹跟踪对比
  2. 双移线轨迹跟踪对比
  3. 两场景 RMSE 柱状图汇总
  4. 速度-RMSE 扫描曲线（核心图）
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import os

# ---- 路径配置 ----
RESULTS_DIR = os.path.join(os.path.dirname(__file__), '..', 'build', 'results')
OUTPUT_DIR  = os.path.join(os.path.dirname(__file__), '..', 'build', 'results')

COLORS = {
    'Pure Pursuit': '#E74C3C',
    'Stanley':      '#E67E22',
    'LQR':          '#27AE60',
    'MPC':          '#2980B9',
    'Reference':    '#7F8C8D',
}

def load_csv(filename):
    path = os.path.join(RESULTS_DIR, filename)
    if not os.path.exists(path):
        return None
    return np.loadtxt(path, delimiter=',', skiprows=1)

def compute_rmse(data):
    return np.sqrt(np.mean(data[:, 5] ** 2))

# ---- 生成参考轨迹 ----
def ref_s_curve(N=300, length=100.0):
    x = np.array([length * i / N for i in range(N)])
    y = 10.0 * np.sin(2 * np.pi * x / length)
    return x, y

def ref_dlc(N=600, total=130.0, offset=3.5, L=20.0):
    x = np.linspace(0, total, N)
    y = np.zeros(N)
    for i, xi in enumerate(x):
        if 30 <= xi < 50:
            s = xi - 30
            y[i] = offset / 2 * (1 - np.cos(np.pi * s / L))
        elif 50 <= xi < 80:
            y[i] = offset
        elif 80 <= xi < 100:
            s = xi - 80
            y[i] = offset / 2 * (1 + np.cos(np.pi * s / L))
    return x, y

# ========== 图1：S 弯轨迹对比 ==========
fig1, axes = plt.subplots(1, 2, figsize=(14, 5))
fig1.suptitle('S-Curve Tracking (6 m/s)', fontsize=13, fontweight='bold')

ref_x, ref_y = ref_s_curve()
axes[0].plot(ref_x, ref_y, '--', color=COLORS['Reference'], lw=2, label='Reference', zorder=5)

s_curve_files = {
    'Pure Pursuit': 'simulation_output.csv',
    'LQR':          'lqr_output.csv',
    'MPC':          'mpc_output.csv',
}
s_rmse = {}
for name, fname in s_curve_files.items():
    data = load_csv(fname)
    if data is None:
        continue
    axes[0].plot(data[:, 1], data[:, 2], '-', color=COLORS[name], lw=1.5, label=name, alpha=0.85)
    s_rmse[name] = compute_rmse(data)
    axes[1].plot(data[:, 0], data[:, 5], '-', color=COLORS[name], lw=1.2, label=f'{name} ({s_rmse[name]:.4f} m)')

axes[0].set_xlabel('X (m)'); axes[0].set_ylabel('Y (m)')
axes[0].set_title('Trajectory'); axes[0].legend(fontsize=9); axes[0].set_aspect('equal'); axes[0].grid(True, alpha=0.3)
axes[1].set_xlabel('Step'); axes[1].set_ylabel('Lateral Error (m)')
axes[1].set_title('Lateral Error'); axes[1].legend(fontsize=9); axes[1].grid(True, alpha=0.3)

plt.tight_layout()
out1 = os.path.join(OUTPUT_DIR, 'benchmark_s_curve.png')
plt.savefig(out1, dpi=150)
print(f'Saved: {out1}')

# ========== 图2：双移线轨迹对比 ==========
fig2, axes = plt.subplots(1, 2, figsize=(14, 5))
fig2.suptitle('ISO 3888 Double Lane Change (8 m/s)', fontsize=13, fontweight='bold')

ref_x, ref_y = ref_dlc()
axes[0].plot(ref_x, ref_y, '--', color=COLORS['Reference'], lw=2, label='Reference', zorder=5)

dlc_files = {
    'Pure Pursuit': 'dlc_pp_output.csv',
    'Stanley':      'dlc_stanley_output.csv',
    'LQR':          'dlc_lqr_output.csv',
    'MPC':          'dlc_mpc_output.csv',
}
dlc_rmse = {}
for name, fname in dlc_files.items():
    data = load_csv(fname)
    if data is None:
        continue
    rmse = compute_rmse(data)
    dlc_rmse[name] = rmse
    # Stanley 发散，轨迹太乱只画误差曲线不画轨迹
    if name != 'Stanley':
        axes[0].plot(data[:, 1], data[:, 2], '-', color=COLORS[name], lw=1.5, label=name, alpha=0.85)
    axes[1].plot(data[:, 0], data[:, 5], '-', color=COLORS[name], lw=1.2,
                 label=f'{name} ({rmse:.4f} m)', alpha=0.9)

axes[0].set_xlabel('X (m)'); axes[0].set_ylabel('Y (m)')
axes[0].set_title('Trajectory (Stanley excluded — diverged)')
axes[0].legend(fontsize=9); axes[0].set_aspect('equal'); axes[0].grid(True, alpha=0.3)
axes[1].set_xlabel('Step'); axes[1].set_ylabel('Lateral Error (m)')
axes[1].set_title('Lateral Error'); axes[1].legend(fontsize=9); axes[1].grid(True, alpha=0.3)
axes[1].set_ylim(0, min(axes[1].get_ylim()[1], 0.5))  # 截断 Stanley 发散峰值，便于观察

plt.tight_layout()
out2 = os.path.join(OUTPUT_DIR, 'benchmark_dlc.png')
plt.savefig(out2, dpi=150)
print(f'Saved: {out2}')

# ========== 图3：RMSE 汇总柱状图 ==========
fig3, axes = plt.subplots(1, 2, figsize=(12, 5))
fig3.suptitle('RMSE Comparison', fontsize=13, fontweight='bold')

# S 弯
names_s = list(s_rmse.keys())
vals_s  = [s_rmse[n] for n in names_s]
bars = axes[0].bar(names_s, vals_s, color=[COLORS[n] for n in names_s], width=0.5, edgecolor='white')
for bar, v in zip(bars, vals_s):
    axes[0].text(bar.get_x() + bar.get_width()/2, v + 0.0005, f'{v:.4f}', ha='center', va='bottom', fontsize=9)
axes[0].set_title('S-Curve (6 m/s)'); axes[0].set_ylabel('RMSE (m)'); axes[0].grid(axis='y', alpha=0.3)
axes[0].set_ylim(0, max(vals_s) * 1.3)

# 双移线（排除 Stanley 方便纵轴可读）
names_dlc_plot = [n for n in dlc_rmse if n != 'Stanley']
vals_dlc = [dlc_rmse[n] for n in names_dlc_plot]
bars2 = axes[1].bar(names_dlc_plot, vals_dlc, color=[COLORS[n] for n in names_dlc_plot], width=0.5, edgecolor='white')
for bar, v in zip(bars2, vals_dlc):
    axes[1].text(bar.get_x() + bar.get_width()/2, v + 0.0005, f'{v:.4f}', ha='center', va='bottom', fontsize=9)
axes[1].set_title('Double Lane Change (8 m/s)\n(Stanley diverged, excluded)')
axes[1].set_ylabel('RMSE (m)'); axes[1].grid(axis='y', alpha=0.3)
axes[1].set_ylim(0, max(vals_dlc) * 1.3)

plt.tight_layout()
out3 = os.path.join(OUTPUT_DIR, 'benchmark_rmse_summary.png')
plt.savefig(out3, dpi=150)
print(f'Saved: {out3}')

# ---- 打印汇总表 ----
print('\n===== RMSE 汇总 =====')
print(f'{"控制器":<14} {"S弯 (m)":<12} {"双移线 (m)":<12}')
print('-' * 40)
all_controllers = ['Pure Pursuit', 'Stanley', 'LQR', 'MPC']
for name in all_controllers:
    s = f'{s_rmse[name]:.4f}' if name in s_rmse else '-'
    d = f'{dlc_rmse[name]:.4f}' if name in dlc_rmse else '-'
    print(f'{name:<14} {s:<12} {d:<12}')

# ========== 图4：速度-RMSE 扫描曲线 ==========
sweep_path = os.path.join(RESULTS_DIR, 'speed_sweep.csv')
if os.path.exists(sweep_path):
    sweep = np.loadtxt(sweep_path, delimiter=',', skiprows=1)
    speeds = sweep[:, 0]
    pp_rmse  = sweep[:, 1]
    st_rmse  = sweep[:, 2]
    lqr_rmse = sweep[:, 3]
    mpc_rmse = sweep[:, 4]

    fig4, (ax_main, ax_stanley) = plt.subplots(1, 2, figsize=(14, 5))
    fig4.suptitle('Speed vs RMSE — ISO 3888 Double Lane Change', fontsize=13, fontweight='bold')

    # 左图：PP / LQR / MPC（Stanley 太大放右图）
    ax_main.plot(speeds, pp_rmse,  'o-', color=COLORS['Pure Pursuit'], lw=2, ms=6, label='Pure Pursuit')
    ax_main.plot(speeds, lqr_rmse, 's-', color=COLORS['LQR'],          lw=2, ms=6, label='LQR')
    ax_main.plot(speeds, mpc_rmse, '^-', color=COLORS['MPC'],          lw=2, ms=6, label='MPC')
    ax_main.set_xlabel('Speed (m/s)'); ax_main.set_ylabel('RMSE (m)')
    ax_main.set_title('PP / LQR / MPC\n(Kinematic model — differences limited)')
    ax_main.legend(); ax_main.grid(True, alpha=0.3)
    ax_main.set_ylim(0, max(pp_rmse) * 1.4)
    # 标注趋势
    for v, r in zip(speeds, pp_rmse):
        ax_main.annotate(f'{r:.4f}', (v, r), textcoords='offset points', xytext=(0, 6),
                         ha='center', fontsize=7, color=COLORS['Pure Pursuit'])
    for v, r in zip(speeds, lqr_rmse):
        ax_main.annotate(f'{r:.4f}', (v, r), textcoords='offset points', xytext=(0, -12),
                         ha='center', fontsize=7, color=COLORS['LQR'])

    # 右图：Stanley（log 刻度）
    ax_stanley.semilogy(speeds, st_rmse, 'D-', color=COLORS['Stanley'], lw=2, ms=6, label='Stanley')
    ax_stanley.set_xlabel('Speed (m/s)'); ax_stanley.set_ylabel('RMSE (m, log scale)')
    ax_stanley.set_title('Stanley — High error at low speed\n(Architectural limitation)')
    ax_stanley.legend(); ax_stanley.grid(True, alpha=0.3, which='both')
    for v, r in zip(speeds, st_rmse):
        ax_stanley.annotate(f'{r:.2f}', (v, r), textcoords='offset points', xytext=(4, 0),
                            fontsize=8, color=COLORS['Stanley'])

    plt.tight_layout()
    out4 = os.path.join(OUTPUT_DIR, 'benchmark_speed_sweep.png')
    plt.savefig(out4, dpi=150)
    print(f'Saved: {out4}')
else:
    print('speed_sweep.csv not found, skipping speed sweep chart.')

# ========== 图5：计算耗时柱状图 ==========
timing_path = os.path.join(RESULTS_DIR, 'timing.csv')
if os.path.exists(timing_path):
    import csv
    timing = {}
    with open(timing_path) as f:
        for row in csv.DictReader(f):
            timing[row['controller']] = float(row['mean_us'])

    fig5, (ax_lin, ax_log) = plt.subplots(1, 2, figsize=(12, 5))
    fig5.suptitle('Computation Time per Control Cycle (1000-call average)', fontsize=13, fontweight='bold')

    names = list(timing.keys())
    vals  = [timing[n] for n in names]
    colors = [COLORS.get(n, '#95A5A6') for n in names]

    # 左：线性刻度
    bars = ax_lin.bar(names, vals, color=colors, width=0.5, edgecolor='white')
    for bar, v in zip(bars, vals):
        ax_lin.text(bar.get_x() + bar.get_width()/2, v + 0.5,
                    f'{v:.1f} μs', ha='center', va='bottom', fontsize=9, fontweight='bold')
    ax_lin.set_ylabel('Mean time (μs)'); ax_lin.set_title('Linear scale')
    ax_lin.grid(axis='y', alpha=0.3)
    ax_lin.set_ylim(0, max(vals) * 1.25)

    # 右：对数刻度（更直观看倍数）
    ax_log.bar(names, vals, color=colors, width=0.5, edgecolor='white')
    for i, (n, v) in enumerate(zip(names, vals)):
        ax_log.text(i, v * 1.3, f'{v:.1f} μs', ha='center', va='bottom', fontsize=9, fontweight='bold')
    ax_log.set_yscale('log')
    ax_log.set_ylabel('Mean time (μs, log scale)'); ax_log.set_title('Log scale — shows relative cost')
    ax_log.grid(axis='y', alpha=0.3, which='both')

    # 标注控制周期预算
    budget = 50000  # 20Hz → 50ms = 50000μs
    ax_lin.axhline(budget, color='red', linestyle='--', alpha=0.5, label='20Hz budget (50ms)')
    ax_log.axhline(budget, color='red', linestyle='--', alpha=0.5, label='20Hz budget (50ms)')
    ax_lin.legend(fontsize=8); ax_log.legend(fontsize=8)

    plt.tight_layout()
    out5 = os.path.join(OUTPUT_DIR, 'benchmark_timing.png')
    plt.savefig(out5, dpi=150)
    print(f'Saved: {out5}')
else:
    print('timing.csv not found, skipping timing chart.')

# ========== 图6：多指标雷达图 ==========
if os.path.exists(sweep_path) and os.path.exists(timing_path):
    import math, csv

    # 读取数据
    timing = {}
    with open(timing_path) as f:
        for row in csv.DictReader(f):
            timing[row['controller']] = float(row['mean_us'])

    sweep = np.loadtxt(sweep_path, delimiter=',', skiprows=1)
    spd_idx = {4.0: 0, 14.0: 5}
    rmse_4  = dict(zip(['Pure Pursuit','Stanley','LQR','MPC'], sweep[0, 1:]))
    rmse_14 = dict(zip(['Pure Pursuit','Stanley','LQR','MPC'], sweep[5, 1:]))
    # 速度鲁棒性：最大/最小 RMSE 比值，越接近1越稳定 → 分数 = 10/(ratio)
    robustness = {}
    for i, name in enumerate(['Pure Pursuit','Stanley','LQR','MPC']):
        vals = sweep[:, i+1]
        robustness[name] = max(vals) / (min(vals) + 1e-9)

    # 评分函数（log 归一化，输出 0-10）
    def rmse_to_score(r, best=0.05, worst=2.0):
        if r <= best: return 10.0
        if r >= worst: return 0.0
        return 10.0 * (math.log(worst) - math.log(r)) / (math.log(worst) - math.log(best))

    def time_to_score(t, best=1.0, worst=1000.0):
        if t <= best: return 10.0
        if t >= worst: return 0.0
        return 10.0 * (math.log(worst) - math.log(t)) / (math.log(worst) - math.log(best))

    def robust_to_score(ratio, best=1.0, worst=2000.0):
        if ratio <= best: return 10.0
        if ratio >= worst: return 0.0
        return 10.0 * (math.log(worst) - math.log(ratio)) / (math.log(worst) - math.log(best))

    labels = ['DLC Accuracy\n(8 m/s)', 'High-Speed\nStability (14 m/s)', 'Low-Speed\nStability (4 m/s)',
              'Real-Time\nPerformance', 'Speed\nRobustness']
    controllers = ['Pure Pursuit', 'Stanley', 'LQR', 'MPC']

    scores = {}
    for name in controllers:
        scores[name] = [
            rmse_to_score(dlc_rmse.get(name, 999)),
            rmse_to_score(rmse_14[name]),
            rmse_to_score(rmse_4[name]),
            time_to_score(timing[name]),
            robust_to_score(robustness[name]),
        ]

    N = len(labels)
    angles = np.linspace(0, 2 * np.pi, N, endpoint=False).tolist()
    angles += angles[:1]

    fig6, ax = plt.subplots(figsize=(8, 8), subplot_kw=dict(polar=True))
    ax.set_theta_offset(np.pi / 2)
    ax.set_theta_direction(-1)
    ax.set_thetagrids(np.degrees(angles[:-1]), labels, fontsize=11)
    ax.set_ylim(0, 10)
    ax.set_yticks([2, 4, 6, 8, 10])
    ax.set_yticklabels(['2', '4', '6', '8', '10'], fontsize=8, color='grey')
    ax.grid(color='grey', alpha=0.3)

    for name in controllers:
        vals = scores[name] + scores[name][:1]
        ax.plot(angles, vals, '-o', color=COLORS[name], lw=2, ms=5, label=name)
        ax.fill(angles, vals, color=COLORS[name], alpha=0.08)

    ax.set_title('Multi-Metric Controller Comparison\n(Score 0–10, higher = better)',
                 pad=20, fontsize=13, fontweight='bold')
    ax.legend(loc='upper right', bbox_to_anchor=(1.35, 1.15), fontsize=10)

    plt.tight_layout()
    out6 = os.path.join(OUTPUT_DIR, 'benchmark_radar.png')
    plt.savefig(out6, dpi=150, bbox_inches='tight')
    print(f'Saved: {out6}')

plt.show()
