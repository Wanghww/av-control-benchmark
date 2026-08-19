#!/usr/bin/env python3
"""
第六阶段 Benchmark 可视化：动态模型 vs 运动学模型对比
生成两张图：
  1. 速度-RMSE 曲线（四条线：PP_kin / LQR_kin / LQR_dyn / MPC_kin）
  2. 15 m/s 双移线轨迹对比（PP_kin vs LQR_dyn）

运行方式（在 cpp_controllers 目录下编译并跑完仿真后）：
  cd cpp_controllers/scripts
  python3 plot_dynamic_comparison.py
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import pandas as pd
import os

# 结果文件路径（相对于脚本位置）
RESULTS_DIR = os.path.join(os.path.dirname(__file__), "..", "results")

# ── 样式 ─────────────────────────────────────────────────────────────────────
plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "grid.alpha": 0.35,
    "grid.linestyle": "--",
    "font.size": 11,
})

COLORS = {
    "pp_kin":  "#e17055",   # 橙红：Pure Pursuit（运动学控制器）
    "lqr_kin": "#74b9ff",   # 浅蓝：LQR 运动学
    "lqr_dyn": "#0984e3",   # 深蓝：LQR 动态适配
    "mpc_kin": "#6c5ce7",   # 紫色：MPC 运动学
    "ref":     "#2d3436",   # 深灰：参考轨迹
}

LABELS = {
    "pp_kin":  "Pure Pursuit (kinematic ctrl)",
    "lqr_kin": "LQR (kinematic ctrl)",
    "lqr_dyn": "LQR (dynamic-adapted ctrl)",
    "mpc_kin": "MPC (kinematic ctrl)",
}

# ── 读数据 ────────────────────────────────────────────────────────────────────
def load_csv(name):
    path = os.path.join(RESULTS_DIR, name)
    if not os.path.exists(path):
        print(f"[警告] 找不到文件：{path}，跳过")
        return None
    return pd.read_csv(path)

sweep = load_csv("dynamic_speed_sweep.csv")
traj_pp15  = load_csv("dyn15_pp.csv")
traj_lqr15 = load_csv("dyn15_lqr.csv")

# ── 画布 ──────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(14, 6))
gs  = gridspec.GridSpec(1, 2, figure=fig, wspace=0.35)

# ── 左图：速度-RMSE 曲线 ───────────────────────────────────────────────────────
ax1 = fig.add_subplot(gs[0])

if sweep is not None:
    speeds = sweep["speed"].values
    for key in ["pp_kin", "lqr_kin", "lqr_dyn", "mpc_kin"]:
        lw = 2.5 if key == "lqr_dyn" else 1.8
        ls = "-"  if key == "lqr_dyn" else "--"
        ax1.plot(speeds, sweep[key].values,
                 color=COLORS[key], label=LABELS[key],
                 linewidth=lw, linestyle=ls, marker="o", markersize=5)

    # 标注 15 m/s 改善幅度
    if 15.0 in speeds:
        idx15 = list(speeds).index(15.0)
        pp_r  = sweep["pp_kin"].iloc[idx15]
        ld_r  = sweep["lqr_dyn"].iloc[idx15]
        pct   = (pp_r - ld_r) / pp_r * 100
        ax1.annotate(
            f"−{pct:.0f}%\n@15 m/s",
            xy=(15.0, ld_r), xytext=(13.5, ld_r + 0.02),
            arrowprops=dict(arrowstyle="->", color="#0984e3"),
            color="#0984e3", fontsize=9, fontweight="bold",
        )

ax1.set_xlabel("Speed (m/s)")
ax1.set_ylabel("RMSE (m)")
ax1.set_title("Dynamic Bicycle Model — Speed-RMSE Sweep\n(Double Lane Change, ISO 3888)",
              fontsize=11, fontweight="bold")
ax1.legend(fontsize=8.5, loc="upper left")
ax1.set_xlim(2, 20)

# ── 右图：15 m/s 双移线轨迹对比 ───────────────────────────────────────────────
ax2 = fig.add_subplot(gs[1])

if traj_pp15 is not None and traj_lqr15 is not None:
    # 参考轨迹（从 PP 数据中重建不太准，改用数值生成）
    x_ref = np.linspace(0, 130, 400)
    y_ref = np.zeros_like(x_ref)
    for i, xv in enumerate(x_ref):
        if xv < 30:
            y_ref[i] = 0
        elif xv < 50:
            s = xv - 30
            y_ref[i] = 3.5 / 2 * (1 - np.cos(np.pi * s / 20))
        elif xv < 80:
            y_ref[i] = 3.5
        elif xv < 100:
            s = xv - 80
            y_ref[i] = 3.5 / 2 * (1 + np.cos(np.pi * s / 20))

    ax2.plot(x_ref, y_ref, color=COLORS["ref"], lw=1.5,
             linestyle="--", label="Reference", zorder=3)
    ax2.plot(traj_pp15["x"],  traj_pp15["y"],
             color=COLORS["pp_kin"],  lw=1.8, label=LABELS["pp_kin"],  zorder=4)
    ax2.plot(traj_lqr15["x"], traj_lqr15["y"],
             color=COLORS["lqr_dyn"], lw=2.2, label=LABELS["lqr_dyn"], zorder=5)

    # 计算 RMSE 并标注
    def rmse_from_df(df):
        errs = df["lateral_error"].values
        return np.sqrt(np.mean(errs ** 2))

    r_pp  = rmse_from_df(traj_pp15)
    r_lqr = rmse_from_df(traj_lqr15)
    pct   = (r_pp - r_lqr) / r_pp * 100 if r_pp > 0 else 0

    ax2.set_title(f"15 m/s Double Lane Change — Dynamic Model\n"
                  f"PP RMSE={r_pp:.3f} m   LQR-dyn RMSE={r_lqr:.3f} m  "
                  f"({pct:.0f}% improvement)",
                  fontsize=10, fontweight="bold")

ax2.set_xlabel("X (m)")
ax2.set_ylabel("Y (m)")
ax2.legend(fontsize=8.5, loc="lower right")
ax2.set_xlim(-5, 135)
ax2.set_ylim(-1.0, 5.0)
ax2.set_aspect("auto")

# ── 保存 ──────────────────────────────────────────────────────────────────────
out_path = os.path.join(RESULTS_DIR, "benchmark_dynamic_comparison.png")
plt.savefig(out_path, dpi=150, bbox_inches="tight")
print(f"图表已保存：{out_path}")
plt.show()
