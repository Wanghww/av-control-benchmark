import numpy as np


def generate_straight(length=100, step=1.0, speed=8.0):
    x = np.arange(0, length, step)
    y = np.zeros_like(x)
    yaw = np.zeros_like(x)
    v = np.full_like(x, speed)
    return np.column_stack([x, y, yaw, v])


def generate_circle(radius=20, n_points=200, speed=5.0):
    theta = np.linspace(0, 2 * np.pi, n_points)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    yaw = theta + np.pi / 2
    v = np.full(n_points, speed)
    return np.column_stack([x, y, yaw, v])


def generate_s_curve(length=100, amplitude=10, speed=6.0):
    x = np.linspace(0, length, 300)
    y = amplitude * np.sin(2 * np.pi * x / length)
    dx = np.gradient(x)
    dy = np.gradient(y)
    yaw = np.arctan2(dy, dx)
    v = np.full_like(x, speed)
    return np.column_stack([x, y, yaw, v])


def generate_double_lane_change(speed=8.0):
    """ISO 3888 双移线轨迹，MPC vs 其他算法差异最明显"""
    points = []
    for x in np.linspace(0, 30, 60):
        points.append([x, 0, 0, speed])
    for t in np.linspace(0, np.pi, 40):
        x = 30 + t / np.pi * 20
        y = 3.5 * (1 - np.cos(t)) / 2
        points.append([x, y, 0, speed])
    for x in np.linspace(50, 80, 30):
        points.append([x, 3.5, 0, speed])
    for t in np.linspace(0, np.pi, 40):
        x = 80 + t / np.pi * 20
        y = 3.5 - 3.5 * (1 - np.cos(t)) / 2
        points.append([x, y, 0, speed])
    for x in np.linspace(100, 130, 30):
        points.append([x, 0, 0, speed])

    traj = np.array(points)
    dx = np.gradient(traj[:, 0])
    dy = np.gradient(traj[:, 1])
    traj[:, 2] = np.arctan2(dy, dx)
    return traj
