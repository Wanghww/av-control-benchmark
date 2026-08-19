import carla
import numpy as np


def get_waypoint_trajectory(world, start_transform, length=150, step=2.0, speed=8.0):
    """
    沿道路方向从 CARLA 地图提取路点，生成参考轨迹
    返回 N×4 数组，每行 [x, y, yaw_rad, v_ref]
    """
    carla_map = world.get_map()
    fwd = start_transform.get_forward_vector()

    # 沿车头方向逐渐前移，找到与车头方向一致的路点（避免对向车道）
    wp = None
    for dist in [3, 6, 10, 15, 20]:
        loc = start_transform.location + carla.Location(x=fwd.x * dist, y=fwd.y * dist)
        candidate = carla_map.get_waypoint(loc)
        wp_fwd = candidate.transform.get_forward_vector()
        dot = fwd.x * wp_fwd.x + fwd.y * wp_fwd.y
        if dot > 0:  # 方向一致（点积 > 0）
            wp = candidate
            break

    if wp is None:
        wp = carla_map.get_waypoint(start_transform.location)

    points = []
    for _ in range(int(length / step)):
        loc = wp.transform.location
        yaw = -np.radians(wp.transform.rotation.yaw)
        points.append([loc.x, -loc.y, yaw, speed])  # Y轴取反与 carla_env 保持一致

        nexts = wp.next(step)
        if not nexts:
            break
        wp = nexts[0]

    return np.array(points)
