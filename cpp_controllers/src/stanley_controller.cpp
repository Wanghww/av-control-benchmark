#include "stanley_controller.hpp"
#include <cmath>
#include <limits>

StanleyController::StanleyController(double wheelbase, double k, double ks)
    : L_(wheelbase), k_(k), ks_(ks) {}

std::pair<double, double> StanleyController::compute(
    const Eigen::Vector4d& state,
    const std::vector<Waypoint>& trajectory,
    int min_idx)
{
    const double x     = state[0];
    const double y     = state[1];
    const double theta = state[2];
    const double v     = state[3];

    // 前轴位置
    double front_x = x + L_ * std::cos(theta);
    double front_y = y + L_ * std::sin(theta);

    // 只在 min_idx 之后搜索，防止向后跳
    int start = std::max(min_idx, 0);
    double min_dist = std::numeric_limits<double>::max();
    int nearest = start;
    for (int i = start; i < (int)trajectory.size(); ++i) {
        double d = std::hypot(trajectory[i].x - front_x,
                              trajectory[i].y - front_y);
        if (d < min_dist) { min_dist = d; nearest = i; }
    }
    last_nearest_idx_ = nearest;

    double theta_ref = trajectory[nearest].yaw;
    double theta_e   = normalizeAngle(theta_ref - theta);

    // 横向误差（带符号）
    double dx  = front_x - trajectory[nearest].x;
    double dy  = front_y - trajectory[nearest].y;
    double e_y = -dx * std::sin(theta_ref) + dy * std::cos(theta_ref);

    double delta = theta_e + std::atan2(k_ * e_y, ks_ + v);

    return {delta, trajectory[nearest].v_ref};
}

double StanleyController::normalizeAngle(double angle) {
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
