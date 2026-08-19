#include "pure_pursuit.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

PurePursuitController::PurePursuitController(
    double wheelbase, double k, double ld_min)
    : L_(wheelbase), k_(k), ld_min_(ld_min) {}

std::pair<double, double> PurePursuitController::compute(
    const Eigen::Vector4d& state,
    const std::vector<Waypoint>& trajectory,
    int min_idx)
{
    const double x     = state[0];
    const double y     = state[1];
    const double theta = state[2];
    const double v     = state[3];

    double ld = std::max(k_ * v + ld_min_, ld_min_);

    // 只在 min_idx 之后搜索，防止向后跳
    int start = std::max(min_idx, 0);
    double min_dist = std::numeric_limits<double>::max();
    int nearest = start;
    for (int i = start; i < (int)trajectory.size(); ++i) {
        double d = std::hypot(trajectory[i].x - x, trajectory[i].y - y);
        if (d < min_dist) { min_dist = d; nearest = i; }
    }
    last_nearest_idx_ = nearest;

    // 从最近点向前找前视点
    int target = nearest;
    for (int i = nearest; i < (int)trajectory.size(); ++i) {
        if (std::hypot(trajectory[i].x - x, trajectory[i].y - y) >= ld) {
            target = i;
            break;
        }
    }

    double alpha = std::atan2(trajectory[target].y - y,
                               trajectory[target].x - x) - theta;
    alpha = normalizeAngle(alpha);

    double delta = std::atan2(2.0 * L_ * std::sin(alpha), ld);

    return {delta, trajectory[target].v_ref};
}

double PurePursuitController::normalizeAngle(double angle) {
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
