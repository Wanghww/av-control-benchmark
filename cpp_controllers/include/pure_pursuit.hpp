#pragma once
#include <Eigen/Dense>
#include <vector>

class PurePursuitController {
public:
    struct Waypoint {
        double x, y, yaw, v_ref;
    };

    PurePursuitController(double wheelbase = 2.7,
                          double k        = 0.3,
                          double ld_min   = 3.0);

    std::pair<double, double> compute(
        const Eigen::Vector4d& state,
        const std::vector<Waypoint>& trajectory,
        int min_idx = 0);

    int lastNearestIdx() const { return last_nearest_idx_; }

private:
    double L_, k_, ld_min_;
    int last_nearest_idx_ = 0;

    static double normalizeAngle(double angle);
};
