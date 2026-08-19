#pragma once
#include <Eigen/Dense>
#include <vector>

class StanleyController {
public:
    struct Waypoint {
        double x, y, yaw, v_ref;
    };

    StanleyController(double wheelbase = 2.7,
                      double k         = 0.5,
                      double ks        = 1.0);

    std::pair<double, double> compute(
        const Eigen::Vector4d& state,
        const std::vector<Waypoint>& trajectory,
        int min_idx = 0);

    int lastNearestIdx() const { return last_nearest_idx_; }

private:
    double L_, k_, ks_;
    int last_nearest_idx_ = 0;
    static double normalizeAngle(double angle);
};
