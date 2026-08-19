#pragma once
#include <Eigen/Dense>
#include <cmath>

class BicycleModel {
public:
    using State = Eigen::Vector4d;

    struct Params {
        double wheelbase = 2.7;
        double dt        = 0.05;
        double max_steer = 0.5236;  // 30°
        double max_speed = 20.0;
        double max_accel = 3.0;
    };

    BicycleModel();
    explicit BicycleModel(const Params& params);

    State update(const State& state, double delta, double accel) const;

    const Params& params() const { return params_; }

private:
    Params params_;
    static double normalizeAngle(double angle);
};
