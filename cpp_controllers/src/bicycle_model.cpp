#include "bicycle_model.hpp"
#include <algorithm>

BicycleModel::BicycleModel() : params_(Params{}) {}
BicycleModel::BicycleModel(const Params& params) : params_(params) {}

BicycleModel::State BicycleModel::update(
    const State& state, double delta, double accel) const
{
    delta = std::clamp(delta, -params_.max_steer, params_.max_steer);
    accel = std::clamp(accel, -params_.max_accel,  params_.max_accel);

    const double x     = state[0];
    const double y     = state[1];
    const double theta = state[2];
    const double v     = state[3];
    const double dt    = params_.dt;

    State next;
    next[0] = x + v * std::cos(theta) * dt;
    next[1] = y + v * std::sin(theta) * dt;
    next[2] = normalizeAngle(theta + v / params_.wheelbase * std::tan(delta) * dt);
    next[3] = std::clamp(v + accel * dt, 0.0, params_.max_speed);

    return next;
}

double BicycleModel::normalizeAngle(double angle) {
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
