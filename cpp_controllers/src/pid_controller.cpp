#include "pid_controller.hpp"
#include <algorithm>

PIDController::PIDController(double kp, double ki, double kd, double dt)
    : kp_(kp), ki_(ki), kd_(kd), dt_(dt),
      error_sum_(0.0), last_error_(0.0) {}

double PIDController::compute(double target, double current) {
    double error = target - current;

    error_sum_ += error * dt_;
    error_sum_  = std::clamp(error_sum_, -10.0, 10.0);  // 积分限幅，防止饱和

    double error_diff = (error - last_error_) / dt_;
    last_error_ = error;

    return kp_ * error + ki_ * error_sum_ + kd_ * error_diff;
}

void PIDController::reset() {
    error_sum_  = 0.0;
    last_error_ = 0.0;
}
