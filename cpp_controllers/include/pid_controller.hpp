#pragma once

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double dt = 0.05);

    double compute(double target, double current);
    void reset();

private:
    double kp_, ki_, kd_, dt_;
    double error_sum_;
    double last_error_;
};
