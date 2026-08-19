#include "dynamic_bicycle_model.hpp"
#include <algorithm>

DynamicBicycleModel::DynamicBicycleModel() : params_(Params{}) {}
DynamicBicycleModel::DynamicBicycleModel(const Params& p) : params_(p) {}

DynamicBicycleModel::State DynamicBicycleModel::update(
    const State& s, double delta, double accel) const
{
    delta = std::clamp(delta, -params_.max_steer, params_.max_steer);
    accel = std::clamp(accel, -params_.max_accel,  params_.max_accel);

    State ns = s;

    // v < 0.5 m/s：退化为运动学近似，避免 1/v 数值奇异
    if (s.v < 0.5) {
        const double L = params_.lf + params_.lr;
        ns.x      += s.v * std::cos(s.theta) * params_.dt;
        ns.y      += s.v * std::sin(s.theta) * params_.dt;
        ns.theta  += s.v / L * std::tan(delta) * params_.dt;
        ns.theta   = normalizeAngle(ns.theta);
        ns.v       = std::clamp(s.v + accel * params_.dt, 0.0, params_.max_speed);
        ns.beta    = 0.0;
        ns.yaw_rate = ns.v / L * std::tan(delta);
        return ns;
    }

    // RK4 积分（v 在一步内视为常数——标准工程近似）
    using Vec5 = Eigen::Matrix<double, 5, 1>;
    Vec5 sv;
    sv << s.x, s.y, s.theta, s.beta, s.yaw_rate;

    const double v  = s.v;
    const double dt = params_.dt;

    Vec5 k1 = derivatives(sv,             v, delta);
    Vec5 k2 = derivatives(sv + dt/2*k1,   v, delta);
    Vec5 k3 = derivatives(sv + dt/2*k2,   v, delta);
    Vec5 k4 = derivatives(sv + dt*k3,     v, delta);

    Vec5 sv_new = sv + dt / 6.0 * (k1 + 2*k2 + 2*k3 + k4);

    ns.x        = sv_new[0];
    ns.y        = sv_new[1];
    ns.theta    = normalizeAngle(sv_new[2]);
    ns.beta     = std::clamp(sv_new[3], -0.2, 0.2);      // 限制在线性轮胎有效范围
    ns.yaw_rate = std::clamp(sv_new[4], -1.0, 1.0);
    ns.v        = std::clamp(s.v + accel * dt, 0.0, params_.max_speed);

    return ns;
}

Eigen::Matrix<double, 5, 1> DynamicBicycleModel::derivatives(
    const Eigen::Matrix<double, 5, 1>& sv, double v, double delta) const
{
    const double theta = sv[2];
    const double beta  = sv[3];
    const double r     = sv[4];

    const double m  = params_.m,  Iz = params_.Iz;
    const double lf = params_.lf, lr = params_.lr;
    const double Cf = params_.Cf, Cr = params_.Cr;

    const double dx    = v * std::cos(theta + beta);
    const double dy    = v * std::sin(theta + beta);
    const double dtheta = r;

    // 线性轮胎模型横向动力学方程
    const double dbeta = (-(Cf + Cr) / (m * v) * beta
                         + ((Cr * lr - Cf * lf) / (m * v * v) - 1.0) * r
                         + Cf / (m * v) * delta);

    const double dr    = ((Cr * lr - Cf * lf) / Iz * beta
                         - (Cf * lf * lf + Cr * lr * lr) / (Iz * v) * r
                         + Cf * lf / Iz * delta);

    Eigen::Matrix<double, 5, 1> deriv;
    deriv << dx, dy, dtheta, dbeta, dr;
    return deriv;
}

std::pair<Eigen::Matrix2d, Eigen::Vector2d>
DynamicBicycleModel::linearize(double v) const
{
    const double m  = params_.m,  Iz = params_.Iz;
    const double lf = params_.lf, lr = params_.lr;
    const double Cf = params_.Cf, Cr = params_.Cr;
    const double dt = params_.dt;

    // 连续时间横向状态矩阵 Ac（状态：[beta, r]）
    Eigen::Matrix2d Ac;
    Ac(0, 0) = -(Cf + Cr) / (m * v);
    Ac(0, 1) =  (Cr * lr - Cf * lf) / (m * v * v) - 1.0;
    Ac(1, 0) =  (Cr * lr - Cf * lf) / Iz;
    Ac(1, 1) = -(Cf * lf * lf + Cr * lr * lr) / (Iz * v);

    // 连续时间输入矩阵 Bc
    Eigen::Vector2d Bc;
    Bc(0) = Cf / (m * v);
    Bc(1) = Cf * lf / Iz;

    // 欧拉离散化：Ad = I + Ac*dt，Bd = Bc*dt
    Eigen::Matrix2d Ad = Eigen::Matrix2d::Identity() + Ac * dt;
    Eigen::Vector2d Bd = Bc * dt;

    return {Ad, Bd};
}

Eigen::Vector4d DynamicBicycleModel::toKinematicState(const State& s)
{
    Eigen::Vector4d ks;
    ks << s.x, s.y, s.theta, s.v;
    return ks;
}

double DynamicBicycleModel::normalizeAngle(double angle)
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
