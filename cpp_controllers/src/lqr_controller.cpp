#include "lqr_controller.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

LQRController::LQRController() : params_(Params{}) {}
LQRController::LQRController(const Params& params) : params_(params) {}

std::pair<double, double> LQRController::compute(
    const Eigen::Vector4d& state,
    const std::vector<Waypoint>& trajectory,
    int min_idx)
{
    const double x = state[0], y = state[1];
    const double theta = state[2], v = state[3];

    // 找最近轨迹点（只在 min_idx 之后搜索，防止向后跳）
    int start = std::max(min_idx, 0);
    double min_dist = std::numeric_limits<double>::max();
    int idx = start;
    for (int i = start; i < (int)trajectory.size(); ++i) {
        double d = std::hypot(trajectory[i].x - x, trajectory[i].y - y);
        if (d < min_dist) { min_dist = d; idx = i; }
    }
    last_nearest_idx_ = idx;

    const double yaw_ref = trajectory[idx].yaw;
    const double v_ref   = trajectory[idx].v_ref;
    const double kappa   = trajectory[idx].curvature;

    // Frenet 坐标下的横向误差和航向误差
    double e_y = -(x - trajectory[idx].x) * std::sin(yaw_ref)
                + (y - trajectory[idx].y) * std::cos(yaw_ref);
    double e_theta = normalizeAngle(theta - yaw_ref);
    Eigen::Vector2d err_state(e_y, e_theta);

    // 线性化误差模型：e_{k+1} = A e_k + B delta
    const double dt = params_.dt;
    Eigen::Matrix2d A;
    A << 1.0, v * dt,
         0.0, 1.0;
    Eigen::Vector2d B(0.0, v / params_.wheelbase * dt);

    Eigen::Matrix2d Q = params_.Q_diag.asDiagonal();
    Eigen::Matrix2d P = solveDARE(A, B, Q, params_.R);

    double denom = params_.R + (B.transpose() * P * B)(0, 0);
    Eigen::RowVector2d K = (B.transpose() * P * A) / denom;

    // 前馈项：补偿参考轨迹曲率带来的稳态转角需求
    double ff = std::atan2(params_.wheelbase * kappa, 1.0);
    double delta = ff - (K * err_state)(0, 0);
    delta = std::clamp(delta, -0.5236, 0.5236);  // ±30°

    return {delta, v_ref};
}

Eigen::Matrix2d LQRController::solveDARE(
    const Eigen::Matrix2d& A, const Eigen::Vector2d& B,
    const Eigen::Matrix2d& Q, double R)
{
    Eigen::Matrix2d P = Q;
    for (int i = 0; i < params_.max_iter; ++i) {
        double denom = R + (B.transpose() * P * B)(0, 0);
        Eigen::Matrix2d Pn = A.transpose() * P * A
            - (A.transpose() * P * B) * (B.transpose() * P * A) / denom
            + Q;
        if ((Pn - P).norm() < params_.tolerance) { P = Pn; break; }
        P = Pn;
    }
    return P;
}

double LQRController::normalizeAngle(double angle) {
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 动态模型适配版本：B 向量改用动态横摆响应系数
std::pair<double, double> LQRController::computeDynamic(
    const Eigen::Vector4d& state,
    const std::vector<Waypoint>& trajectory,
    double Bc1,
    int min_idx)
{
    const double x = state[0], y = state[1];
    const double theta = state[2], v = state[3];

    int start = std::max(min_idx, 0);
    double min_dist = std::numeric_limits<double>::max();
    int idx = start;
    for (int i = start; i < (int)trajectory.size(); ++i) {
        double d = std::hypot(trajectory[i].x - x, trajectory[i].y - y);
        if (d < min_dist) { min_dist = d; idx = i; }
    }
    last_nearest_idx_ = idx;

    const double yaw_ref = trajectory[idx].yaw;
    const double v_ref   = trajectory[idx].v_ref;
    const double kappa   = trajectory[idx].curvature;

    double e_y     = -(x - trajectory[idx].x) * std::sin(yaw_ref)
                    + (y - trajectory[idx].y) * std::cos(yaw_ref);
    double e_theta = normalizeAngle(theta - yaw_ref);
    Eigen::Vector2d err_state(e_y, e_theta);

    const double dt = params_.dt;
    Eigen::Matrix2d A;
    A << 1.0, v * dt,
         0.0, 1.0;

    // B[1] = Bc1 × dt²：Bc1*dt 是 Δr/δ，再乘 dt 才得到 Δe_theta/δ
    // 物理含义：δ 先通过轮胎力矩改变横摆角速度，横摆角速度再积分改变航向角
    Eigen::Vector2d B(0.0, Bc1 * dt * dt);

    Eigen::Matrix2d Q = params_.Q_diag.asDiagonal();
    Eigen::Matrix2d P = solveDARE(A, B, Q, params_.R);

    double denom = params_.R + (B.transpose() * P * B)(0, 0);
    Eigen::RowVector2d K = (B.transpose() * P * A) / denom;

    double ff    = std::atan2(params_.wheelbase * kappa, 1.0);
    double delta = ff - (K * err_state)(0, 0);
    delta = std::clamp(delta, -0.5236, 0.5236);

    return {delta, v_ref};
}
