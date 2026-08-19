#pragma once
#include <Eigen/Dense>
#include <vector>

class LQRController {
public:
    struct Waypoint {
        double x, y, yaw, v_ref, curvature;
    };

    struct Params {
        double wheelbase = 2.7;
        double dt = 0.05;
        Eigen::Vector2d Q_diag{1.0, 1.0};  // [横向误差权重, 航向误差权重]
        double R = 1.0;                    // 转角权重
        int max_iter = 100;                // 黎卡提迭代次数
        double tolerance = 1e-8;           // 收敛阈值
    };

    LQRController();
    explicit LQRController(const Params& params);

    std::pair<double, double> compute(
        const Eigen::Vector4d& state,
        const std::vector<Waypoint>& trajectory,
        int min_idx = 0);

    // 动态模型适配版本：用动态横摆力矩系数 Bc1 = Cf*lf/Iz 替换运动学 v/L
    // 在高速场景下更准确地预测车辆横向响应
    std::pair<double, double> computeDynamic(
        const Eigen::Vector4d& state,
        const std::vector<Waypoint>& trajectory,
        double Bc1,        // 动态模型输入矩阵系数 Cf*lf/Iz (rad/s² per rad)
        int min_idx = 0);

    int lastNearestIdx() const { return last_nearest_idx_; }

private:
    Params params_;
    int last_nearest_idx_ = 0;

    // 求解离散代数黎卡提方程，返回矩阵 P
    Eigen::Matrix2d solveDARE(
        const Eigen::Matrix2d& A,
        const Eigen::Vector2d& B,
        const Eigen::Matrix2d& Q,
        double R);

    static double normalizeAngle(double angle);
};
