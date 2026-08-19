#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

class MPCController {
public:
    struct Waypoint {
        double x, y, yaw, v_ref, curvature;
    };

    struct Params {
        int    N         = 10;     // 预测步数
        double dt        = 0.05;   // 时间步长 (s)
        double wheelbase = 2.7;    // 轴距 (m)
        double q_ey      = 10.0;   // 横向误差权重
        double q_etheta  = 1.0;    // 航向误差权重
        double qf_ey     = 20.0;   // 终端横向误差权重
        double qf_etheta = 2.0;    // 终端航向误差权重
        double R         = 0.1;    // 转角权重
        double max_steer = 0.5236; // 最大转角 ±30° (rad)
    };

    MPCController();
    explicit MPCController(const Params& params);

    // 返回 {转角 delta (rad), 参考速度 v_ref (m/s)}
    std::pair<double, double> compute(
        const Eigen::Vector4d& state,
        const std::vector<Waypoint>& trajectory,
        int min_idx = 0);

    int lastNearestIdx() const { return last_nearest_idx_; }

private:
    Params params_;
    int last_nearest_idx_ = 0;

    static double normalizeAngle(double angle);
};
