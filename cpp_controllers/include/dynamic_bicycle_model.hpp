#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <utility>

// 动态自行车模型（线性轮胎模型）
// 状态量：[x, y, theta, v, beta, yaw_rate]
// 相比运动学模型，增加了质心侧偏角 beta 和横摆角速度 yaw_rate
// 适用速度：< 20 m/s（线性轮胎假设成立）
class DynamicBicycleModel {
public:
    struct State {
        double x        = 0.0;   // 世界坐标 x (m)
        double y        = 0.0;   // 世界坐标 y (m)
        double theta    = 0.0;   // 航向角 (rad)
        double v        = 0.0;   // 纵向速度 (m/s)
        double beta     = 0.0;   // 质心侧偏角 (rad)
        double yaw_rate = 0.0;   // 横摆角速度 r (rad/s)
    };

    struct Params {
        double m   = 1500.0;    // 整备质量 (kg)
        double Iz  = 2500.0;    // 横摆转动惯量 (kg·m²)
        double lf  = 1.2;       // 质心到前轴距离 (m)
        double lr  = 1.5;       // 质心到后轴距离 (m)
        double Cf  = 60000.0;   // 前轮侧偏刚度 (N/rad)
        double Cr  = 60000.0;   // 后轮侧偏刚度 (N/rad)
        double dt  = 0.05;      // 时间步长 (s)
        double max_steer = 0.5236;  // 最大转角 ±30° (rad)
        double max_speed = 20.0;
        double max_accel = 3.0;
    };

    DynamicBicycleModel();
    explicit DynamicBicycleModel(const Params& params);

    // RK4 积分更新一步，v < 0.5 m/s 时自动退化为运动学近似
    State update(const State& s, double delta, double accel) const;

    // 在速度 v 处线性化横向动力学（欧拉离散化）
    // 返回 (Ad, Bd)，其中：
    //   状态 [beta, yaw_rate]
    //   Ad: 2×2 状态转移矩阵
    //   Bd: 2×1 控制输入矩阵（输入为前轮转角 delta）
    std::pair<Eigen::Matrix2d, Eigen::Vector2d> linearize(double v) const;

    // 从 State 提取控制器可见的 4 维状态 [x, y, theta, v]
    static Eigen::Vector4d toKinematicState(const State& s);

    const Params& params() const { return params_; }

private:
    Params params_;

    // 计算状态导数（供 RK4 使用）
    // state_vec = [x, y, theta, beta, yaw_rate]，v 视为一步内常数
    Eigen::Matrix<double, 5, 1> derivatives(
        const Eigen::Matrix<double, 5, 1>& sv,
        double v, double delta) const;

    static double normalizeAngle(double angle);
};
