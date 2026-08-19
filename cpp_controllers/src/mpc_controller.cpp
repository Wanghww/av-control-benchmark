#include "mpc_controller.hpp"
#include <OsqpEigen/OsqpEigen.h>
#include <cmath>
#include <limits>
#include <algorithm>

MPCController::MPCController() : params_(Params{}) {}
MPCController::MPCController(const Params& params) : params_(params) {}

std::pair<double, double> MPCController::compute(
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

    const auto& wp = trajectory[idx];
    const double yaw_ref = wp.yaw;
    const double v_ref   = wp.v_ref;
    const double kappa   = wp.curvature;

    // 初始误差状态（Frenet 坐标）
    double e_y = -(x - wp.x) * std::sin(yaw_ref) + (y - wp.y) * std::cos(yaw_ref);
    double e_theta = normalizeAngle(theta - yaw_ref);
    Eigen::Vector2d x0(e_y, e_theta);

    // 前馈转角（曲率补偿，用于代价函数中的参考控制量）
    const double L  = params_.wheelbase;
    double ff = std::atan2(L * kappa, 1.0);

    // 速度取较小值防止 v=0 时模型退化
    double v_eff = std::max(std::abs(v), 0.5);
    const double dt = params_.dt;

    // 线性化误差动力学：e_{k+1} = A e_k + B delta_k
    // 状态量 e = [e_y, e_theta]，控制量 delta (转角)
    Eigen::Matrix2d A;
    A << 1.0, v_eff * dt,
         0.0, 1.0;
    Eigen::Vector2d B(0.0, v_eff / L * dt);

    // QP 变量：z = [x_1,...,x_N, u_0,...,u_{N-1}]
    // x_k ∈ R^2, u_k ∈ R^1
    const int N    = params_.N;
    const int nx   = 2;
    const int nu   = 1;
    const int nz   = nx * N + nu * N;  // 总决策变量数
    const int neq  = nx * N;           // 等式约束（动态方程）
    const int nineq = nu * N;          // 不等式约束（转角约束）
    const int nc   = neq + nineq;      // 总约束数

    // OSQP 形式：min (1/2) z^T P z + q^T z
    // 将原始权重乘以 2 以匹配 OSQP 的 1/2 系数
    const double p_ey  = 2.0 * params_.q_ey;
    const double p_eth = 2.0 * params_.q_etheta;
    const double pf_ey = 2.0 * params_.qf_ey;
    const double pf_eth = 2.0 * params_.qf_etheta;
    const double p_R   = 2.0 * params_.R;

    // ---- Hessian 矩阵 P（纯对角，上三角即为自身）----
    Eigen::SparseMatrix<double> P(nz, nz);
    {
        std::vector<Eigen::Triplet<double>> trips;
        for (int k = 0; k < N; ++k) {
            bool term = (k == N - 1);
            int base = k * nx;
            trips.push_back({base,     base,     term ? pf_ey : p_ey});
            trips.push_back({base + 1, base + 1, term ? pf_eth : p_eth});
        }
        for (int k = 0; k < N; ++k) {
            int col = nx * N + k;
            trips.push_back({col, col, p_R});
        }
        P.setFromTriplets(trips.begin(), trips.end());
    }

    // ---- 梯度向量 q ----
    // 控制前馈：cost += p_R * (u - ff)^2，线性项为 -p_R * ff
    Eigen::VectorXd q_vec = Eigen::VectorXd::Zero(nz);
    for (int k = 0; k < N; ++k) {
        q_vec(nx * N + k) = -p_R * ff;
    }

    // ---- 约束矩阵 A 和上下界 ----
    Eigen::SparseMatrix<double> A_con(nc, nz);
    Eigen::VectorXd lb(nc), ub(nc);
    {
        std::vector<Eigen::Triplet<double>> trips;

        // 1. 等式约束（动力学方程）
        // k=0: x_1 = A*x0 + B*u_0 → I*x_1 - B*u_0 = A*x0
        {
            trips.push_back({0, 0, 1.0});
            trips.push_back({1, 1, 1.0});
            trips.push_back({1, nx * N, -B(1)});  // -B[1]*u_0
            Eigen::Vector2d rhs = A * x0;
            lb(0) = rhs(0); ub(0) = rhs(0);
            lb(1) = rhs(1); ub(1) = rhs(1);
        }
        // k=1..N-1: -A*x_k + x_{k+1} - B*u_k = 0
        for (int k = 1; k < N; ++k) {
            int row    = k * nx;
            int xk_col = (k - 1) * nx;
            int xk1_col = k * nx;
            int uk_col  = nx * N + k;

            trips.push_back({row,     xk_col,     -A(0, 0)});
            trips.push_back({row,     xk_col + 1, -A(0, 1)});
            trips.push_back({row + 1, xk_col,     -A(1, 0)});
            trips.push_back({row + 1, xk_col + 1, -A(1, 1)});
            trips.push_back({row,     xk1_col,     1.0});
            trips.push_back({row + 1, xk1_col + 1, 1.0});
            trips.push_back({row + 1, uk_col, -B(1)});

            lb(row) = 0.0; ub(row) = 0.0;
            lb(row + 1) = 0.0; ub(row + 1) = 0.0;
        }

        // 2. 不等式约束（转角约束）
        for (int k = 0; k < N; ++k) {
            int row = neq + k;
            int col = nx * N + k;
            trips.push_back({row, col, 1.0});
            lb(row) = -params_.max_steer;
            ub(row) =  params_.max_steer;
        }

        A_con.setFromTriplets(trips.begin(), trips.end());
    }

    // ---- 设置并求解 QP ----
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(false);
    solver.settings()->setMaxIteration(500);
    solver.settings()->setAbsoluteTolerance(1e-4);
    solver.settings()->setRelativeTolerance(1e-4);

    solver.data()->setNumberOfVariables(nz);
    solver.data()->setNumberOfConstraints(nc);

    if (!solver.data()->setHessianMatrix(P))             return {ff, v_ref};
    if (!solver.data()->setGradient(q_vec))              return {ff, v_ref};
    if (!solver.data()->setLinearConstraintsMatrix(A_con)) return {ff, v_ref};
    if (!solver.data()->setLowerBound(lb))               return {ff, v_ref};
    if (!solver.data()->setUpperBound(ub))               return {ff, v_ref};

    if (!solver.initSolver()) return {ff, v_ref};
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return {ff, v_ref};

    Eigen::VectorXd sol = solver.getSolution();
    double delta = std::clamp(sol(nx * N), -params_.max_steer, params_.max_steer);

    return {delta, v_ref};
}

double MPCController::normalizeAngle(double angle) {
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
