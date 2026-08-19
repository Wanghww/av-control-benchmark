# 第四阶段：MPC 控制器（第 13-18 周）

## 本阶段目标

实现 LQR 和 MPC 两种基于优化的控制器，MPC 支持横纵向联合控制和约束处理。

**里程碑：MPC 在 ISO 3888 双移线场景下横向 RMSE 低于 Pure Pursuit 30% 以上，有数据为证。**

---

## 理论准备（第 13 周）

### 13.1 LQR 原理

**Linear Quadratic Regulator（线性二次调节器）**

LQR 把路径跟踪问题转化为一个最优控制问题：

**目标：** 找到控制输入 u，使代价函数 J 最小：

```
J = Σ (x^T Q x + u^T R u)

x = 状态误差向量（横向误差、航向误差等）
u = 控制输入（转角）
Q = 状态误差权重矩阵（调大 → 误差更小）
R = 控制量权重矩阵（调大 → 控制更平滑）
```

**线性化误差模型（围绕参考轨迹展开）：**

```
状态量：e = [e_y, e_y_dot, e_θ, e_θ_dot]
  e_y      = 横向位置误差
  e_y_dot  = 横向速度误差
  e_θ      = 航向角误差
  e_θ_dot  = 航向角速度误差

状态方程：ė = A·e + B·δ

其中 A、B 矩阵由车速 v 和轴距 L 决定
```

**求解步骤：**
1. 求解 离散代数黎卡提方程（DARE）得到最优增益矩阵 P
2. 最优控制律：`δ = -K·e`，其中 `K = (R + B^T P B)^{-1} B^T P A`

---

### 13.2 MPC 原理

**Model Predictive Control（模型预测控制）**

MPC 在每个控制步做以下事情：

```
1. 用当前状态，预测未来 N 步的轨迹
2. 求解一个优化问题，找到最优控制序列
3. 只执行第一步控制量
4. 下一步重复（滚动优化）
```

**优化问题形式（二次规划 QP）：**

```
min  Σ_{k=0}^{N-1} (e_k^T Q e_k + u_k^T R u_k) + e_N^T P e_N
 u

subject to:
  x_{k+1} = A x_k + B u_k        (预测模型)
  |δ| ≤ δ_max                    (转角约束)
  |Δδ| ≤ Δδ_max                  (转角变化率约束，保证平滑)
  |a| ≤ a_max                    (加速度约束)
```

**MPC vs LQR vs Pure Pursuit：**

| 对比项 | Pure Pursuit | LQR | MPC |
|--------|-------------|-----|-----|
| 是否考虑未来 | 否（仅当前步） | 否 | 是（预测 N 步）|
| 是否处理约束 | 否 | 否 | 是 |
| 计算复杂度 | 低 | 中 | 高 |
| 高速弯道表现 | 较差 | 中等 | 最好 |
| 实现难度 | 简单 | 中等 | 较难 |

---

## 第十四周：安装 OSQP + 实现 LQR

### 14.1 安装 OSQP

```bash
# 从源码编译安装
cd ~
git clone https://github.com/osqp/osqp.git
cd osqp
git submodule update --init --recursive
mkdir build && cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j4
sudo make install

# 安装 C++ 接口
cd ~
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake ..
make -j4
sudo make install

# 验证
pkg-config --modversion osqp
```

---

### 14.2 实现 LQR 横向控制器

创建 `include/lqr_controller.hpp`：

```cpp
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
        // Q 矩阵对角元素：[横向误差, 横向误差速率, 航向误差, 航向误差速率]
        Eigen::Vector4d Q_diag{1.0, 0.0, 1.0, 0.0};
        // R 矩阵（转角权重）
        double R = 1.0;
        int max_iter = 100;         // 黎卡提迭代次数
        double tolerance = 1e-8;    // 收敛阈值
    };

    explicit LQRController(const Params& params = Params{});

    std::pair<double, double> compute(
        const Eigen::Vector4d& state,
        const std::vector<Waypoint>& trajectory);

private:
    Params params_;

    // 求解离散代数黎卡提方程
    Eigen::MatrixXd solveDARE(
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& B,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R);

    // 构建线性化误差模型矩阵 A, B
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
    buildLinearModel(double v, double phi_ref);

    int findNearestIndex(const Eigen::Vector2d& pos,
                         const std::vector<Waypoint>& traj);
    static double normalizeAngle(double angle);
};
```

创建 `src/lqr_controller.cpp`：

```cpp
#include "lqr_controller.hpp"
#include <cmath>
#include <limits>

LQRController::LQRController(const Params& params) : params_(params) {}

std::pair<double, double> LQRController::compute(
    const Eigen::Vector4d& state,
    const std::vector<LQRController::Waypoint>& trajectory)
{
    const double x = state[0], y = state[1];
    const double theta = state[2], v = state[3];

    // 找最近轨迹点
    int idx = findNearestIndex(Eigen::Vector2d(x, y), trajectory);

    // 参考点信息
    double x_ref   = trajectory[idx].x;
    double y_ref   = trajectory[idx].y;
    double yaw_ref = trajectory[idx].yaw;
    double v_ref   = trajectory[idx].v_ref;
    double kappa   = trajectory[idx].curvature;  // 参考曲率

    // 计算误差向量（在 Frenet 坐标系下）
    double e_y = -(x - x_ref) * std::sin(yaw_ref)
                + (y - y_ref) * std::cos(yaw_ref);  // 横向误差
    double e_theta = normalizeAngle(theta - yaw_ref);  // 航向误差

    // 构建状态向量（简化为 2 状态）
    Eigen::Vector2d err_state(e_y, e_theta);

    // 构建线性化模型（简化版 2 阶）
    double dt = params_.dt;
    Eigen::Matrix2d A;
    A << 1, v * dt,
         0, 1;
    Eigen::Vector2d B(0, v / params_.wheelbase * dt);

    Eigen::Matrix2d Q = Eigen::Matrix2d::Zero();
    Q(0, 0) = params_.Q_diag[0];  // 横向误差权重
    Q(1, 1) = params_.Q_diag[2];  // 航向误差权重
    Eigen::Matrix<double, 1, 1> R;
    R(0, 0) = params_.R;

    // 求解 DARE
    Eigen::MatrixXd P = solveDARE(A, B.reshaped(2, 1), Q, R);

    // 最优增益
    Eigen::MatrixXd K = (R + B.transpose() * P * B).inverse()
                        * B.transpose() * P * A;

    // 前馈项（补偿参考曲率）
    double ff = std::atan2(params_.wheelbase * kappa, 1.0);

    // 控制律
    double delta = ff - (K * err_state)(0);
    delta = std::clamp(delta, -0.5236, 0.5236);  // ±30°

    return {delta, v_ref};
}

Eigen::MatrixXd LQRController::solveDARE(
    const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R)
{
    Eigen::MatrixXd P = Q;
    for (int i = 0; i < params_.max_iter; ++i) {
        Eigen::MatrixXd Pn = A.transpose() * P * A
            - A.transpose() * P * B
              * (R + B.transpose() * P * B).inverse()
              * B.transpose() * P * A
            + Q;
        if ((Pn - P).norm() < params_.tolerance) break;
        P = Pn;
    }
    return P;
}

int LQRController::findNearestIndex(
    const Eigen::Vector2d& pos, const std::vector<Waypoint>& traj)
{
    double min_dist = std::numeric_limits<double>::max();
    int idx = 0;
    for (int i = 0; i < (int)traj.size(); ++i) {
        double d = std::hypot(traj[i].x - pos[0], traj[i].y - pos[1]);
        if (d < min_dist) { min_dist = d; idx = i; }
    }
    return idx;
}

double LQRController::normalizeAngle(double angle) {
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
```

---

## 第十五至十六周：实现 MPC 控制器

### 15.1 MPC 横纵向联合控制器

创建 `include/mpc_controller.hpp`：

```cpp
#pragma once
#include <Eigen/Dense>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>

class MPCController {
public:
    struct Waypoint {
        double x, y, yaw, v_ref;
    };

    struct Params {
        int    N        = 10;       // 预测步数
        double dt       = 0.05;     // 时间步长 (s)
        double wheelbase = 2.7;     // 轴距 (m)

        // 状态权重 Q：[x误差, y误差, θ误差, v误差]
        Eigen::Vector4d Q_diag{10.0, 10.0, 1.0, 1.0};
        // 终端权重（通常比 Q 大）
        Eigen::Vector4d Qf_diag{20.0, 20.0, 2.0, 2.0};
        // 控制权重 R：[转角, 加速度]
        Eigen::Vector2d R_diag{0.1, 0.1};
        // 控制变化量权重（抑制抖动）
        Eigen::Vector2d Rd_diag{0.01, 0.01};

        // 约束
        double max_steer   = 0.5236;  // 最大转角 30° (rad)
        double max_accel   = 3.0;     // 最大加速度 (m/s²)
        double max_speed   = 20.0;    // 最大速度 (m/s)
        double max_d_steer = 0.1;     // 每步最大转角变化 (rad)
    };

    explicit MPCController(const Params& params = Params{});

    // 返回 {转角(rad), 加速度(m/s²)}
    std::pair<double, double> compute(
        const Eigen::Vector4d& state,
        const std::vector<Waypoint>& trajectory);

private:
    Params params_;

    // 构建线性化预测模型
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
    linearize(const Eigen::Vector4d& state, double v_ref, double delta_ref);

    // 构建 QP 问题并求解
    std::pair<double, double> solveQP(
        const Eigen::Vector4d& state,
        const std::vector<Eigen::MatrixXd>& A_list,
        const std::vector<Eigen::MatrixXd>& B_list,
        const std::vector<Eigen::Vector4d>& ref_states,
        const Eigen::Vector2d& u_prev);

    int findNearestIndex(const Eigen::Vector2d& pos,
                         const std::vector<Waypoint>& traj);

    Eigen::Vector2d u_prev_ = Eigen::Vector2d::Zero();
    static double normalizeAngle(double angle);
};
```

创建 `src/mpc_controller.cpp`：

```cpp
#include "mpc_controller.hpp"
#include <cmath>
#include <limits>

MPCController::MPCController(const Params& params) : params_(params) {}

std::pair<double, double> MPCController::compute(
    const Eigen::Vector4d& state,
    const std::vector<MPCController::Waypoint>& trajectory)
{
    int nearest = findNearestIndex(Eigen::Vector2d(state[0], state[1]), trajectory);

    // 构建未来 N 步的参考状态和线性化模型
    std::vector<Eigen::MatrixXd> A_list, B_list;
    std::vector<Eigen::Vector4d> ref_states;

    for (int k = 0; k < params_.N; ++k) {
        int idx = std::min(nearest + k, (int)trajectory.size() - 1);
        Eigen::Vector4d ref(
            trajectory[idx].x, trajectory[idx].y,
            trajectory[idx].yaw, trajectory[idx].v_ref);
        ref_states.push_back(ref);

        auto [A, B] = linearize(ref, trajectory[idx].v_ref, 0.0);
        A_list.push_back(A);
        B_list.push_back(B);
    }

    auto [delta, accel] = solveQP(state, A_list, B_list, ref_states, u_prev_);
    u_prev_ << delta, accel;
    return {delta, accel};
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
MPCController::linearize(const Eigen::Vector4d& state, double v_ref, double delta_ref)
{
    // 在参考点处线性化自行车模型
    // 状态量: [x, y, θ, v]，控制量: [δ, a]
    double theta = state[2];
    double dt    = params_.dt;
    double L     = params_.wheelbase;

    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    A(0, 2) = -v_ref * std::sin(theta) * dt;
    A(0, 3) =  std::cos(theta) * dt;
    A(1, 2) =  v_ref * std::cos(theta) * dt;
    A(1, 3) =  std::sin(theta) * dt;
    A(2, 3) =  std::tan(delta_ref) / L * dt;

    Eigen::MatrixXd B(4, 2);
    B.setZero();
    B(2, 0) = v_ref / (L * std::cos(delta_ref) * std::cos(delta_ref)) * dt;
    B(3, 1) = dt;

    return {A, B};
}

std::pair<double, double> MPCController::solveQP(
    const Eigen::Vector4d& state,
    const std::vector<Eigen::MatrixXd>& A_list,
    const std::vector<Eigen::MatrixXd>& B_list,
    const std::vector<Eigen::Vector4d>& ref_states,
    const Eigen::Vector2d& u_prev)
{
    const int nx = 4;   // 状态维数
    const int nu = 2;   // 控制维数
    const int N  = params_.N;

    // 构建稀疏 QP：min 0.5 z^T H z + f^T z
    // 决策变量 z = [x_1,...,x_N, u_0,...,u_{N-1}]

    // 权重矩阵
    Eigen::DiagonalMatrix<double,4> Q(params_.Q_diag);
    Eigen::DiagonalMatrix<double,4> Qf(params_.Qf_diag);
    Eigen::DiagonalMatrix<double,2> R(params_.R_diag);
    Eigen::DiagonalMatrix<double,2> Rd(params_.Rd_diag);

    int n_vars = nx * N + nu * N;  // 总决策变量数
    Eigen::SparseMatrix<double> H(n_vars, n_vars);
    Eigen::VectorXd f = Eigen::VectorXd::Zero(n_vars);

    // 填充 H 和 f（代价函数）
    std::vector<Eigen::Triplet<double>> H_triplets;
    for (int k = 0; k < N; ++k) {
        int x_offset = k * nx;
        // 状态代价
        const auto& Qk = (k == N-1) ? Qf : Q;
        for (int i = 0; i < nx; ++i)
            H_triplets.push_back({x_offset+i, x_offset+i, Qk.diagonal()[i]});

        // 线性项（参考状态）
        for (int i = 0; i < nx; ++i)
            f(x_offset+i) = -Qk.diagonal()[i] * ref_states[k][i];

        // 控制代价
        int u_offset = nx * N + k * nu;
        for (int i = 0; i < nu; ++i)
            H_triplets.push_back({u_offset+i, u_offset+i, R.diagonal()[i]});

        // 控制变化量代价（相邻步之差）
        if (k > 0) {
            int u_prev_offset = nx * N + (k-1) * nu;
            for (int i = 0; i < nu; ++i) {
                H_triplets.push_back({u_offset+i, u_offset+i, Rd.diagonal()[i]});
                H_triplets.push_back({u_prev_offset+i, u_prev_offset+i, Rd.diagonal()[i]});
            }
        }
    }
    H.setFromTriplets(H_triplets.begin(), H_triplets.end());

    // 等式约束（动态方程）
    int n_eq = nx * N;
    Eigen::SparseMatrix<double> A_eq(n_eq, n_vars);
    Eigen::VectorXd b_eq = Eigen::VectorXd::Zero(n_eq);
    std::vector<Eigen::Triplet<double>> Aeq_triplets;

    for (int k = 0; k < N; ++k) {
        int row = k * nx;
        // x_{k+1} 的系数
        for (int i = 0; i < nx; ++i)
            Aeq_triplets.push_back({row+i, k*nx+i, -1.0});

        if (k == 0) {
            // x_1 = A_0 x_0 + B_0 u_0
            b_eq.segment(row, nx) = -(A_list[k] * state);
        } else {
            // x_{k+1} = A_k x_k + B_k u_k
            for (int i = 0; i < nx; ++i)
                for (int j = 0; j < nx; ++j)
                    Aeq_triplets.push_back({row+i, (k-1)*nx+j, A_list[k](i,j)});
        }
        // B_k u_k 的系数
        for (int i = 0; i < nx; ++i)
            for (int j = 0; j < nu; ++j)
                Aeq_triplets.push_back({row+i, nx*N+k*nu+j, B_list[k](i,j)});
    }
    A_eq.setFromTriplets(Aeq_triplets.begin(), Aeq_triplets.end());

    // 不等式约束（控制量约束）
    int n_ineq = nu * N;
    Eigen::SparseMatrix<double> A_ineq(n_ineq, n_vars);
    Eigen::VectorXd lb = Eigen::VectorXd::Zero(n_ineq);
    Eigen::VectorXd ub = Eigen::VectorXd::Zero(n_ineq);
    std::vector<Eigen::Triplet<double>> Aineq_triplets;

    for (int k = 0; k < N; ++k) {
        int u_offset = nx * N + k * nu;
        Aineq_triplets.push_back({k*2,   u_offset,   1.0});
        Aineq_triplets.push_back({k*2+1, u_offset+1, 1.0});
        lb(k*2)   = -params_.max_steer;
        ub(k*2)   =  params_.max_steer;
        lb(k*2+1) = -params_.max_accel;
        ub(k*2+1) =  params_.max_accel;
    }
    A_ineq.setFromTriplets(Aineq_triplets.begin(), Aineq_triplets.end());

    // 合并约束
    Eigen::SparseMatrix<double> A_full(n_eq + n_ineq, n_vars);
    // （此处省略合并代码，实际中用 OsqpEigen 的接口）

    // 用 OsqpEigen 求解
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);
    solver.settings()->setMaxIteration(1000);
    solver.settings()->setAbsoluteTolerance(1e-4);

    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_eq + n_ineq);
    if (!solver.data()->setHessianMatrix(H)) return {0.0, 0.0};
    if (!solver.data()->setGradient(f)) return {0.0, 0.0};
    // 省略完整约束设置...

    if (!solver.initSolver()) return {0.0, 0.0};
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        return {0.0, 0.0};

    Eigen::VectorXd sol = solver.getSolution();
    // 提取第一步控制量
    double delta = sol(nx * N);
    double accel = sol(nx * N + 1);

    return {delta, accel};
}

int MPCController::findNearestIndex(
    const Eigen::Vector2d& pos, const std::vector<Waypoint>& traj)
{
    double min_dist = std::numeric_limits<double>::max();
    int idx = 0;
    for (int i = 0; i < (int)traj.size(); ++i) {
        double d = std::hypot(traj[i].x - pos[0], traj[i].y - pos[1]);
        if (d < min_dist) { min_dist = d; idx = i; }
    }
    return idx;
}

double MPCController::normalizeAngle(double angle) {
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
```

---

## 第十七周：三个测试场景

### 17.1 场景 1：直线加减速（纵向测试）

**目的：** 验证 PID 纵向控制精度

```
轨迹：x=0~100m 直线
速度序列：0→8→4→8 m/s（变速行驶）
测量指标：速度跟踪 RMSE
```

**预期结果：**
- PID RMSE < 0.3 m/s
- 变速响应时间 < 2s

---

### 17.2 场景 2：S 形弯道（横向跟踪测试）

**目的：** 对比 Pure Pursuit / Stanley / LQR 横向精度

```
轨迹：振幅 10m，周期 100m 的正弦曲线
速度：匀速 6 m/s
测量指标：横向误差 RMSE、最大误差
```

**预期结果（数据仅供参考，实际调参后获得）：**

| 控制器 | RMSE (m) | 最大误差 (m) |
|--------|----------|-------------|
| Pure Pursuit | 0.3~0.5 | 0.8~1.2 |
| Stanley | 0.2~0.4 | 0.6~1.0 |
| LQR | 0.1~0.3 | 0.4~0.8 |
| MPC | 0.05~0.15 | 0.2~0.5 |

---

### 17.3 场景 3：ISO 3888 双移线（核心场景）

**ISO 3888 标准双移线参数：**

```
第一段：直线进入，30m
第二段：向左偏移 3.5m，过渡段 20m
第三段：直线，30m
第四段：向右回位 3.5m，过渡段 20m
第五段：直线退出，30m
测试速度：8 m/s
```

**为什么这个场景最有说服力：**
- 包含急速横向切换，最能体现 MPC 的预测优势
- 是真实道路换道测试的简化版
- 这个场景能直观说明控制器对实际工况的适用性

---

## 第十八周：参数调优

### 18.1 MPC 参数调优方法

**预测步数 N：**
```
N 太小 → 看不够远，高速弯道超调
N 太大 → 计算太慢，实时性差
建议：从 N=5 开始，逐步增加到 N=10~15
```

**权重矩阵 Q 和 R：**
```
Q 大、R 小 → 跟踪精度高，但控制激进，可能抖动
Q 小、R 大 → 控制平滑，但跟踪精度低
调优策略：先把 R 设为 1，调 Q 到误差满意；再调 R 减小抖动
```

**调优记录表（建议填写）：**

| N | Q(e_y) | Q(e_θ) | R(δ) | R(a) | S弯RMSE | 双移线RMSE | 备注 |
|---|--------|--------|------|------|---------|-----------|------|
| 5 | 10 | 1 | 0.1 | 0.1 | ? | ? | 初始值 |
| 10 | 10 | 1 | 0.1 | 0.1 | ? | ? | 增加预测步 |
| 10 | 20 | 2 | 0.1 | 0.1 | ? | ? | 增大状态权重 |

---

## 第四阶段总结

完成本阶段后，你应该拥有：

| 产出 | 验证方式 |
|------|----------|
| LQR 控制器（C++） | S 弯 RMSE 低于 Pure Pursuit |
| MPC 控制器（C++） | 双移线 RMSE 最低 |
| 三个测试场景数据 | CSV 文件 + 对比图 |
| 参数调优记录 | 填写调优记录表 |
| CARLA+ROS2 中 MPC 跑通 | 目视 + RMSE 数据 |

**MPC 实现难点和解决办法：**
- OSQP 矩阵构建繁琐 → 先用 Python scipy.optimize 验证逻辑，再移植 C++
- 求解器不收敛 → 检查约束是否可行，先去掉约束调通再加入
- 实时性不足 → 减小 N，或用热启动（warm start）加速
- 线性化误差大 → 减小 dt，或在每步重新线性化
