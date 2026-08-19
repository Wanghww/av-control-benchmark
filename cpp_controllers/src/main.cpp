#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <cmath>
#include <filesystem>
#include "bicycle_model.hpp"
#include "dynamic_bicycle_model.hpp"
#include "pid_controller.hpp"
#include "pure_pursuit.hpp"
#include "stanley_controller.hpp"
#include "lqr_controller.hpp"
#include "mpc_controller.hpp"

using Waypoint = PurePursuitController::Waypoint;

std::vector<Waypoint> generateSCurve() {
    std::vector<Waypoint> traj;
    const int    N      = 300;
    const double length = 100.0;
    for (int i = 0; i < N; ++i) {
        double x   = length * i / N;
        double y   = 10.0 * std::sin(2.0 * M_PI * x / length);
        double dx  = length / N;
        double dy  = 10.0 * std::cos(2.0 * M_PI * x / length) * (2.0 * M_PI / length) * dx;
        double yaw = std::atan2(dy, dx);
        traj.push_back({x, y, yaw, 6.0});
    }
    return traj;
}

// LQR 需要参考曲率做前馈补偿，S 曲线曲率可解析求出：kappa = y'' / (1+y'^2)^1.5
std::vector<LQRController::Waypoint> generateSCurveLQR() {
    std::vector<LQRController::Waypoint> traj;
    const int    N      = 300;
    const double length = 100.0;
    const double amp    = 10.0;
    const double omega  = 2.0 * M_PI / length;
    for (int i = 0; i < N; ++i) {
        double x     = length * i / N;
        double y     = amp * std::sin(omega * x);
        double dy    = amp * omega * std::cos(omega * x);
        double d2y   = -amp * omega * omega * std::sin(omega * x);
        double yaw   = std::atan2(dy, 1.0);
        double kappa = d2y / std::pow(1.0 + dy * dy, 1.5);
        traj.push_back({x, y, yaw, 6.0, kappa});
    }
    return traj;
}

// ISO 3888 双移线轨迹（带曲率）
// 第一段直线30m → 向左偏移3.5m（余弦过渡20m）→ 直线30m → 向右回位3.5m（余弦过渡20m）→ 直线30m
std::vector<LQRController::Waypoint> generateDoubleLaneChange(double speed = 8.0) {
    std::vector<LQRController::Waypoint> traj;
    const int N = 600;
    const double total = 130.0;
    const double offset = 3.5;
    const double L_trans = 20.0;

    for (int i = 0; i < N; ++i) {
        double x = total * i / (N - 1);
        double y, dy, d2y;

        if (x < 30.0) {
            y = 0.0; dy = 0.0; d2y = 0.0;
        } else if (x < 50.0) {
            double s = x - 30.0;
            y   = offset / 2.0 * (1.0 - std::cos(M_PI * s / L_trans));
            dy  = offset * M_PI / (2.0 * L_trans) * std::sin(M_PI * s / L_trans);
            d2y = offset * M_PI * M_PI / (2.0 * L_trans * L_trans) * std::cos(M_PI * s / L_trans);
        } else if (x < 80.0) {
            y = offset; dy = 0.0; d2y = 0.0;
        } else if (x < 100.0) {
            double s = x - 80.0;
            y   = offset / 2.0 * (1.0 + std::cos(M_PI * s / L_trans));
            dy  = -offset * M_PI / (2.0 * L_trans) * std::sin(M_PI * s / L_trans);
            d2y = -offset * M_PI * M_PI / (2.0 * L_trans * L_trans) * std::cos(M_PI * s / L_trans);
        } else {
            y = 0.0; dy = 0.0; d2y = 0.0;
        }

        double yaw   = std::atan2(dy, 1.0);
        double kappa = d2y / std::pow(1.0 + dy * dy, 1.5);
        traj.push_back({x, y, yaw, speed, kappa});
    }
    return traj;
}

int main() {
    std::filesystem::create_directories("results");

    // ---- Pure Pursuit ----
    {
        auto traj = generateSCurve();
        BicycleModel       model;
        PIDController      pid(1.5, 0.1, 0.05);
        PurePursuitController pp;

        BicycleModel::State state;
        state << traj[0].x, traj[0].y, traj[0].yaw, 0.0;

        std::ofstream csv("results/simulation_output.csv");
        csv << "step,x,y,theta,v,lateral_error\n";

        double total_sq = 0.0;
        int    nearest_idx = 0;
        const  int STEPS = 1000;

        for (int step = 0; step < STEPS; ++step) {
            auto [delta, v_ref] = pp.compute(state, traj, nearest_idx);
            nearest_idx = pp.lastNearestIdx();

            double accel = pid.compute(v_ref, state[3]);
            state = model.update(state, delta, accel);

            double min_dist = 1e9;
            for (auto& wp : traj) {
                double d = std::hypot(wp.x - state[0], wp.y - state[1]);
                if (d < min_dist) min_dist = d;
            }
            total_sq += min_dist * min_dist;

            csv << step << "," << state[0] << "," << state[1] << ","
                << state[2] << "," << state[3] << "," << min_dist << "\n";

            if (nearest_idx >= (int)traj.size() - 10) break;
        }

        double rmse = std::sqrt(total_sq / STEPS);
        std::cout << "Pure Pursuit RMSE: " << rmse << " m\n";
    }

    // ---- LQR ----
    {
        auto traj = generateSCurveLQR();
        BicycleModel  model;
        PIDController pid(1.5, 0.1, 0.05);
        LQRController lqr;

        BicycleModel::State state;
        state << traj[0].x, traj[0].y, traj[0].yaw, 0.0;

        std::ofstream csv("results/lqr_output.csv");
        csv << "step,x,y,theta,v,lateral_error\n";

        double total_sq = 0.0;
        int    nearest_idx = 0;
        const  int STEPS = 1000;

        for (int step = 0; step < STEPS; ++step) {
            auto [delta, v_ref] = lqr.compute(state, traj, nearest_idx);
            nearest_idx = lqr.lastNearestIdx();

            double accel = pid.compute(v_ref, state[3]);
            state = model.update(state, delta, accel);

            double min_dist = 1e9;
            for (auto& wp : traj) {
                double d = std::hypot(wp.x - state[0], wp.y - state[1]);
                if (d < min_dist) min_dist = d;
            }
            total_sq += min_dist * min_dist;

            csv << step << "," << state[0] << "," << state[1] << ","
                << state[2] << "," << state[3] << "," << min_dist << "\n";

            if (nearest_idx >= (int)traj.size() - 10) break;
        }

        double rmse = std::sqrt(total_sq / STEPS);
        std::cout << "LQR RMSE: " << rmse << " m\n";
    }

    // ---- MPC ----
    {
        // LQRController::Waypoint 和 MPCController::Waypoint 结构相同，做一次类型转换
        auto lqr_traj = generateSCurveLQR();
        std::vector<MPCController::Waypoint> traj;
        traj.reserve(lqr_traj.size());
        for (auto& wp : lqr_traj)
            traj.push_back({wp.x, wp.y, wp.yaw, wp.v_ref, wp.curvature});
        BicycleModel  model;
        PIDController pid(1.5, 0.1, 0.05);
        MPCController mpc;

        BicycleModel::State state;
        state << traj[0].x, traj[0].y, traj[0].yaw, 0.0;

        std::ofstream csv("results/mpc_output.csv");
        csv << "step,x,y,theta,v,lateral_error\n";

        double total_sq = 0.0;
        int    nearest_idx = 0;
        const  int STEPS = 1000;

        for (int step = 0; step < STEPS; ++step) {
            auto [delta, v_ref] = mpc.compute(state, traj, nearest_idx);
            nearest_idx = mpc.lastNearestIdx();

            double accel = pid.compute(v_ref, state[3]);
            state = model.update(state, delta, accel);

            double min_dist = 1e9;
            for (auto& wp : traj) {
                double d = std::hypot(wp.x - state[0], wp.y - state[1]);
                if (d < min_dist) min_dist = d;
            }
            total_sq += min_dist * min_dist;

            csv << step << "," << state[0] << "," << state[1] << ","
                << state[2] << "," << state[3] << "," << min_dist << "\n";

            if (nearest_idx >= (int)traj.size() - 10) break;
        }

        double rmse = std::sqrt(total_sq / STEPS);
        std::cout << "MPC    RMSE: " << rmse << " m\n";
    }

    // ========== 双移线场景（ISO 3888） ==========
    std::cout << "\n--- ISO 3888 双移线 (8 m/s) ---\n";

    // 生成双移线轨迹（带曲率，供 LQR/MPC 使用）
    auto dlc_traj_lqr = generateDoubleLaneChange();

    // 转为无曲率格式（供 Pure Pursuit / Stanley 使用）
    std::vector<Waypoint> dlc_traj_pp;
    std::vector<StanleyController::Waypoint> dlc_traj_stanley;
    std::vector<MPCController::Waypoint> dlc_traj_mpc;
    for (auto& wp : dlc_traj_lqr) {
        dlc_traj_pp.push_back({wp.x, wp.y, wp.yaw, wp.v_ref});
        dlc_traj_stanley.push_back({wp.x, wp.y, wp.yaw, wp.v_ref});
        dlc_traj_mpc.push_back({wp.x, wp.y, wp.yaw, wp.v_ref, wp.curvature});
    }

    // ---- 双移线：Pure Pursuit ----
    {
        BicycleModel model;
        PIDController pid(1.5, 0.1, 0.05);
        PurePursuitController pp(2.7, 0.1, 3.0);

        BicycleModel::State state;
        state << dlc_traj_pp[0].x, dlc_traj_pp[0].y, dlc_traj_pp[0].yaw, 0.0;

        std::ofstream csv("results/dlc_pp_output.csv");
        csv << "step,x,y,theta,v,lateral_error\n";

        double total_sq = 0.0;
        int nearest_idx = 0;
        const int STEPS = 2000;

        for (int step = 0; step < STEPS; ++step) {
            auto [delta, v_ref] = pp.compute(state, dlc_traj_pp, nearest_idx);
            nearest_idx = pp.lastNearestIdx();
            double accel = pid.compute(v_ref, state[3]);
            state = model.update(state, delta, accel);

            double min_dist = 1e9;
            for (auto& wp : dlc_traj_pp) {
                double d = std::hypot(wp.x - state[0], wp.y - state[1]);
                if (d < min_dist) min_dist = d;
            }
            total_sq += min_dist * min_dist;
            csv << step << "," << state[0] << "," << state[1] << ","
                << state[2] << "," << state[3] << "," << min_dist << "\n";
            if (nearest_idx >= (int)dlc_traj_pp.size() - 10) break;
        }
        std::cout << "Pure Pursuit RMSE: " << std::sqrt(total_sq / STEPS) << " m\n";
    }

    // ---- 双移线：Stanley ----
    {
        BicycleModel model;
        PIDController pid(1.5, 0.1, 0.05);
        StanleyController stanley(2.7, 0.5, 1.0);

        BicycleModel::State state;
        state << dlc_traj_stanley[0].x, dlc_traj_stanley[0].y, dlc_traj_stanley[0].yaw, 0.0;

        std::ofstream csv("results/dlc_stanley_output.csv");
        csv << "step,x,y,theta,v,lateral_error\n";

        double total_sq = 0.0;
        int nearest_idx = 0;
        const int STEPS = 2000;

        for (int step = 0; step < STEPS; ++step) {
            auto [delta, v_ref] = stanley.compute(state, dlc_traj_stanley, nearest_idx);
            nearest_idx = stanley.lastNearestIdx();
            double accel = pid.compute(v_ref, state[3]);
            state = model.update(state, delta, accel);

            double min_dist = 1e9;
            for (auto& wp : dlc_traj_stanley) {
                double d = std::hypot(wp.x - state[0], wp.y - state[1]);
                if (d < min_dist) min_dist = d;
            }
            total_sq += min_dist * min_dist;
            csv << step << "," << state[0] << "," << state[1] << ","
                << state[2] << "," << state[3] << "," << min_dist << "\n";
            if (nearest_idx >= (int)dlc_traj_stanley.size() - 10) break;
        }
        std::cout << "Stanley      RMSE: " << std::sqrt(total_sq / STEPS) << " m\n";
    }

    // ---- 双移线：LQR ----
    {
        BicycleModel model;
        PIDController pid(1.5, 0.1, 0.05);
        LQRController lqr;

        BicycleModel::State state;
        state << dlc_traj_lqr[0].x, dlc_traj_lqr[0].y, dlc_traj_lqr[0].yaw, 0.0;

        std::ofstream csv("results/dlc_lqr_output.csv");
        csv << "step,x,y,theta,v,lateral_error\n";

        double total_sq = 0.0;
        int nearest_idx = 0;
        const int STEPS = 2000;

        for (int step = 0; step < STEPS; ++step) {
            auto [delta, v_ref] = lqr.compute(state, dlc_traj_lqr, nearest_idx);
            nearest_idx = lqr.lastNearestIdx();
            double accel = pid.compute(v_ref, state[3]);
            state = model.update(state, delta, accel);

            double min_dist = 1e9;
            for (auto& wp : dlc_traj_lqr) {
                double d = std::hypot(wp.x - state[0], wp.y - state[1]);
                if (d < min_dist) min_dist = d;
            }
            total_sq += min_dist * min_dist;
            csv << step << "," << state[0] << "," << state[1] << ","
                << state[2] << "," << state[3] << "," << min_dist << "\n";
            if (nearest_idx >= (int)dlc_traj_lqr.size() - 10) break;
        }
        std::cout << "LQR          RMSE: " << std::sqrt(total_sq / STEPS) << " m\n";
    }

    // ---- 双移线：MPC ----
    {
        BicycleModel model;
        PIDController pid(1.5, 0.1, 0.05);
        MPCController mpc;

        BicycleModel::State state;
        state << dlc_traj_mpc[0].x, dlc_traj_mpc[0].y, dlc_traj_mpc[0].yaw, 0.0;

        std::ofstream csv("results/dlc_mpc_output.csv");
        csv << "step,x,y,theta,v,lateral_error\n";

        double total_sq = 0.0;
        int nearest_idx = 0;
        const int STEPS = 2000;

        for (int step = 0; step < STEPS; ++step) {
            auto [delta, v_ref] = mpc.compute(state, dlc_traj_mpc, nearest_idx);
            nearest_idx = mpc.lastNearestIdx();
            double accel = pid.compute(v_ref, state[3]);
            state = model.update(state, delta, accel);

            double min_dist = 1e9;
            for (auto& wp : dlc_traj_mpc) {
                double d = std::hypot(wp.x - state[0], wp.y - state[1]);
                if (d < min_dist) min_dist = d;
            }
            total_sq += min_dist * min_dist;
            csv << step << "," << state[0] << "," << state[1] << ","
                << state[2] << "," << state[3] << "," << min_dist << "\n";
            if (nearest_idx >= (int)dlc_traj_mpc.size() - 10) break;
        }
        std::cout << "MPC          RMSE: " << std::sqrt(total_sq / STEPS) << " m\n";
    }

    // ========== 速度扫描（Speed Sweep） ==========
    std::cout << "\n--- Speed Sweep: 双移线 RMSE vs 速度 ---\n";
    std::cout << "速度(m/s)  PurePursuit  Stanley      LQR          MPC\n";

    std::ofstream sweep_csv("results/speed_sweep.csv");
    sweep_csv << "speed,pure_pursuit,stanley,lqr,mpc\n";

    for (double spd : {4.0, 6.0, 8.0, 10.0, 12.0, 14.0}) {
        auto traj_lqr = generateDoubleLaneChange(spd);
        std::vector<Waypoint> traj_pp;
        std::vector<StanleyController::Waypoint> traj_st;
        std::vector<MPCController::Waypoint> traj_mpc;
        for (auto& wp : traj_lqr) {
            traj_pp.push_back({wp.x, wp.y, wp.yaw, wp.v_ref});
            traj_st.push_back({wp.x, wp.y, wp.yaw, wp.v_ref});
            traj_mpc.push_back({wp.x, wp.y, wp.yaw, wp.v_ref, wp.curvature});
        }

        auto runCtrl = [&](auto& ctrl, auto& traj) -> double {
            BicycleModel model;
            PIDController pid(1.5, 0.1, 0.05);
            BicycleModel::State state;
            state << traj[0].x, traj[0].y, traj[0].yaw, 0.0;
            double sq = 0.0;
            int nearest = 0;
            int actual_steps = 0;
            const int STEPS = 2000;
            for (int step = 0; step < STEPS; ++step) {
                auto [delta, v_ref] = ctrl.compute(state, traj, nearest);
                nearest = ctrl.lastNearestIdx();
                state = model.update(state, delta, pid.compute(v_ref, state[3]));
                double md = 1e9;
                for (auto& wp : traj) {
                    double d = std::hypot(wp.x - state[0], wp.y - state[1]);
                    if (d < md) md = d;
                }
                sq += md * md;
                actual_steps++;
                if (nearest >= (int)traj.size() - 10) break;
            }
            return std::sqrt(sq / actual_steps);  // 实际步数，不是固定2000
        };

        PurePursuitController pp(2.7, 0.1, 3.0);
        StanleyController     stanley(2.7, 0.5, 1.0);
        LQRController         lqr;
        MPCController         mpc;

        double r_pp  = runCtrl(pp,      traj_pp);
        double r_st  = runCtrl(stanley, traj_st);
        double r_lqr = runCtrl(lqr,     traj_lqr);
        double r_mpc = runCtrl(mpc,      traj_mpc);

        std::cout << std::fixed << std::setprecision(1) << spd << "        "
                  << std::setprecision(4)
                  << r_pp << "       " << r_st << "       "
                  << r_lqr << "       " << r_mpc << "\n";
        sweep_csv << spd << "," << r_pp << "," << r_st << ","
                  << r_lqr << "," << r_mpc << "\n";
    }

    // ========== 计算耗时测试 ==========
    std::cout << "\n--- 计算耗时 (1000次调用均值) ---\n";

    // 使用双移线中段状态作为测试点
    auto time_traj_lqr = generateDoubleLaneChange(8.0);
    std::vector<Waypoint> time_traj_pp;
    std::vector<StanleyController::Waypoint> time_traj_st;
    std::vector<MPCController::Waypoint> time_traj_mpc;
    for (auto& wp : time_traj_lqr) {
        time_traj_pp.push_back({wp.x, wp.y, wp.yaw, wp.v_ref});
        time_traj_st.push_back({wp.x, wp.y, wp.yaw, wp.v_ref});
        time_traj_mpc.push_back({wp.x, wp.y, wp.yaw, wp.v_ref, wp.curvature});
    }
    Eigen::Vector4d test_state(65.0, 1.75, 0.0, 8.0);  // 双移线中段
    const int N_TIMING = 1000;

    auto measureTime = [&](auto& ctrl, auto& traj) -> double {
        int nearest = 300;
        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < N_TIMING; ++i)
            ctrl.compute(test_state, traj, nearest);
        auto t1 = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double, std::micro>(t1 - t0).count() / N_TIMING;
    };

    PurePursuitController t_pp(2.7, 0.1, 3.0);
    StanleyController     t_st(2.7, 0.5, 1.0);
    LQRController         t_lqr;
    MPCController         t_mpc;

    double us_pp  = measureTime(t_pp,  time_traj_pp);
    double us_st  = measureTime(t_st,  time_traj_st);
    double us_lqr = measureTime(t_lqr, time_traj_lqr);
    double us_mpc = measureTime(t_mpc, time_traj_mpc);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Pure Pursuit : " << us_pp  << " μs\n";
    std::cout << "Stanley      : " << us_st  << " μs\n";
    std::cout << "LQR          : " << us_lqr << " μs\n";
    std::cout << "MPC          : " << us_mpc << " μs\n";

    std::ofstream timing_csv("results/timing.csv");
    timing_csv << "controller,mean_us\n";
    timing_csv << "Pure Pursuit," << us_pp  << "\n";
    timing_csv << "Stanley,"      << us_st  << "\n";
    timing_csv << "LQR,"          << us_lqr << "\n";
    timing_csv << "MPC,"          << us_mpc << "\n";

    // ========== 第六阶段：动态自行车模型速度扫描 ==========
    // 使用 DynamicBicycleModel 作为物理仿真器，替换运动学模型
    // 对比：
    //   PP_kin   — Pure Pursuit，控制器仍用运动学假设，仿真器升级为动态
    //   LQR_kin  — LQR，运动学线性化，动态仿真器
    //   LQR_dyn  — LQR，动态线性化（B 向量用 Cf*lf/Iz），动态仿真器
    //   MPC_kin  — MPC，运动学线性化，动态仿真器
    std::cout << "\n===== 第六阶段：动态模型速度扫描 =====\n";
    std::cout << "速度(m/s)  PP_kin   LQR_kin  LQR_dyn  MPC_kin\n";

    std::ofstream dyn_csv("results/dynamic_speed_sweep.csv");
    dyn_csv << "speed,pp_kin,lqr_kin,lqr_dyn,mpc_kin\n";

    DynamicBicycleModel dyn_model;
    // 动态模型线性化系数：横摆力矩对转角的响应 Bc[1] = Cf*lf/Iz
    const double Bc1 = dyn_model.params().Cf * dyn_model.params().lf
                       / dyn_model.params().Iz;

    // 辅助 lambda：用动态模型仿真，控制器使用标准运动学 compute()
    auto runDynCtrlKin = [&](auto& ctrl, auto& traj, double spd) -> double {
        DynamicBicycleModel sim;
        PIDController pid(1.5, 0.1, 0.05);
        DynamicBicycleModel::State state;
        state.x = traj[0].x;  state.y = traj[0].y;
        state.theta = traj[0].yaw;  state.v = 0.0;

        double sq = 0.0;
        int nearest = 0, actual_steps = 0;
        const int STEPS = 2000;

        for (int step = 0; step < STEPS; ++step) {
            // 控制器只看 [x, y, theta, v]，不知道 beta/r
            Eigen::Vector4d ks = DynamicBicycleModel::toKinematicState(state);
            auto [delta, v_ref] = ctrl.compute(ks, traj, nearest);
            nearest = ctrl.lastNearestIdx();
            state = sim.update(state, delta, pid.compute(v_ref, state.v));

            double md = 1e9;
            for (auto& wp : traj) {
                double d = std::hypot(wp.x - state.x, wp.y - state.y);
                if (d < md) md = d;
            }
            sq += md * md;
            actual_steps++;
            if (nearest >= (int)traj.size() - 10) break;
        }
        return std::sqrt(sq / actual_steps);
    };

    // LQR 动态适配版 lambda：B 向量使用动态线性化
    auto runDynLQR = [&](auto& traj, double /*spd*/) -> double {
        DynamicBicycleModel sim;
        PIDController pid(1.5, 0.1, 0.05);
        LQRController lqr_d;
        DynamicBicycleModel::State state;
        state.x = traj[0].x;  state.y = traj[0].y;
        state.theta = traj[0].yaw;  state.v = 0.0;

        double sq = 0.0;
        int nearest = 0, actual_steps = 0;
        const int STEPS = 2000;

        for (int step = 0; step < STEPS; ++step) {
            Eigen::Vector4d ks = DynamicBicycleModel::toKinematicState(state);
            // computeDynamic 使用动态横摆响应系数，而非运动学 v/L
            auto [delta, v_ref] = lqr_d.computeDynamic(ks, traj, Bc1, nearest);
            nearest = lqr_d.lastNearestIdx();
            state = sim.update(state, delta, pid.compute(v_ref, state.v));

            double md = 1e9;
            for (auto& wp : traj) {
                double d = std::hypot(wp.x - state.x, wp.y - state.y);
                if (d < md) md = d;
            }
            sq += md * md;
            actual_steps++;
            if (nearest >= (int)traj.size() - 10) break;
        }
        return std::sqrt(sq / actual_steps);
    };

    for (double spd : {4.0, 8.0, 12.0, 15.0, 18.0}) {
        auto traj_lqr_d = generateDoubleLaneChange(spd);
        std::vector<Waypoint> traj_pp_d;
        std::vector<MPCController::Waypoint> traj_mpc_d;
        for (auto& wp : traj_lqr_d) {
            traj_pp_d.push_back({wp.x, wp.y, wp.yaw, wp.v_ref});
            traj_mpc_d.push_back({wp.x, wp.y, wp.yaw, wp.v_ref, wp.curvature});
        }

        PurePursuitController pp_d(2.7, 0.1, 3.0);
        LQRController         lqr_k;
        MPCController         mpc_d;

        double r_pp  = runDynCtrlKin(pp_d,   traj_pp_d,  spd);
        double r_lk  = runDynCtrlKin(lqr_k,  traj_lqr_d, spd);
        double r_ld  = runDynLQR(traj_lqr_d, spd);
        double r_mk  = runDynCtrlKin(mpc_d,  traj_mpc_d, spd);

        std::cout << std::fixed << std::setprecision(1) << spd << "        "
                  << std::setprecision(4)
                  << r_pp << "   " << r_lk << "   "
                  << r_ld << "   " << r_mk << "\n";
        dyn_csv << spd << "," << r_pp << "," << r_lk << ","
                << r_ld << "," << r_mk << "\n";
    }

    // 15 m/s 重点场景：输出轨迹文件供可视化
    std::cout << "\n--- 15 m/s 双移线轨迹文件已写入 results/ ---\n";
    {
        auto traj15 = generateDoubleLaneChange(15.0);
        std::vector<Waypoint> traj15_pp;
        std::vector<MPCController::Waypoint> traj15_mpc;
        for (auto& wp : traj15) {
            traj15_pp.push_back({wp.x, wp.y, wp.yaw, wp.v_ref});
            traj15_mpc.push_back({wp.x, wp.y, wp.yaw, wp.v_ref, wp.curvature});
        }

        // PP_kin @ 15 m/s
        {
            DynamicBicycleModel sim;
            PIDController pid(1.5, 0.1, 0.05);
            PurePursuitController pp_15(2.7, 0.1, 3.0);
            DynamicBicycleModel::State state;
            state.x = traj15[0].x; state.y = traj15[0].y;
            state.theta = traj15[0].yaw; state.v = 0.0;

            std::ofstream f("results/dyn15_pp.csv");
            f << "step,x,y,theta,v,lateral_error\n";
            int nearest = 0;
            for (int step = 0; step < 2000; ++step) {
                Eigen::Vector4d ks = DynamicBicycleModel::toKinematicState(state);
                auto [delta, v_ref] = pp_15.compute(ks, traj15_pp, nearest);
                nearest = pp_15.lastNearestIdx();
                state = sim.update(state, delta, pid.compute(v_ref, state.v));
                double md = 1e9;
                for (auto& wp : traj15_pp) {
                    double d = std::hypot(wp.x - state.x, wp.y - state.y);
                    if (d < md) md = d;
                }
                f << step << "," << state.x << "," << state.y << ","
                  << state.theta << "," << state.v << "," << md << "\n";
                if (nearest >= (int)traj15.size() - 10) break;
            }
        }

        // LQR_dyn @ 15 m/s
        {
            DynamicBicycleModel sim;
            PIDController pid(1.5, 0.1, 0.05);
            LQRController lqr15;
            DynamicBicycleModel::State state;
            state.x = traj15[0].x; state.y = traj15[0].y;
            state.theta = traj15[0].yaw; state.v = 0.0;

            std::ofstream f("results/dyn15_lqr.csv");
            f << "step,x,y,theta,v,lateral_error\n";
            int nearest = 0;
            for (int step = 0; step < 2000; ++step) {
                Eigen::Vector4d ks = DynamicBicycleModel::toKinematicState(state);
                auto [delta, v_ref] = lqr15.computeDynamic(ks, traj15, Bc1, nearest);
                nearest = lqr15.lastNearestIdx();
                state = sim.update(state, delta, pid.compute(v_ref, state.v));
                double md = 1e9;
                for (auto& wp : traj15) {
                    double d = std::hypot(wp.x - state.x, wp.y - state.y);
                    if (d < md) md = d;
                }
                f << step << "," << state.x << "," << state.y << ","
                  << state.theta << "," << state.v << "," << md << "\n";
                if (nearest >= (int)traj15.size() - 10) break;
            }
        }
    }

    return 0;
}
