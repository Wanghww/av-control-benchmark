#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "pid_controller.hpp"
#include "pure_pursuit.hpp"

static std::vector<PurePursuitController::Waypoint> loadTrajectory(const std::string& path) {
    std::vector<PurePursuitController::Waypoint> traj;
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "Cannot open trajectory: " << path << std::endl;
        return traj;
    }
    std::string line;
    std::getline(f, line);  // 跳过表头
    while (std::getline(f, line)) {
        std::istringstream ss(line);
        PurePursuitController::Waypoint wp;
        char comma;
        ss >> wp.x >> comma >> wp.y >> comma >> wp.yaw >> comma >> wp.v_ref;
        traj.push_back(wp);
    }
    return traj;
}

int main(int argc, char* argv[]) {
    std::string traj_path = (argc > 1) ? argv[1] : "trajectory.csv";

    auto traj = loadTrajectory(traj_path);
    if (traj.empty()) return 1;

    PIDController pid(1.5, 0.1, 0.05);
    PurePursuitController pp(2.87, 0.3, 3.0);  // Tesla Model 3 轴距

    const double MAX_STEER_RAD = 1.2217;  // 70°，与 carla_env.py 一致

    double x, y, theta, v;
    int nearest_idx = 0;

    // 从 stdin 持续读取车辆状态，输出控制指令到 stdout
    while (std::cin >> x >> y >> theta >> v) {
        Eigen::Vector4d state(x, y, theta, v);
        auto [delta, v_ref] = pp.compute(state, traj, nearest_idx);
        nearest_idx = pp.lastNearestIdx();

        double accel    = pid.compute(v_ref, v);
        double throttle = std::max(accel / 3.0, 0.0);
        double brake    = std::max(-accel / 5.0, 0.0);
        double steer    = -delta / MAX_STEER_RAD;  // Pure Pursuit delta>0=左，CARLA steer<0=左

        // 限幅
        throttle = std::min(throttle, 1.0);
        brake    = std::min(brake, 1.0);
        steer    = std::max(-1.0, std::min(1.0, steer));

        std::cout << throttle << " " << steer << " " << brake << std::endl;
    }

    return 0;
}
