#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>

#include "av_control_msgs/msg/trajectory.hpp"
#include "av_control_msgs/msg/vehicle_state.hpp"
#include "av_control_msgs/msg/vehicle_cmd.hpp"
#include "pid_controller.hpp"
#include "pure_pursuit.hpp"

using namespace std::chrono_literals;
using Trajectory   = av_control_msgs::msg::Trajectory;
using VehicleState = av_control_msgs::msg::VehicleState;
using VehicleCmd   = av_control_msgs::msg::VehicleCmd;

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {
        this->declare_parameter("pid_kp",    1.5);
        this->declare_parameter("pid_ki",    0.1);
        this->declare_parameter("pid_kd",    0.05);
        this->declare_parameter("pp_k",      0.3);
        this->declare_parameter("pp_ld_min", 3.0);

        pid_ = std::make_unique<PIDController>(
            this->get_parameter("pid_kp").as_double(),
            this->get_parameter("pid_ki").as_double(),
            this->get_parameter("pid_kd").as_double()
        );
        pp_ = std::make_unique<PurePursuitController>(
            2.7,
            this->get_parameter("pp_k").as_double(),
            this->get_parameter("pp_ld_min").as_double()
        );

        traj_sub_ = this->create_subscription<Trajectory>(
            "/trajectory", 10,
            [this](const Trajectory::SharedPtr msg) {
                trajectory_.clear();
                for (size_t i = 0; i < msg->x.size(); ++i)
                    trajectory_.push_back({msg->x[i], msg->y[i], msg->yaw[i], msg->v_ref[i]});
                nearest_idx_ = 0;
                RCLCPP_INFO_ONCE(this->get_logger(), "收到轨迹，共 %zu 个路点", trajectory_.size());
            });

        state_sub_ = this->create_subscription<VehicleState>(
            "/vehicle_state", 10,
            [this](const VehicleState::SharedPtr msg) {
                state_ << msg->x, msg->y, msg->theta, msg->v;
                state_received_ = true;
            });

        cmd_pub_ = this->create_publisher<VehicleCmd>("/vehicle_cmd", 10);

        timer_ = this->create_wall_timer(50ms, [this]() { controlLoop(); });

        RCLCPP_INFO(this->get_logger(), "控制器节点已启动");
    }

private:
    void controlLoop() {
        if (!state_received_ || trajectory_.empty()) return;

        auto [delta, v_ref] = pp_->compute(state_, trajectory_, nearest_idx_);
        nearest_idx_ = pp_->lastNearestIdx();

        double accel = pid_->compute(v_ref, state_[3]);

        VehicleCmd cmd;
        cmd.header.stamp = this->now();
        cmd.delta    = delta;
        cmd.accel    = accel;
        cmd.throttle = std::clamp(accel / 3.0,  0.0, 1.0);
        cmd.brake    = std::clamp(-accel / 5.0, 0.0, 1.0);
        cmd.steer    = std::clamp(-delta / 1.2217, -1.0, 1.0);

        cmd_pub_->publish(cmd);

        if (nearest_idx_ >= (int)trajectory_.size() - 10)
            RCLCPP_INFO_ONCE(this->get_logger(), "已到达轨迹终点");
    }

    std::unique_ptr<PIDController>         pid_;
    std::unique_ptr<PurePursuitController> pp_;

    std::vector<PurePursuitController::Waypoint> trajectory_;
    Eigen::Vector4d state_ = Eigen::Vector4d::Zero();
    bool state_received_   = false;
    int  nearest_idx_      = 0;

    rclcpp::Subscription<Trajectory>::SharedPtr   traj_sub_;
    rclcpp::Subscription<VehicleState>::SharedPtr state_sub_;
    rclcpp::Publisher<VehicleCmd>::SharedPtr      cmd_pub_;
    rclcpp::TimerBase::SharedPtr                  timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
