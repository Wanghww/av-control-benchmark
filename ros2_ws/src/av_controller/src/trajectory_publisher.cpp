#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include "av_control_msgs/msg/trajectory.hpp"

using namespace std::chrono_literals;
using Trajectory = av_control_msgs::msg::Trajectory;

class TrajectoryPublisher : public rclcpp::Node {
public:
    TrajectoryPublisher() : Node("trajectory_publisher") {
        pub_   = this->create_publisher<Trajectory>("/trajectory", 10);
        msg_   = generateSCurve();
        timer_ = this->create_wall_timer(1000ms, [this]() {
            msg_.header.stamp = this->now();
            pub_->publish(msg_);
            RCLCPP_INFO_ONCE(this->get_logger(), "轨迹已发布，共 %zu 个路点", msg_.x.size());
        });
        RCLCPP_INFO(this->get_logger(), "轨迹发布节点已启动");
    }

private:
    Trajectory generateSCurve() {
        Trajectory msg;
        const int    N      = 300;
        const double length = 100.0;
        for (int i = 0; i < N; ++i) {
            double x    = length * i / N;
            double y    = 10.0 * std::sin(2.0 * M_PI * x / length);
            double dy_dx = 10.0 * std::cos(2.0 * M_PI * x / length) * (2.0 * M_PI / length);
            double yaw  = std::atan2(dy_dx, 1.0);
            msg.x.push_back(x);
            msg.y.push_back(y);
            msg.yaw.push_back(yaw);
            msg.v_ref.push_back(6.0);
        }
        return msg;
    }

    rclcpp::Publisher<Trajectory>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr             timer_;
    Trajectory                               msg_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
