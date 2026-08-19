cmake_minimum_required(VERSION 3.8)
project(av_controller)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(av_control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 引入第二阶段的控制器源文件
set(CONTROLLER_SRC
    $ENV{HOME}/av_control_guide/cpp_controllers/src/pid_controller.cpp
    $ENV{HOME}/av_control_guide/cpp_controllers/src/pure_pursuit.cpp
    $ENV{HOME}/av_control_guide/cpp_controllers/src/stanley_controller.cpp
)
include_directories($ENV{HOME}/av_control_guide/cpp_controllers/include)

# 控制器节点
add_executable(controller_node src/controller_node.cpp ${CONTROLLER_SRC})
ament_target_dependencies(controller_node rclcpp av_control_msgs)
target_link_libraries(controller_node Eigen3::Eigen)

# 轨迹发布节点
add_executable(trajectory_publisher src/trajectory_publisher.cpp)
ament_target_dependencies(trajectory_publisher rclcpp av_control_msgs)

install(TARGETS controller_node trajectory_publisher
    DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
    scripts/sim_bridge_node.py
    scripts/carla_bridge_node.py
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

ament_package()
