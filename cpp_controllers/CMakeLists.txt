cmake_minimum_required(VERSION 3.22)
project(av_control_benchmark VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OsqpEigen REQUIRED)
find_package(GTest REQUIRED)

add_compile_options(-Wall -Wextra -O2)

include_directories(include)

add_library(controllers STATIC
    src/bicycle_model.cpp
    src/dynamic_bicycle_model.cpp
    src/pid_controller.cpp
    src/pure_pursuit.cpp
    src/stanley_controller.cpp
    src/lqr_controller.cpp
    src/mpc_controller.cpp
)
target_link_libraries(controllers Eigen3::Eigen OsqpEigen::OsqpEigen)

add_executable(run_simulation src/main.cpp)
target_link_libraries(run_simulation controllers)

add_executable(carla_interface src/carla_interface.cpp)
target_link_libraries(carla_interface controllers)

enable_testing()
add_executable(test_controllers
    tests/test_pid.cpp
    tests/test_pure_pursuit.cpp
    tests/test_lqr.cpp
    tests/test_mpc.cpp
)
target_link_libraries(test_controllers controllers GTest::GTest GTest::Main)
add_test(NAME unit_tests COMMAND test_controllers)
