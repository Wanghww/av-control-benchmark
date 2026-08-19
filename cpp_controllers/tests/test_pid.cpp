#include <gtest/gtest.h>
#include "pid_controller.hpp"

TEST(PIDTest, ZeroErrorOutputsZero) {
    PIDController pid(1.0, 0.0, 0.0);
    EXPECT_NEAR(pid.compute(5.0, 5.0), 0.0, 1e-6);
}

TEST(PIDTest, PositiveErrorOutputsPositive) {
    PIDController pid(1.0, 0.0, 0.0);
    EXPECT_GT(pid.compute(10.0, 5.0), 0.0);
}

TEST(PIDTest, ResetClearsIntegral) {
    PIDController pid(0.0, 1.0, 0.0);
    pid.compute(1.0, 0.0);
    pid.reset();
    EXPECT_NEAR(pid.compute(0.0, 0.0), 0.0, 1e-6);
}
