#include <gtest/gtest.h>
#include "lqr_controller.hpp"

TEST(LQRTest, StraightLineZeroSteer) {
    LQRController lqr;

    std::vector<LQRController::Waypoint> traj;
    for (int i = 0; i < 50; ++i)
        traj.push_back({(double)i * 2.0, 0.0, 0.0, 5.0, 0.0});

    Eigen::Vector4d state(0.0, 0.0, 0.0, 5.0);
    auto [delta, v_ref] = lqr.compute(state, traj);

    EXPECT_NEAR(delta,  0.0, 0.1);
    EXPECT_NEAR(v_ref,  5.0, 0.1);
}
