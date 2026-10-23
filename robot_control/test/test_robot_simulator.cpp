#include <gtest/gtest.h>
#include <cmath>

#include "robot_control/robot_motion_utils.hpp"

TEST(RobotSimulatorTest, RotateBy90Degrees)
{
  const double current_heading = 0.0;
  const double angular_velocity = std::acos(-1.0) / 2.0;
  const double dt = 0.1;

  const double actual_heading =
      updateHeading(current_heading, angular_velocity, dt);

  const double expected_heading = std::acos(-1.0) / 2.0;

  EXPECT_NEAR(actual_heading, expected_heading, 0.001);
}
