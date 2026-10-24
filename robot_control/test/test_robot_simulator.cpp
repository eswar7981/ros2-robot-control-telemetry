#include <gtest/gtest.h>
#include <cmath>

#include "robot_control/robot_motion_utils.hpp"

TEST(RobotSimulatorTest, RotateBy90Degrees)
{
  const double current_heading = 0.0;
  const double angular_velocity = 5.0 * std::acos(-1.0);
  const double dt = 0.1;

  const double actual_heading =
    updateHeading(current_heading, angular_velocity, dt);

  const double expected_heading = std::acos(-1.0) / 2.0;

  EXPECT_NEAR(actual_heading, expected_heading, 0.001);
}

TEST(RobotSimulatorTest, CalculateDistanceMoved)
{
  const double angular_velocity = 0.0;
  const double linear_velocity = 0.3;
  const double dt = 0.1;

  const double heading = angular_velocity * dt;

  const double actual_distance_x = updateDistanceX(linear_velocity, dt, heading);
  const double actual_distance_y = updateDistanceY(linear_velocity, dt, heading);

  const double expected_distance_x = 0.03;
  const double expected_distance_y = 0;

  EXPECT_NEAR(actual_distance_x, expected_distance_x, 0.001);
  EXPECT_NEAR(actual_distance_y, expected_distance_y, 0.001);
}
