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

TEST(RobotSimulatorTest, CalculateDistanceMovedInXAxis)
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

TEST(RobotSimulatorTest, CalculateDistanceMovedInYAxis)
{
  const double linear_velocity = 0.3;
  const double dt = 0.1;

  const double heading = std::acos(-1.0) / 2.0;

  const double actual_distance_x = updateDistanceX(linear_velocity, dt, heading);
  const double actual_distance_y = updateDistanceY(linear_velocity, dt, heading);

  EXPECT_NEAR(actual_distance_x, 0.0, 0.001);
  EXPECT_NEAR(actual_distance_y, 0.03, 0.001);
}


TEST(RobotSimulatorTest, DistanceGoalValidation)
{
  EXPECT_FALSE(isValidDistanceGoal(-1.0));
  EXPECT_FALSE(isValidDistanceGoal(0.0));
  EXPECT_TRUE(isValidDistanceGoal(1.0));

}

TEST(RobotSimulatorTest, RotationAngleValidation)
{
  EXPECT_FALSE(isValidRotationAngle(-1));
  EXPECT_TRUE(isValidRotationAngle(0));
  EXPECT_TRUE(isValidRotationAngle(180));
  EXPECT_TRUE(isValidRotationAngle(360));
  EXPECT_FALSE(isValidRotationAngle(361));
}

TEST(RobotSimulatorTest, RobotVelocityValidation)
{
  EXPECT_DOUBLE_EQ(getClampVelocity(0.8, 1.0), 0.8);
  EXPECT_DOUBLE_EQ(getClampVelocity(0.8, 0.5), 0.5);
  EXPECT_DOUBLE_EQ(getClampVelocity(-0.8, 0.5), -0.5);
}
