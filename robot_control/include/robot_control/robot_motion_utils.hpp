#pragma once
#include <cmath>
#include <algorithm>

inline double updateHeading(
  double current_heading,
  double angular_velocity,
  double dt)
{
  return current_heading + angular_velocity * dt;
}

inline double updateDistanceX(
  double linear_velocity,
  double dt,
  double heading)
{
  return linear_velocity * cos(heading) * dt;
}

inline double updateDistanceY(
  double linear_velocity,
  double dt,
  double heading)
{
  return linear_velocity * sin(heading) * dt;
}

inline bool isValidDistanceGoal(
  double distance)
{
  return distance > 0.0;
}

inline bool isValidRotationAngle(
  int angle)
{
  return angle >= 0 && angle <= 360;
}

inline double getClampVelocity(
  double velocity,
  double maximum_velocity)
{
  return std::clamp(velocity, -maximum_velocity, maximum_velocity);
}
