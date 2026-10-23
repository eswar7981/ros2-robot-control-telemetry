#pragma once
#include <cmath>

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
  double heading
)
{
  return linear_velocity * cos(heading) * dt;
}

inline double updateDistanceY(
  double linear_velocity,
  double dt,
  double heading
)
{
  return linear_velocity * sin(heading) * dt;
}
