#pragma once

inline double updateHeading(
    double current_heading,
    double angular_velocity,
    double dt)
{
  return current_heading + angular_velocity * dt;
}