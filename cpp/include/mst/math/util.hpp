#pragma once

#include <cmath>

namespace mst {

constexpr double kPi = 3.14159265358979323846;

inline double wrap_angle_rad(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

inline double sqr(double x) { return x * x; }

}  // namespace mst
