#ifndef GJK_INTERFACE_UTIL_H_
#define GJK_INTERFACE_UTIL_H_

#include <cmath>

namespace toolbox {
// normalizate to -Pi ~ Pi
inline double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2 * M_PI);
  if (a <= 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

// return to -from
inline double DifferentAngle(const double from, const double to) {
  return NormalizeAngle(to - from);
}

inline double FastSqrt(const float x) {
  float xhalf = 0.5f * x;
  int i = *(int*)&x;
  i = 0x5f3759df - (i >> 1);
  float temp_x = *(float*)&i;
  temp_x = temp_x * (1.5f - xhalf * temp_x * temp_x);
  return 1 / temp_x;
}

inline double Distance(const double x1,
                       const double y1,
                       const double x2,
                       const double y2) {
  double x = (x2 - x1) * (x2 - x1);
  double y = (y2 - y1) * (y2 - y1);
  float s = static_cast<float>(x + y);
  return FastSqrt(s);
}

} // namespace toolbox

#endif // GJK_INTERFACE_UTIL_H_