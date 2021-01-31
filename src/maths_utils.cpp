
#include "vio/maths_utils.h"

#include <cassert>
#include <limits>

namespace vio {
Eigen::Vector2d project2d(const Eigen::Vector3d& v) {
  Eigen::Vector2d res;
  res(0) = v(0) / v(2);
  res(1) = v(1) / v(2);
  return res;
}

Eigen::Vector2f project2f(const Eigen::Vector3f& v) {
  Eigen::Vector2f res;
  res(0) = v(0) / v(2);
  res(1) = v(1) / v(2);
  return res;
}

Eigen::Vector3d project3d(const Eigen::Vector4d& v) {
  Eigen::Vector3d res;
  res(0) = v(0) / v(3);
  res(1) = v(1) / v(3);
  res(2) = v(2) / v(3);
  return res;
}

Eigen::Vector3d unproject2d(const Eigen::Vector2d& v) {
  Eigen::Vector3d res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = 1;
  return res;
}

Eigen::Vector3f unproject2f(const Eigen::Vector2f& v) {
  Eigen::Vector3f res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = 1.f;
  return res;
}

Eigen::Vector4d unproject3d(const Eigen::Vector3d& v) {
  Eigen::Vector4d res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = v(2);
  res(3) = 1;
  return res;
}

double norm_max(const Eigen::VectorXd& v) {
  double max = -1;
  for (int i = 0; i < v.size(); i++) {
    double abs = std::fabs(v[i]);
    if (abs > max) {
      max = abs;
    }
  }
  return max;
}

// https://stackoverflow.com/questions/14344833/rounding-integers-to-nearest-ten-or-hundred-in-c
int round_up_to_max_pow10(int n) {
  int tmp = n;
  int i = 0;
  while ((tmp /= 10) >= 10) {
    i++;
  }

  if (n % (int)(pow(10, i + 1) + 0.5)) {
    tmp++;
  }

  for (; i >= 0; i--) {
    tmp *= 10;
  }
  return tmp;
}

}  // namespace vio
