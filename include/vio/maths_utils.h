// Copyright (c) 2016, Jianzhu Huai
// You can contact the author at <huai dot 3 at osu dot edu>

#ifndef MATHSUTILS_H
#define MATHSUTILS_H

#define GL_GLEXT_PROTOTYPES 1

#include <Eigen/Core>
#include <set>

namespace vio {

Eigen::Vector2d project2d(const Eigen::Vector3d&);

Eigen::Vector3d project3d(const Eigen::Vector4d&);

Eigen::Vector3d unproject2d(const Eigen::Vector2d&);

Eigen::Vector4d unproject3d(const Eigen::Vector3d&);

double norm_max(const Eigen::VectorXd& v);

int round_up_to_max_pow10(int n);

inline Eigen::Vector3d invert_depth(const Eigen::Vector3d& x) {
  return unproject2d(x.head<2>()) / x[2];
}

template <typename T>
inline T Po2(const T& value) {
  return value * value;
}

template <typename T>
inline T Po3(const T& value) {
  return Po2(value) * value;
}

template <typename T>
T Po4(const T& value) {
  return Po2(Po2(value));
}

template <typename T>
T Po5(const T& value) {
  return Po4(value) * value;
}

template <typename T>
T Po6(const T& value) {
  return Po4(value) * Po2(value);
}

template <typename T>
T Po7(const T& value) {
  return Po6(value) * value;
}

template <typename T>
T Po8(const T& value) {
  return Po2(Po4(value));
}

template <typename T>
T median(const std::multiset<T>& m_set) {
  int size = m_set.size();
  assert(size > 0);
  typename std::multiset<T>::const_iterator it = m_set.begin();
  if (m_set.size() % 2 == 1) {
    for (int i = 0; i < size / 2; ++i) {
      it++;
    }
    return *it;
  }

  for (int i = 0; i < size / 2 - 1; ++i) {
    it++;
  }
  T val = *it;
  it++;
  val += *it;
  return 0.5 * val;
}

}  // namespace vio
#endif
