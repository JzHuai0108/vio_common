#include "vio/eigen_utils.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace vio {

Eigen::Vector3d rotro2eu(Eigen::Matrix3d R) {
  Eigen::Vector3d euler;
  euler[0] = atan2(R(2, 1), R(2, 2));
  euler[1] = -(atan2(R(2, 0), sqrt(1 - R(2, 0) * R(2, 0))));
  euler[2] = atan2(R(1, 0), R(0, 0));
  return euler;
}

Eigen::Matrix3d roteu2ro(Eigen::Vector3d eul) {
  double cr = cos(eul[0]);
  double sr = sin(eul[0]);  // roll
  double cp = cos(eul[1]);
  double sp = sin(eul[1]);  // pitch
  double ch = cos(eul[2]);
  double sh = sin(eul[2]);  // heading
  Eigen::Matrix3d dcm;
  dcm(0, 0) = cp * ch;
  dcm(0, 1) = (sp * sr * ch) - (cr * sh);
  dcm(0, 2) = (cr * sp * ch) + (sh * sr);

  dcm(1, 0) = cp * sh;
  dcm(1, 1) = (sr * sp * sh) + (cr * ch);
  dcm(1, 2) = (cr * sp * sh) - (sr * ch);

  dcm(2, 0) = -sp;
  dcm(2, 1) = sr * cp;
  dcm(2, 2) = cr * cp;
  return dcm;
}
// input: lat, long in radians, height is immaterial
// output: Ce2n
Eigen::Matrix3d llh2dcm(const Eigen::Vector3d llh) {
  double sL = sin(llh[0]);
  double cL = cos(llh[0]);
  double sl = sin(llh[1]);
  double cl = cos(llh[1]);

  Eigen::Matrix3d Ce2n;
  Ce2n << -sL * cl, -sL * sl, cL, -sl, cl, 0, -cL * cl, -cL * sl, -sL;
  return Ce2n;
}

Eigen::MatrixXd nullspace(const Eigen::MatrixXd &A) {
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(A);
  // ColPivHouseholderQR<MatrixXd> qr(A); //don't use column pivoting because in
  // that case Q*R-A!=0
  Eigen::MatrixXd nullQ = qr.householderQ();

  int rows = A.rows(), cols = A.cols();
  assert(rows >
         cols);  // "Rows should be greater than columns in computing nullspace"
  nullQ = nullQ.block(0, cols, rows, rows - cols).eval();
  return nullQ;
}

void leftNullspaceAndColumnSpace(const Eigen::MatrixXd &A, Eigen::MatrixXd *Q2,
                                 Eigen::MatrixXd *Q1) {
  int rows = A.rows(), cols = A.cols();
  assert(rows > cols);  // "Rows should be greater than columns in computing
                        // left nullspace"
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(A);
  // don't use column pivoting because in that case Q*R-A!=0
  Eigen::MatrixXd Q = qr.householderQ();

  Q2->resize(rows, rows - cols);
  *Q2 = Q.block(0, cols, rows, rows - cols);

  Q1->resize(rows, cols);
  *Q1 = Q.block(0, 0, rows, cols);
}

Eigen::Matrix<double, Eigen::Dynamic, 1> superdiagonal(
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &M) {
  const int numElements = std::min(M.rows(), M.cols()) - 1;
  Eigen::Matrix<double, Eigen::Dynamic, 1> r(numElements, 1);
  for (int jack = 0; jack < numElements; ++jack) r[jack] = M(jack, jack + 1);
  return r;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> subdiagonal(
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &M) {
  const int numElements = std::min(M.rows(), M.cols()) - 1;
  Eigen::Matrix<double, Eigen::Dynamic, 1> r(numElements, 1);
  for (int jack = 0; jack < numElements; ++jack) r[jack] = M(jack + 1, jack);
  return r;
}

Eigen::Vector3d unskew3d(const Eigen::Matrix3d &Omega) {
  return 0.5 * Eigen::Vector3d(Omega(2, 1) - Omega(1, 2),
                               Omega(0, 2) - Omega(2, 0),
                               Omega(1, 0) - Omega(0, 1));
}

// https://github.com/ethz-asl/maplab_lightweight_filtering/blob/28417454164b08d5483754f0c0d250b84f2ce951/include/lightweight_filtering/Update.hpp
double getConditionNumberOfMatrix(const Eigen::MatrixXd &matrix) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
  return svd.singularValues()(0) /
         svd.singularValues()(svd.singularValues().size() - 1);
}
}  // namespace vio
