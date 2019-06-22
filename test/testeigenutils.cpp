#include <gtest/gtest.h>
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Dense"

#include "vio/eigen_utils.h"

TEST(Eigen, Inverse) {
  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
  information(5, 5) = 1.0e8;
  information(0, 0) = 1.0e8;
  information(1, 1) = 1.0e8;
  information(2, 2) = 1.0e8;
  Eigen::Matrix<double, 6, 6> cov = information.inverse();
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(std::isnan(cov(0, 0)));
  }
  EXPECT_NEAR(cov(5, 5), 1e-8, 1e-10);
}

TEST(Eigen, SuperDiagonal) {
  Eigen::MatrixXd M = Eigen::MatrixXd::Random(3, 5);
  ASSERT_LT((vio::superdiagonal(M) - vio::subdiagonal(M.transpose())).norm(),
            1e-9);

  M = Eigen::MatrixXd::Random(4, 4);
  ASSERT_LT((vio::superdiagonal(M) - vio::subdiagonal(M.transpose())).norm(),
            1e-9);

  M = Eigen::MatrixXd::Random(5, 3);
  ASSERT_LT((vio::superdiagonal(M) - vio::subdiagonal(M.transpose())).norm(),
            1e-9);
}
