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

TEST(eigen_utils, ExtractBlocks) {
  Eigen::MatrixXd m(5, 5);
  m << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
      21, 22, 23, 24, 25;

  std::vector<std::pair<size_t, size_t> > vRowStartInterval;
  for (size_t jack = 0; jack < 5; ++jack)
    vRowStartInterval.push_back(std::make_pair(jack, 1));
  // test deleting none entry
  Eigen::MatrixXd res = vio::extractBlocks(m, vRowStartInterval, vRowStartInterval);
  EXPECT_LT((res - m).lpNorm<Eigen::Infinity>(), 1e-8);

  // test deleting odd indexed rows/cols
  vRowStartInterval.clear();
  for (size_t jack = 0; jack < 5; jack += 2)
    vRowStartInterval.push_back(std::make_pair(jack, 1));
  res = vio::extractBlocks(m, vRowStartInterval, vRowStartInterval);
  Eigen::MatrixXd expected(3, 3);
  expected << 1, 3, 5, 11, 13, 15, 21, 23, 25;
  EXPECT_LT((res - expected).lpNorm<Eigen::Infinity>(), 1e-8);

  // test deleting even indexed rows/cols
  vRowStartInterval.clear();
  for (size_t jack = 1; jack < 5; jack += 2)
    vRowStartInterval.push_back(std::make_pair(jack, 1));
  res = vio::extractBlocks(m, vRowStartInterval, vRowStartInterval);
  Eigen::MatrixXd expected2(2, 2);
  expected2 << 7, 9, 17, 19;
  EXPECT_LT((res - expected2).lpNorm<Eigen::Infinity>(), 1e-8);

  // test with keeping more than 1 rows/cols each time
  vRowStartInterval.clear();
  vRowStartInterval.push_back(std::make_pair(0, 2));
  vRowStartInterval.push_back(std::make_pair(3, 2));
  res = vio::extractBlocks(m, vRowStartInterval, vRowStartInterval);
  Eigen::MatrixXd expected3(4, 4);
  expected3 << 1, 2, 4, 5, 6, 7, 9, 10, 16, 17, 19, 20, 21, 22, 24, 25;

  EXPECT_LT((res - expected3).lpNorm<Eigen::Infinity>(), 1e-8);

  // test with different rows and cols to keep
  vRowStartInterval.clear();
  vRowStartInterval.push_back(std::make_pair(0, 2));
  vRowStartInterval.push_back(std::make_pair(3, 2));
  std::vector<std::pair<size_t, size_t> > vColStartInterval;
  vColStartInterval.push_back(std::make_pair(0, 2));
  vColStartInterval.push_back(std::make_pair(3, 1));
  res = vio::extractBlocks(m, vRowStartInterval, vColStartInterval);
  Eigen::MatrixXd expected4(4, 3);
  expected4 << 1, 2, 4, 6, 7, 9, 16, 17, 19, 21, 22, 24;
  EXPECT_LT((res - expected4).lpNorm<Eigen::Infinity>(), 1e-8);
}

TEST(Eigen, QuaternionDiff) {
  Eigen::Vector4d xyzw(0.0002580249512621188, 0.0005713455181395015,
                       0.7010130254309982, 0.7131481929890183);
  Eigen::Quaterniond ref(xyzw[3], xyzw[0], xyzw[1], xyzw[2]);

  Eigen::Quaterniond actual(xyzw.data());
  EXPECT_LT(Eigen::AngleAxisd(ref.inverse() * actual).angle(), 1e-8);
}
