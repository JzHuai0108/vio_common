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

TEST(eigen_utils, ReparameterizeJacobians) {
  double distances[] = {3, 3e2, 3e4, 3e8};  // close to inifity
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      expected_abrhoi{{1, -0.378937,  0.488034},
                      {1,  -0.378937, 0.00488034},
                      {1,  -0.378937, 4.88034e-05},
                      {1,  -0.378937, 4.88034e-09}};
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      expected_abrhoj{{0.225489, -0.287133, 0.507999},
                      {0.264809, -0.280117, 0.00356978},
                      {0.267946, -0.277415, 3.57259e-05},
                      {0.267949, -0.277401, 3.57266e-09}};

  for (size_t jack = 0; jack < sizeof(distances) / sizeof(distances[0]);
       ++jack) {
    double dist = distances[jack];

    Eigen::Matrix3d Ri = Eigen::Matrix3d::Identity();
    Eigen::Vector3d ptini;
    ptini << dist * cos(15 * M_PI / 180) * cos(45 * M_PI / 180),
        -dist * sin(15 * M_PI / 180),
        dist * cos(15 * M_PI / 180) * sin(45 * M_PI / 180);
    Eigen::Matrix3d Rj =
        Eigen::AngleAxisd(30 * M_PI / 180, Eigen::Vector3d::UnitY())
            .toRotationMatrix();

    Eigen::Vector3d pi = Eigen::Vector3d::Zero();
    Eigen::Vector3d pj = Eigen::Vector3d::Random();

    Eigen::Vector3d ptinj = Rj.transpose() * (ptini - pj);

    Eigen::Vector3d abrhoi = Eigen::Vector3d(ptini[0], ptini[1], 1) / ptini[2];
    Eigen::Vector3d abrhoj;
    Eigen::Matrix<double, 3, 9> jacobian;

    vio::reparameterize_AIDP(Ri, Rj, abrhoi, pi, pj, abrhoj, &jacobian);
    Eigen::Matrix<double, 3, 9> jacobianNumeric;
    vio::reparameterizeNumericalJacobian(Ri, Rj, abrhoi, pi, pj, abrhoj,
                                         jacobianNumeric);
    EXPECT_TRUE(jacobian.isApprox(jacobianNumeric, 1e-7));
    EXPECT_TRUE(expected_abrhoi[jack].isApprox(abrhoi, 1e-6));
    EXPECT_LT((expected_abrhoj[jack] - abrhoj).lpNorm<Eigen::Infinity>(), 1e-5);
  }
  // infinity
  Eigen::Matrix3d Ri = Eigen::Matrix3d::Identity();
  Eigen::Vector3d ptiniRay;
  ptiniRay << cos(15 * M_PI / 180) * cos(45 * M_PI / 180),
      -sin(15 * M_PI / 180), cos(15 * M_PI / 180) * sin(45 * M_PI / 180);
  Eigen::Matrix3d Rj =
      Eigen::AngleAxisd(30 * M_PI / 180, Eigen::Vector3d::UnitY())
          .toRotationMatrix();

  Eigen::Vector3d pi = Eigen::Vector3d::Zero();
  Eigen::Vector3d pj = Eigen::Vector3d::Random();

  Eigen::Vector3d ptinjRay = Rj.transpose() * ptiniRay;
  ptinjRay /= ptinjRay[2];
  ptinjRay[2] = 0;

  Eigen::Vector3d abrhoi =
      Eigen::Vector3d(1, -tan(15 * M_PI / 180) / sin(45 * M_PI / 180), 0);
  Eigen::Vector3d abrhoj;
  Eigen::Matrix<double, 3, 9> jacobian;

  vio::reparameterize_AIDP(Ri, Rj, abrhoi, pi, pj, abrhoj, &jacobian);
  Eigen::Matrix<double, 3, 9> jacobianNumeric;
  vio::reparameterizeNumericalJacobian(Ri, Rj, abrhoi, pi, pj, abrhoj,
                                       jacobianNumeric);

  EXPECT_TRUE(jacobian.rightCols<6>().isMuchSmallerThan(1, 1e-8));
  EXPECT_TRUE(jacobian.isApprox(jacobianNumeric, 1e-8));
  EXPECT_TRUE(abrhoj.isApprox(ptinjRay, 1e-8));
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
