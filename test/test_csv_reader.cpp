#include "vio/derivedLinePatterns.h"

#include <gtest/gtest.h>

TEST(CsvReader, LoadOkvisOutput) {
  const std::string csvFile = "../test/data/okvis_output.csv";
  std::vector<vio::OkvisOutputPattern,
              Eigen::aligned_allocator<vio::OkvisOutputPattern>>
      csvData;
  double startTime = 0, finishTime = 1e10;
  if (!vio::fileExists(csvFile))
    return;
  vio::loadCsvData(csvFile, csvData, 1, startTime, finishTime);
  vio::OkvisOutputPattern front, back;
  front.time_ = "512936278663280";
  front.sec_ = 512936;
  front.nsec_ = 278663280;
  front.frameIdInSource = 28700;
  front.p_WS_.setZero();
  front.q_WS_ = Eigen::Quaterniond(0.64142462, -0.017613815, -0.76698384, 0);
  front.sb_WS_ << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  front.p_SC_ << 0, 0, 0;
  front.q_SC_.coeffs() << 0.70710678, -0.70710678, 0, 0;

  back.time_ = "513235791900702";
  back.sec_ = 513235;
  back.nsec_ = 791900702;
  back.frameIdInSource = 37687;
  back.p_WS_ << 122.4555, -165.11338, 0.39004863;
  back.q_WS_.coeffs() << -0.68518187, -0.24485088, -0.65239702, 0.21201882;
  back.sb_WS_ << 0.032632162, 0.043736248, -0.0049669805, 0.00126365,
      0.00693262, -0.000191527, -0.0884216, -0.111703, 0.0403887;
  back.p_SC_ << 0, 0, 0;
  back.q_SC_.coeffs() << 0.70710678, -0.70710678, 0, 0;

  EXPECT_TRUE(csvData[0].equal(front));
  EXPECT_TRUE(csvData.back().equal(back));
}

TEST(CsvReader, OkvisOutputPattern) {
  vio::OkvisOutputPattern pat = vio::OkvisOutputPattern::Random();
  std::stringstream ss;
  pat.print(ss);
  vio::OkvisOutputPattern pat2;
  pat2.read(ss);
  EXPECT_TRUE(pat2.equal(pat));
}

TEST(CsvReader, MaplabVertexPattern) {
  vio::MaplabVertexPattern pat = vio::MaplabVertexPattern::Random();
  std::stringstream ss;
  pat.print(ss);
  vio::MaplabVertexPattern pat2;
  pat2.read(ss);
  EXPECT_TRUE(pat2.equal(pat));
}

TEST(CsvReader, MaplabTrackPattern) {
  vio::MaplabTrackPattern pat = vio::MaplabTrackPattern::Random();
  std::stringstream ss;
  pat.print(ss);
  vio::MaplabTrackPattern pat2;
  pat2.read(ss);
  EXPECT_TRUE(pat2.equal(pat));
}

TEST(CsvReader, MaplabObservationPattern) {
  vio::MaplabObservationPattern pat = vio::MaplabObservationPattern::Random();
  std::stringstream ss;
  pat.print(ss);
  vio::MaplabObservationPattern pat2;
  pat2.read(ss);
  EXPECT_TRUE(pat2.equal(pat));
}

TEST(CsvReader, MaplabLandmarkPattern) {
  vio::MaplabLandmarkPattern pat = vio::MaplabLandmarkPattern::Random();
  std::stringstream ss;
  pat.print(ss);
  vio::MaplabLandmarkPattern pat2;
  pat2.read(ss);
  EXPECT_TRUE(pat2.equal(pat));
}
