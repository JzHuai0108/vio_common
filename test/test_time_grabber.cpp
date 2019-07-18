#include <gtest/gtest.h>
#include "vio/TimeGrabber.h"

const double tol = 1e-8;
TEST(TimeGrabber, PlainTextKITTI) {
  std::string file_path = __FILE__;
  std::string vio_common_test_dir =
      file_path.substr(0, file_path.find_last_of("/\\"));
  std::string timeFile = vio_common_test_dir + "/data/times_kitti_seq00.txt";
  vio::TimeGrabber tg(timeFile);
  double timestamp = tg.readTimestamp(-1);

  ASSERT_NEAR(timestamp, -1, tol);

  timestamp = tg.readTimestamp(0);
  ASSERT_NEAR(timestamp, 0, tol);

  timestamp = tg.readTimestamp(1799);
  ASSERT_NEAR(timestamp, 1.864921e+02, tol);

  timestamp = tg.readTimestamp(1800);
  ASSERT_NEAR(timestamp, 1.865959e+02, tol);

  timestamp = tg.readTimestamp(4540);
  ASSERT_NEAR(timestamp, 4.705816e+02, tol);

  timestamp = tg.readTimestamp(4541);
  ASSERT_NEAR(timestamp, -1, tol);
}

TEST(TimeGrabber, DILILI) {
  std::string file_path = __FILE__;
  std::string vio_common_test_dir =
      file_path.substr(0, file_path.find_last_of("/\\"));
  vio::TimeGrabber tg(vio_common_test_dir +
                      "/data/dilili_video_timestamps.txt");
  double timestamp = tg.extractTimestamp(50, false);
  ASSERT_NEAR(timestamp, 507.895, tol);
  timestamp = tg.extractTimestamp(100, false);
  std::cout << "image index timestamp at 100 " << timestamp << std::endl;
  ASSERT_NEAR(timestamp, 510.891, tol);
}

TEST(TimeGrabber, HonorMarsLogger) {
  std::string file_path = __FILE__;
  std::string vio_common_test_dir =
      file_path.substr(0, file_path.find_last_of("/\\"));
  vio::TimeGrabber tg(vio_common_test_dir + "/data/mars_frame_timestamps.txt");
  double timestamp = tg.readTimestamp(-1);

  ASSERT_NEAR(timestamp, -1, tol);

  timestamp = tg.readTimestamp(0);
  ASSERT_NEAR(timestamp, 1896318.620700000, tol);

  timestamp = tg.readTimestamp(1799);
  ASSERT_NEAR(timestamp, 1896378.707300000, tol);
}
