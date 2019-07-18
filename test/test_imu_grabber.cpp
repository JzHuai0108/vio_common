#include <gtest/gtest.h>
#include "vio/ImuGrabber.h"

const double tol = 1e-8;

TEST(IMUGrabber, TwoTypes) {
  using namespace vio;
  std::string file_path = __FILE__;
  std::string vio_common_test_dir =
      file_path.substr(0, file_path.find_last_of("/\\"));
  std::string mImuFile0(vio_common_test_dir + "/data/indexed_imu.txt");
  IMUGrabber mIG0(mImuFile0, IndexedPlainText);

  mIG0.getObservation(0);

  ASSERT_EQ(mIG0.getMeasurement().size(), 0u);
  mIG0.getObservation(505.417);

  ASSERT_EQ(mIG0.getMeasurement().size(), 1u);
  RawImuMeasurementVector measurement = mIG0.getMeasurement();
  ASSERT_NEAR(measurement.back()[0], 505.403, tol);
  ASSERT_NEAR(measurement.back()[1], -0.617105477905, tol);

  std::string mImuFile(vio_common_test_dir + "/data/sensorstream.csv");
  IMUGrabber mIG(mImuFile, SensorStreamCSV);

  mIG.getObservation(0);

  ASSERT_EQ(mIG.getMeasurement().size(), 0u);
  mIG.getObservation(511951.63293);

  ASSERT_EQ(mIG.getMeasurement().size(), 0u);
  mIG.getObservation(511951.72555);

  ASSERT_EQ(mIG.getMeasurement().size(), 0u);

  mIG.getObservation(511951.75116);

  ASSERT_EQ(mIG.getMeasurement().size(), 3u);
  measurement = mIG.getMeasurement();
  ASSERT_NEAR(measurement.back()[0], 511951.75116, tol);
  ASSERT_NEAR(measurement.back()[1], 9.876, tol);

  mIG.getObservation(511951.88972);

  measurement = mIG.getMeasurement();
  ASSERT_NEAR(measurement.front()[0], 511951.75116, tol);
  ASSERT_NEAR(measurement.front()[1], 9.876, tol);
  ASSERT_NEAR(measurement.back()[0], 511951.88972, tol);
  ASSERT_NEAR(measurement.back()[1], 9.893, tol);
}

TEST(IMUGrabber, KalibrCsv) {
  std::string file_path = __FILE__;
  std::string vio_common_test_dir =
      file_path.substr(0, file_path.find_last_of("/\\"));
  std::string filename = vio_common_test_dir + "/data/mars_gyro_accel.csv";
  vio::IMUGrabber grabber(filename, vio::IMUFileType::KalibrCsv);
  grabber.getObservation(0);

  ASSERT_EQ(grabber.getMeasurement().size(), 0u);
  grabber.getObservation(13634.912727792);
  ASSERT_EQ(grabber.getMeasurement().size(), 0u);

  grabber.getObservation(675013.980903239);

  ASSERT_EQ(grabber.getMeasurement().size(), 1u);
  vio::RawImuMeasurementVector measurement = grabber.getMeasurement();
  ASSERT_NEAR(measurement.front()[0], 675013.980903239, tol);
  ASSERT_NEAR(measurement.front()[1], -0.2753296, tol);

  grabber.getObservation(675014.000882253);
  ASSERT_EQ(grabber.getMeasurement().size(), 1u);

  measurement = grabber.getMeasurement();
  ASSERT_NEAR(measurement.front()[0], 675013.980903239, tol);
  ASSERT_NEAR(measurement.front()[1], -0.2753296, tol);

  grabber.getObservation(675014.000892253);
  ASSERT_EQ(grabber.getMeasurement().size(), 2u);

  measurement = grabber.getMeasurement();
  ASSERT_NEAR(measurement.front()[0], 675013.980903239, tol);
  ASSERT_NEAR(measurement.front()[1], -0.2753296, tol);
  ASSERT_NEAR(measurement.back()[0], 675014.000892253, tol);
  ASSERT_NEAR(measurement.back()[6], -0.021194458, tol);

  grabber.getObservation(675018.577222439);
  measurement = grabber.getMeasurement();
  ASSERT_NEAR(measurement.front()[0], 675014.000892253, tol);
  ASSERT_NEAR(measurement.front()[1], -0.47584534, tol);
  ASSERT_NEAR(measurement.back()[0], 675018.477222439, tol);
  ASSERT_NEAR(measurement.back()[1], -2.6174622, tol);
}
