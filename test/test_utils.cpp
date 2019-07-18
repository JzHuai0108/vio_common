#include <gtest/gtest.h>
#include "vio/utils.h"

TEST(IsHeaderLine, AllSorts) {
  ASSERT_FALSE(vio::isHeaderLine("000.312,0.231, 21312"));
  ASSERT_TRUE(vio::isHeaderLine("/000.312,0.231, 21312"));
  ASSERT_TRUE(vio::isHeaderLine("%000.312,0.231, 21312"));
  ASSERT_TRUE(vio::isHeaderLine("#000.312,0.231, 21312"));
  ASSERT_TRUE(vio::isHeaderLine(""));
  ASSERT_TRUE(vio::isHeaderLine("000.312a,0.231, 21312"));
  ASSERT_TRUE(vio::isHeaderLine("00.3S2 0.231 21312"));
  ASSERT_FALSE(vio::isHeaderLine("00.382 0.231 21312"));
}

TEST(countHeaderLines, ReadStringStream) {
  std::istringstream ss(
      "#comment1\n"
      "%comment2\n"
      "\n"
      "//comment3\n"
      "2019_07_12_12_23_04\n"
      "0342.324s,23901s\n"
      "21938.901j 9129213.938\n"
      "2319.213,2349081,2139,95938\n"
      "213189.943,43289,319282\n"
      "9853892 8432 903218\n");

  int count = vio::countHeaderLines(&ss);
  ASSERT_EQ(count, 7);
  double val1, val2;
  char delimiter;
  ss >> val1 >> delimiter >> val2;

  ASSERT_EQ(val1, 213189.943);
  ASSERT_EQ(val2, 43289);
}

TEST(nanoIntToSecDouble, LargeNumber) {
  double time = vio::nanoIntToSecDouble(2138791718739417839);
  ASSERT_NEAR(time, 2138791718.739417839, 1e-9);
}

TEST(isVideoFile, AllSorts) {
  ASSERT_TRUE(vio::isVideoFile("afsad.mp4"));
  ASSERT_TRUE(vio::isVideoFile("/afsad/adfs/sd.MP4"));
  ASSERT_FALSE(vio::isVideoFile("afsad/MP34/"));
  ASSERT_FALSE(vio::isVideoFile("afsad/MP34"));
}

TEST(isTimeInNanos, Number) {
  ASSERT_TRUE(vio::isTimeInNanos(312.0000000006));
  ASSERT_TRUE(vio::isTimeInNanos(3120000000006));
  ASSERT_FALSE(vio::isTimeInNanos(312.000000002));
}

TEST(FileSystem, fileExist) {
  ASSERT_FALSE(vio::pathExist(""));
  std::string file_path = __FILE__;
  std::string vio_common_test_dir =
      file_path.substr(0, file_path.find_last_of("/\\"));

  EXPECT_TRUE(vio::pathExist(vio_common_test_dir + "/test_utils.cpp"));
  EXPECT_TRUE(vio::pathExist(vio_common_test_dir));
  EXPECT_TRUE(vio::pathExist(vio_common_test_dir + "/"));

  ASSERT_FALSE(vio::dirExist(""));
  ASSERT_FALSE(vio::dirExist(vio_common_test_dir + "/test_utils.cpp"));
  EXPECT_TRUE(vio::dirExist(vio_common_test_dir));
  EXPECT_TRUE(vio::dirExist(vio_common_test_dir + "/"));
}

TEST(String, locateSubstring) {
  std::string a = "what is this";
  std::vector<std::string> vec{"si", "", "what", " "};
  std::vector<bool> res;
  vio::locateSubstring(a, vec, &res);
  ASSERT_FALSE(res[0]);
  ASSERT_FALSE(res[1]);
  ASSERT_TRUE(res[2]);
  ASSERT_TRUE(res[3]);
}
