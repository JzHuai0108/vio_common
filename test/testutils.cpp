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
  std::cout << "val1 " << val1 << " val2 " << val2 << std::endl;
  ASSERT_EQ(val1, 213189.943);
  ASSERT_EQ(val2, 43289);
}
