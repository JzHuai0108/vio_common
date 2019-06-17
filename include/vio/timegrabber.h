
#ifndef TIMEGRABBER_H
#define TIMEGRABBER_H

#include <fstream>
#include <iostream>
#include <vector>

namespace vio {
class TimeGrabber {
 public:
  TimeGrabber();
  TimeGrabber(const std::string time_file_name);
  TimeGrabber &operator=(const TimeGrabber &) = delete;
  TimeGrabber(const TimeGrabber &) = delete;
  ~TimeGrabber();
  bool init(const std::string time_file_name);

  // this reading function only works for KITTI timestamp files
  double readTimestamp(int line_number);

  double extractTimestamp(int frame_number, bool isMalagaDataset = true);
  bool isTimeAvailable() const;

  std::string time_file;
  std::ifstream time_stream;
  int last_line_index;  // must be initialized as -1
  double last_line_time;
  std::string last_left_image_name;
};
}  // namespace vio
#endif  // TIMEGRABBER_H
