
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

  // this reading function mainly works for KITTI timestamp files
  // line_number 0 based index in the timestamp file
  double readTimestamp(int line_number);

  // frame_number 0 based index in the timestamp file
  double extractTimestamp(int frame_number, bool isMalagaDataset = true);
  bool isTimeAvailable() const;

  std::string time_file;
  std::ifstream time_stream;
  int last_line_index;  // must be initialized as -1
  double last_line_time;
  std::string last_left_image_name;
  bool isTimeInNanos;
  bool isTimeFormatSet;
};
}  // namespace vio
#endif  // TIMEGRABBER_H
