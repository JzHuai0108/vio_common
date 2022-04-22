
#ifndef DATA_GRABBER_H
#define DATA_GRABBER_H

#include <fstream>
#include <iostream>

namespace vio {

class DataGrabber {
 public:
  DataGrabber(const std::string file)
      : reinit_data(true),
        is_measurement_good(false),
        file(file),
        stream(file.c_str()),
        tkm1(-1) {
    if (!stream.is_open())
      std::cerr << "DataGrabber failed to open file:" << file << ".\n";
  }

  DataGrabber(const DataGrabber &) =
      delete;  // Note a derived class CANNOT be made uncopyable by declaring
               // copy constructor/operator private in base class
  DataGrabber &operator=(const DataGrabber &) = delete;
  virtual ~DataGrabber() {
    if (stream.is_open()) {
      stream.close();
    }
  }
  // get next bundle of observations from t(p(k-1)-1) to t(p(k)-1) given t(k),
  // where t(p(k)-1)<=t(k), t(p(k))>t(k)
  virtual bool getObservation(double tk) {
    tkm1 = tk;
    return false;
  }

 protected:
  bool reinit_data;  // do we need to grab sensor data until the current frame
                     // timestamp t(k) from an unknown timestamp
  bool is_measurement_good;  // does the measurement fit our requirements?
  std::string file;          // sensor data file name
  std::ifstream stream;      // sensor data stream
  double tkm1;               // t(k-1), timestamp of last frame
};
}  // namespace vio
#endif  // IMU_GRABBER_H
