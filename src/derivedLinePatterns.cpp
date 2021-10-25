#include "vio/derivedLinePatterns.h"
#include <iomanip>
#include <sstream>
#include <string>

namespace vio {

std::ostream &MaplabVertexPattern::print(std::ostream &os) const {
  char delim = ',';
  os << vertex_index << delim << time_ns << delim << std::setprecision(8)
     << p_WS_[0] << delim << p_WS_[1] << delim << p_WS_[2] << delim << q_WS_.x()
     << delim << q_WS_.y() << delim << q_WS_.z() << delim << q_WS_.w() << delim
     << v_WS_[0] << delim << v_WS_[1] << delim << v_WS_[2] << delim
     << std::setprecision(6) << accelBias_[0] << delim << accelBias_[1] << delim
     << accelBias_[2] << delim << gyroBias_[0] << delim << gyroBias_[1] << delim
     << gyroBias_[2];
  return os;
}

std::istream &MaplabVertexPattern::read(std::istream &is) {
  char delim;
  is >> vertex_index >> delim >> time_ns >> delim >> p_WS_[0] >> delim >>
      p_WS_[1] >> delim >> p_WS_[2] >> delim >> q_WS_.x() >> delim >>
      q_WS_.y() >> delim >> q_WS_.z() >> delim >> q_WS_.w() >> delim >>
      v_WS_[0] >> delim >> v_WS_[1] >> delim >> v_WS_[2] >> delim >>
      accelBias_[0] >> delim >> accelBias_[1] >> delim >> accelBias_[2] >>
      delim >> gyroBias_[0] >> delim >> gyroBias_[1] >> delim >> gyroBias_[2];
  return is;
}

OkvisOutputPattern::OkvisOutputPattern()
    : p_SC_(0, 0, 0), q_SC_(1.0, 0, 0, 0) {}

double OkvisOutputPattern::timestamp() const { return sec_ + nsec_ * 1e-9; }

std::ostream &OkvisOutputPattern::print(std::ostream &os) const {
  char delim = ',';
  os << sec_ << std::setw(9) << std::setfill('0') << nsec_ << delim
     << frameIdInSource << delim << std::setprecision(8) << p_WS_[0] << delim
     << p_WS_[1] << delim << p_WS_[2] << delim << q_WS_.x() << delim
     << q_WS_.y() << delim << q_WS_.z() << delim << q_WS_.w() << delim
     << sb_WS_[0] << delim << sb_WS_[1] << delim << sb_WS_[2] << delim
     << std::setprecision(6) << sb_WS_[3] << delim << sb_WS_[4] << delim
     << sb_WS_[5] << delim << sb_WS_[6] << delim << sb_WS_[7] << delim
     << sb_WS_[8] << delim << std::setprecision(8) << p_SC_[0] << delim
     << p_SC_[1] << delim << p_SC_[2] << delim << q_SC_.x() << delim
     << q_SC_.y() << delim << q_SC_.z() << delim << q_SC_.w();
  return os;
}

std::istream &OkvisOutputPattern::read(std::istream &is) {
  getline(is, time_, ',');
  std::string trunk = time_.substr(0, time_.length() - 9);
  std::istringstream ss1(trunk);
  ss1 >> sec_;

  std::string residuals = time_.substr(time_.length() - 9);
  std::istringstream ss2(residuals);
  ss2 >> nsec_;

  char delim;
  is >> frameIdInSource >> delim >> p_WS_[0] >> delim >> p_WS_[1] >> delim >>
      p_WS_[2] >> delim >> q_WS_.x() >> delim >> q_WS_.y() >> delim >>
      q_WS_.z() >> delim >> q_WS_.w() >> delim >> sb_WS_[0] >> delim >>
      sb_WS_[1] >> delim >> sb_WS_[2] >> delim >> sb_WS_[3] >> delim >>
      sb_WS_[4] >> delim >> sb_WS_[5] >> delim >> sb_WS_[6] >> delim >>
      sb_WS_[7] >> delim >> sb_WS_[8];
  if (is >> delim) {
    is >> p_SC_[0] >> delim >> p_SC_[1] >> delim >> p_SC_[2] >> delim >>
        q_SC_.x() >> delim >> q_SC_.y() >> delim >> q_SC_.z() >> delim >>
        q_SC_.w();
  }
  return is;
}

TumTrajPattern::TumTrajPattern(char delim) : LinePattern(delim) {}

double TumTrajPattern::timestamp() const { return sec_ + nsec_ * 1e-9; }

std::ostream &TumTrajPattern::print(std::ostream &os) const {
  char delim = delim_;
  os << sec_ << "." << std::setw(9) << std::setfill('0') << nsec_ << delim
     << std::setprecision(8) << p_WS_[0] << delim
     << p_WS_[1] << delim << p_WS_[2] << delim << q_WS_.x() << delim
     << q_WS_.y() << delim << q_WS_.z() << delim << q_WS_.w();
  return os;
}

std::istream &TumTrajPattern::read(std::istream &is) {
  getline(is, time_, delim_);
  double time = atof(time_.c_str());

  sec_ = uint32_t(floor(time));
  nsec_ = (time - sec_) * 1e9;

  if (delim_ == ' ') {
    is >> p_WS_[0] >> p_WS_[1] >> p_WS_[2] >> q_WS_.x() >> q_WS_.y() >>
        q_WS_.z() >> q_WS_.w();
  } else {
    char delim;
    is >> p_WS_[0] >> delim >> p_WS_[1] >> delim >> p_WS_[2] >>
        delim >> q_WS_.x() >> delim >> q_WS_.y() >> delim >> q_WS_.z() >>
        delim >> q_WS_.w();
  }
  return is;
}

ViclamOutputPattern::ViclamOutputPattern() {}

std::ostream &ViclamOutputPattern::print(std::ostream &os) const {
  char delim = ' ';

  os << sec_ << "." << std::setw(9) << std::setfill('0') << nsec_ << delim
     << "0" << delim << p_WS_[0] << delim << p_WS_[1] << delim << p_WS_[2]
     << delim << q_WS_[0] << delim << q_WS_[1] << delim << q_WS_[2] << delim
     << q_WS_[3] << delim << sb_WS_[0] << delim << sb_WS_[1] << delim
     << sb_WS_[2] << delim << sb_WS_[3] << delim << sb_WS_[4] << delim
     << sb_WS_[5] << delim << sb_WS_[6] << delim << sb_WS_[7] << delim
     << sb_WS_[8];

  return os;
}

std::istream &ViclamOutputPattern::read(std::istream &is) {
  getline(is, time_, ' ');
  std::string trunk = time_.substr(0, time_.length() - 9);
  std::istringstream ss1(trunk);
  ss1 >> sec_;

  std::string residuals = time_.substr(time_.length() - 9);
  std::istringstream ss2(residuals);
  ss2 >> nsec_;

  is >> p_WS_[0] >> p_WS_[1] >> p_WS_[2] >> q_WS_[0] >> q_WS_[1] >> q_WS_[2] >>
      q_WS_[3] >> sb_WS_[0] >> sb_WS_[1] >> sb_WS_[2] >> sb_WS_[3] >>
      sb_WS_[4] >> sb_WS_[5] >> sb_WS_[6] >> sb_WS_[7] >> sb_WS_[8];
  return is;
}

double ViclamOutputPattern::timestamp() const { return sec_ + nsec_ * 1e-9; }
} //  namespace vio
