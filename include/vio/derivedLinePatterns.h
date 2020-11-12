#ifndef DERIVED_LINE_PATTERNS_H
#define DERIVED_LINE_PATTERNS_H

#include "vio/CsvReader.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iomanip>

namespace vio {

class MaplabVertexPattern : public LinePattern {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MaplabVertexPattern() {}
  virtual ~MaplabVertexPattern() {}
  static MaplabVertexPattern Random() {
    MaplabVertexPattern pat;
    pat.vertex_index = rand() / 4;
    pat.time_ns = rand();
    pat.p_WS_ = Eigen::Vector3d::Random();
    pat.q_WS_.coeffs().setRandom();
    pat.q_WS_.normalize();
    pat.v_WS_ = Eigen::Vector3d::Random();
    pat.accelBias_ = Eigen::Vector3d::Random();
    pat.gyroBias_ = Eigen::Vector3d::Random();
    return pat;
  };

  bool equal(const MaplabVertexPattern &rhs) {
    double precision = 1e-7;
    return vertex_index == rhs.vertex_index && time_ns == rhs.time_ns &&
           p_WS_.isApprox(rhs.p_WS_, precision) &&
           q_WS_.isApprox(rhs.q_WS_, precision) &&
           v_WS_.isApprox(rhs.v_WS_, precision) &&
           accelBias_.isApprox(rhs.accelBias_, 1e-5) &&
           gyroBias_.isApprox(rhs.gyroBias_, 1e-5);
  }

  std::ostream &print(std::ostream &os) const override;
  std::istream &read(std::istream &is) override;

  size_t vertex_index;
  int64_t time_ns;
  Eigen::Vector3d p_WS_;
  Eigen::Quaterniond q_WS_; // In order x, y, z, w when printed out.
  Eigen::Vector3d v_WS_;
  Eigen::Vector3d accelBias_;
  Eigen::Vector3d gyroBias_;
};

class MaplabTrackPattern : public LinePattern {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MaplabTrackPattern() {}
  virtual ~MaplabTrackPattern() {}

  std::ostream &print(std::ostream &os) const override {
    char delim = ',';
    os << time_ns << delim << vertex_index << delim << frame_index << delim
       << keypoint_index << delim << std::setprecision(8) << measurement[0]
       << delim << measurement[1] << delim << std::setprecision(6)
       << uncertainty << delim << scale << delim << track_id;
    return os;
  }

  std::istream &read(std::istream &is) override {
    char delim;
    is >> time_ns >> delim >> vertex_index >> delim >> frame_index >> delim >>
        keypoint_index >> delim >> measurement[0] >> delim >> measurement[1] >>
        delim >> uncertainty >> delim >> scale >> delim >> track_id;
    return is;
  }

  static MaplabTrackPattern Random() {
    MaplabTrackPattern pat;
    pat.time_ns = rand();
    pat.vertex_index = rand() / 4;
    pat.frame_index = rand() / 4;
    pat.keypoint_index = rand() / 4;
    pat.measurement = Eigen::Vector2d::Random();
    pat.uncertainty = rand() % 100;
    pat.scale = rand() % 100;
    pat.track_id = rand() / 4;
    return pat;
  }

  bool equal(const MaplabTrackPattern &rhs) {
    double precision = 1e-7;
    return time_ns == rhs.time_ns && vertex_index == rhs.vertex_index &&
           frame_index == rhs.frame_index &&
           keypoint_index == rhs.keypoint_index &&
           measurement.isApprox(rhs.measurement, precision) &&
           std::fabs(uncertainty - rhs.uncertainty) < precision &&
           std::fabs(scale - rhs.scale) < precision && track_id == rhs.track_id;
  }

  int64_t time_ns;
  size_t vertex_index;
  size_t frame_index;
  size_t keypoint_index;
  Eigen::Vector2d measurement;
  double uncertainty;
  double scale;
  size_t track_id;
};

class MaplabLandmarkPattern : public LinePattern {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MaplabLandmarkPattern() {}
  virtual ~MaplabLandmarkPattern() {}

  std::ostream &print(std::ostream &os) const override {
    char delim = ',';
    os << landmark_index << delim << std::setprecision(8) << position[0]
       << delim << position[1] << delim << position[2];
    return os;
  }

  std::istream &read(std::istream &is) override {
    char delim;
    is >> landmark_index >> delim >> position[0] >> delim >> position[1] >>
        delim >> position[2];
    return is;
  }

  static MaplabLandmarkPattern Random() {
    MaplabLandmarkPattern pat;
    pat.landmark_index = rand() / 4;
    pat.position = Eigen::Vector3d::Random();
    return pat;
  }

  bool equal(const MaplabLandmarkPattern &rhs) {
    return landmark_index == rhs.landmark_index &&
           position.isApprox(rhs.position, 1e-7);
  }

  size_t landmark_index;
  Eigen::Vector3d position;
};

class MaplabObservationPattern : public LinePattern {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MaplabObservationPattern() {}
  virtual ~MaplabObservationPattern() {}

  std::ostream &print(std::ostream &os) const override {
    char delim = ',';
    os << vertex_index << delim << frame_index << delim << keypoint_index
       << delim << landmark_index;
    return os;
  }

  std::istream &read(std::istream &is) override {
    char delim;
    is >> vertex_index >> delim >> frame_index >> delim >> keypoint_index >>
        delim >> landmark_index;
    return is;
  }

  static MaplabObservationPattern Random() {
    MaplabObservationPattern pat;
    pat.vertex_index = rand() / 4;
    pat.frame_index = rand() / 4;
    pat.keypoint_index = rand() / 4;
    pat.landmark_index = rand() / 4;
    return pat;
  }

  bool equal(const MaplabObservationPattern &rhs) {
    return vertex_index == rhs.vertex_index && frame_index == rhs.frame_index &&
           keypoint_index == rhs.keypoint_index &&
           landmark_index == rhs.landmark_index;
  }

  size_t vertex_index;
  size_t frame_index;
  size_t keypoint_index;
  size_t landmark_index;
};

class OkvisOutputPattern : public LinePattern {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OkvisOutputPattern();

  virtual ~OkvisOutputPattern() {}

  std::ostream &print(std::ostream &) const override;

  /**
   * @brief read one line of the csv file:
   * timestamp(unit nsec) frameIdInSource, p_WS_W_x p_WS_W_y p_WS_W_z
   * q_WS_x q_WS_y q_WS_z q_WS_w v_WS_W_x v_WS_W_y v_WS_W_z b_g_x b_g_y b_g_z
   * b_a_x b_a_y b_a_z and optionally, p_SC_S_x p_SC_S_y p_SC_S_z q_SC_x
   * q_SC_y q_SC_z q_SC_w
   * @return
   */
  std::istream &read(std::istream &) override;

  double timestamp() const override;

  static OkvisOutputPattern Random() {
    OkvisOutputPattern pat;

    pat.sec_ = rand() / 4;
    pat.nsec_ = rand() / 4;
    std::stringstream ss;
    ss << pat.sec_ << std::setw(9) << std::setfill('0') << pat.nsec_;
    pat.time_ = ss.str();
    pat.frameIdInSource = rand() / 4;
    pat.p_WS_.setRandom();
    pat.q_WS_.coeffs().setRandom();
    pat.q_WS_.normalize();
    pat.sb_WS_.setRandom();
    pat.p_SC_.setRandom();
    pat.q_SC_.coeffs().setRandom();
    pat.q_SC_.normalize();
    return pat;
  }

  bool equal(const OkvisOutputPattern &rhs) {
    double precision = 1e-7;
    return time_ == rhs.time_ && sec_ == rhs.sec_ && nsec_ == rhs.nsec_ &&
           frameIdInSource == rhs.frameIdInSource &&
           p_WS_.isApprox(rhs.p_WS_, precision) &&
           q_WS_.isApprox(q_WS_, precision) &&
           sb_WS_.isApprox(rhs.sb_WS_, 1e-6) &&
           p_SC_.isApprox(p_SC_, precision) && q_SC_.isApprox(q_SC_, precision);
  }

public:
  std::string time_;
  uint32_t sec_;
  uint32_t nsec_;
  int frameIdInSource;
  Eigen::Vector3d p_WS_;
  Eigen::Quaterniond q_WS_;
  Eigen::Matrix<double, 9, 1> sb_WS_;
  Eigen::Vector3d p_SC_;
  Eigen::Quaterniond q_SC_;
};

class ViclamOutputPattern : public LinePattern {
public:
  ViclamOutputPattern();
  virtual ~ViclamOutputPattern() {}
  std::ostream &print(std::ostream &) const override;

  /**
   * @brief read one line of the viclam output file contains timestamp(unit
   * nsec) p_WS_W_x p_WS_W_y p_WS_W_z q_WS_x q_WS_y q_WS_z q_WS_w v_WS_W_x
   * v_WS_W_y v_WS_W_z b_g_x b_g_y b_g_z b_a_x b_a_y b_a_z
   * @return
   */
  std::istream &read(std::istream &) override;
  double timestamp() const override;

public:
  std::string time_;
  uint32_t sec_;
  uint32_t nsec_;
  double p_WS_[3];
  double q_WS_[4];
  double sb_WS_[9];
};

// use CsvReader to load data from a csv file.
template <class Pattern>
void loadCsvData(
    std::string csvFile,
    std::vector<Pattern, Eigen::aligned_allocator<Pattern>> &csvData,
    int headerLines = 1, double startTime = 0.0, double finishTime = 1e20) {
  if (finishTime <= 0)
    finishTime = 1e20;
  CsvReader reader(csvFile, std::shared_ptr<LinePattern>(new Pattern()),
                   headerLines);
  while (reader.getNextObservation()) {
    double time = reader.currentRow()->timestamp();
    if (time < startTime)
      continue;
    if (time > finishTime)
      break;
    csvData.push_back(
        *std::static_pointer_cast<const Pattern>(reader.currentRow()));
  }
}

} // namespace vio

#endif // DERIVED_LINE_PATTERNS_H
