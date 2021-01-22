#ifndef VIMAP_CONTAINER_H
#define VIMAP_CONTAINER_H

#include <Eigen/Core>
#include <Eigen/StdVector>
#include "vio/derivedLinePatterns.h"

namespace vio {
struct CornersInImage {
  int64_t time_ns;
  size_t cam_id;
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
      corners;
  std::vector<int> corner_ids;
  std::vector<double> radii;  //!< threshold used for maximum displacement
                              //! during sub-pix refinement; Search region is
  //! slightly larger.
  bool equal(const CornersInImage& rhs) const;

  friend std::ostream& operator<<(std::ostream& os, const CornersInImage& t);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class VimapContainer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VimapContainer();

  explicit VimapContainer(std::string vimap_folder);

  void loadVimapFromFolder(std::string vimap_folder);

  std::vector<int64_t> vertexTimestamps() const;

  bool checkLandmarkIndicesConsecutive() const;

  const std::vector<MaplabVertexPattern,
                    Eigen::aligned_allocator<MaplabVertexPattern>>&
  vertices() const {
    return vertices_;
  }

  const std::vector<MaplabTrackPattern,
                    Eigen::aligned_allocator<MaplabTrackPattern>>&
  tracks() const {
    return tracks_;
  }
  const std::vector<MaplabObservationPattern,
                    Eigen::aligned_allocator<MaplabObservationPattern>>&
  observations() const {
    return observations_;
  }
  const std::vector<MaplabLandmarkPattern,
                    Eigen::aligned_allocator<MaplabLandmarkPattern>>&
  landmarks() const {
    return landmarks_;
  }

  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
  homogeneousLandmarks() const {
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
        coordinates;
    coordinates.reserve(landmarks_.size());
    for (const vio::MaplabLandmarkPattern& landmark : landmarks_) {
      Eigen::Vector4d coord;
      coord << landmark.position, 1.0;
      coordinates.push_back(coord);
    }
    return coordinates;
  }

  std::vector<MaplabVertexPattern,
              Eigen::aligned_allocator<MaplabVertexPattern>>&
  verticesMutable() {
    return vertices_;
  }

  std::vector<MaplabTrackPattern, Eigen::aligned_allocator<MaplabTrackPattern>>&
  tracksMutable() {
    return tracks_;
  }
  std::vector<MaplabObservationPattern,
              Eigen::aligned_allocator<MaplabObservationPattern>>&
  observationsMutable() {
    return observations_;
  }
  std::vector<MaplabLandmarkPattern,
              Eigen::aligned_allocator<MaplabLandmarkPattern>>&
  landmarksMutable() {
    return landmarks_;
  }

  size_t numberCameras() const { return numberCameras_; }

  const std::vector<CornersInImage, Eigen::aligned_allocator<CornersInImage>>&
  validKeypoints() const {
    return validKeypoints_;
  }

  bool createValidKeypoints();

 private:
  // Per maplab implementation, the vertex indices are consecutive from 0.
  std::vector<MaplabVertexPattern,
              Eigen::aligned_allocator<MaplabVertexPattern>>
      vertices_;
  // Per maplab implementation, the keypoints in one image have consecutive
  // indices. These keypoints are either detected by a feature detector say ORB
  // or tracked from a keypoint in the previous frame by gyro aided KLT tracker.
  // Note the observations of a feature track are always in consecutive vertices.
  std::vector<MaplabTrackPattern, Eigen::aligned_allocator<MaplabTrackPattern>>
      tracks_;
  // Observations for a landmark may come from vertices of distant indices
  // because of loop closure and landmark merges.
  std::vector<MaplabObservationPattern,
              Eigen::aligned_allocator<MaplabObservationPattern>>
      observations_;
  // Per maplab implementation, the landmark indices are consecutive from 0.
  std::vector<MaplabLandmarkPattern,
              Eigen::aligned_allocator<MaplabLandmarkPattern>>
      landmarks_;

  size_t numberCameras_;
  std::vector<CornersInImage, Eigen::aligned_allocator<CornersInImage>>
      validKeypoints_;
};

}  // namespace vio
#endif  // VIMAP_CONTAINER_H
