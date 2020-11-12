#ifndef VIMAP_CONTAINER_H
#define VIMAP_CONTAINER_H

#include "vio/derivedLinePatterns.h"
#include <Eigen/Core>
#include <Eigen/StdVector>

namespace vio {

class VimapContainer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VimapContainer(std::string vimap_folder);

  std::vector<MaplabVertexPattern,
              Eigen::aligned_allocator<MaplabVertexPattern>>
      vertices;
  std::vector<MaplabTrackPattern, Eigen::aligned_allocator<MaplabTrackPattern>>
      tracks;
  std::vector<MaplabObservationPattern,
              Eigen::aligned_allocator<MaplabObservationPattern>>
      observations;
  std::vector<MaplabLandmarkPattern,
              Eigen::aligned_allocator<MaplabLandmarkPattern>>
      landmarks;
};

} // namespace vio
#endif // VIMAP_CONTAINER_H
