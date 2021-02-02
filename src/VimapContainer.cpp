#include "vio/VimapContainer.h"

#include <iostream>
#include <string>

#include "vio/removeFromVector.hpp"
#include "vio/sort.h"

namespace vio {
bool CornersInImage::equal(const CornersInImage& rhs) const {
  bool corners_equal = true;
  bool ids_equal = true;
  bool radii_equal = true;
  for (size_t i = 0; i < rhs.corner_ids.size(); ++i) {
    corners_equal = corners_equal && corners[i].isApprox(rhs.corners[i], 1e-8);
    ids_equal = ids_equal && corner_ids[i] == rhs.corner_ids[i];
    radii_equal = radii_equal && radii[i] == rhs.radii[i];
  }
  return corners_equal && ids_equal && radii_equal && time_ns == rhs.time_ns &&
         cam_id == rhs.cam_id;
}

std::ostream& operator<<(std::ostream& os, const CornersInImage& t) {
  char delim = ',';
  os << t.time_ns << delim << t.cam_id << "\n";
  for (size_t i = 0; i < t.corner_ids.size(); ++i) {
    os << i << ": (" << t.corner_ids[i] << "," << t.corners[i][0] << ","
       << t.corners[i][1] << "," << t.radii[i] << ")\n";
  }
  return os;
}

VimapContainer::VimapContainer() {}

VimapContainer::VimapContainer(std::string vimap_folder) {
  loadVimapFromFolder(vimap_folder);
}

bool VimapContainer::checkLandmarkIndicesConsecutive() const {
  bool consecutive = landmarks_.front().landmark_index == 0u &&
                     landmarks_.back().landmark_index == landmarks_.size() - 1;
  if (!consecutive) {
    std::cerr << "Landmark indices are not consecutive!\n";
  }
  return consecutive;
}

void VimapContainer::loadVimapFromFolder(std::string vimap_folder) {
  std::string vertices_csv = vimap_folder + "/vertices.csv";
  std::string tracks_csv = vimap_folder + "/tracks.csv";
  std::string observations_csv = vimap_folder + "/observations.csv";
  std::string landmarks_csv = vimap_folder + "/landmarks.csv";
  std::string imu_csv = vimap_folder + "/imu.csv";
  if (fileExists(vertices_csv)) {
    loadCsvData(vertices_csv, vertices_, 1);
  }
  if (fileExists(tracks_csv)) {
    loadCsvData(tracks_csv, tracks_, 1);
    numberCameras_ = tracks_.back().frame_index + 1;
  }
  if (fileExists(landmarks_csv)) {
    loadCsvData(landmarks_csv, landmarks_, 1);
    checkLandmarkIndicesConsecutive();
  }
  if (fileExists(observations_csv)) {
    loadCsvData(observations_csv, observations_, 1);
    createValidKeypoints();
  }
  if (fileExists(imu_csv)) {
    vio::loadCsvData(imu_csv, imuData_, 1);
    for (ImuOutputPattern &row : imuData_) {
        std::swap(row.w_, row.a_);
    }
  }
}

bool VimapContainer::check() const {
  bool status = true;
  if (vertices_.size()) {
    std::cout << "First and last lines of vertices.\n";
    vertices_.front().print(std::cout);
    std::cout << std::endl;
    vertices_.back().print(std::cout);
    std::cout << std::endl;
  } else {
    std::cerr << "Vimap has no vertices.\n";
    status = false;
  }
  if (tracks_.size()) {
    std::cout << "First and last lines of tracks.\n";
    tracks_.front().print(std::cout);
    std::cout << std::endl;
    tracks_.back().print(std::cout);
    std::cout << std::endl;
  } else {
    std::cerr << "Vimap has no tracks.\n";
    status = false;
  }
  if (observations_.size()) {
    std::cout << "First and last lines of observations.\n";
    observations_.front().print(std::cout);
    std::cout << std::endl;
    observations_.back().print(std::cout);
    std::cout << std::endl;
  } else {
    std::cerr << "Vimap has no observations.\n";
    status = false;
  }
  if (landmarks_.size()) {
    std::cout << "First and last lines of landmarks.\n";
    landmarks_.front().print(std::cout);
    std::cout << std::endl;
    landmarks_.back().print(std::cout);
    std::cout << std::endl;
  } else {
    std::cerr << "Vimap has no landmarks.\n";
    status = false;
  }
  return status;
}

std::vector<int64_t> VimapContainer::vertexTimestamps() const {
  std::vector<int64_t> timestamps;
  timestamps.reserve(vertices_.size());
  for (auto vertex : vertices_) {
    timestamps.emplace_back(vertex.time_ns);
  }
  return timestamps;
}

bool VimapContainer::createValidKeypoints() {
  size_t maxVertexIndex = tracks_.back().vertex_index;
  size_t maxCameraIndex = tracks_.back().frame_index;
  size_t numCameras = maxCameraIndex + 1;
  constexpr size_t expectedKeypoints = 500u;
  validKeypoints_.resize((maxVertexIndex + 1) * (maxCameraIndex + 1));
  for (CornersInImage& corners : validKeypoints_) {
    corners.corners.reserve(expectedKeypoints);
    corners.radii.reserve(expectedKeypoints);
    corners.corner_ids.reserve(expectedKeypoints);
  }
  for (const MaplabTrackPattern& keypoint : tracks_) {
    CornersInImage& corners =
        validKeypoints_[keypoint.vertex_index * numCameras +
                        keypoint.frame_index];
    if (corners.corners.size() != keypoint.keypoint_index) {
      std::cerr << "Keypoint index " << keypoint.keypoint_index
                << " disagrees with #corners in image "
                << corners.corners.size() << "!\n";
    }
    corners.radii.push_back(2);
    corners.corners.emplace_back(keypoint.measurement);
    corners.corner_ids.emplace_back(-1);
  }
  // assign vertex timestamps and camera ids
  int imageIndex = 0;
  for (CornersInImage& corners : validKeypoints_) {
    corners.cam_id = imageIndex % numCameras;
    corners.time_ns = vertices_[imageIndex / numCameras].time_ns;
    ++imageIndex;
  }
  // assign landmark ids
  for (const MaplabObservationPattern& observation : observations_) {
    CornersInImage& corners =
        validKeypoints_[observation.vertex_index * numCameras +
                        observation.frame_index];
    corners.corner_ids[observation.keypoint_index] = observation.landmark_index;
  }
  // remove keypoints not associated to landmarks
  for (CornersInImage& corners : validKeypoints_) {
    std::vector<bool> status;
    status.reserve(corners.corners.size());
    for (auto id : corners.corner_ids) {
      status.push_back(id != -1);
    }
    removeUnsetMatrices(&corners.corners, status);
    removeUnsetElements(&corners.corner_ids, status);
    removeUnsetElements(&corners.radii, status);

    // optionally sort according to corner id
    std::vector<size_t> sort_indices;
    vio::sort(corners.corner_ids, corners.corner_ids, sort_indices);
    vio::reorderMatrices(corners.corners, sort_indices, corners.corners);
    vio::reorder(corners.radii, sort_indices, corners.radii);
  }
  return true;
}

}  // namespace vio
