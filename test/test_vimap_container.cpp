/*
 * file: test_vimap_container.cpp
 */
#include "gtest/gtest.h"

#include "vio/VimapContainer.h"

TEST(VimapContainer, createValidKeypoints) {
  vio::VimapContainer container;
  int numVertices = 5;
  int numCameras = 2;
  int numKeypoints = 20;
  int numLandmarks = 10;
  int timeIntervalNs = 33000000;
  Eigen::Vector2d wh(640, 480);
  for (int i = 0; i < numVertices; ++i) {
    vio::MaplabVertexPattern vertex = vio::MaplabVertexPattern::Random();
    vertex.vertex_index = i;
    vertex.time_ns = i * timeIntervalNs;
    container.verticesMutable().emplace_back(vertex);
  }

  std::vector<vio::CornersInImage,
              Eigen::aligned_allocator<vio::CornersInImage>>
      expectedValidKeypoints(numVertices * numCameras);

  for (int i = 0; i < numVertices; ++i) {
    for (int j = 0; j < numCameras; ++j) {
      for (int k = 0; k < numKeypoints; ++k) {
        vio::MaplabTrackPattern keypoint;
        keypoint.time_ns = i * timeIntervalNs;
        keypoint.vertex_index = i;
        keypoint.frame_index = j;
        keypoint.keypoint_index = k;
        keypoint.measurement =
            wh.cwiseProduct(Eigen::Vector2d::Random()) / 2 + wh / 2;
        keypoint.uncertainty = rand() % 100;
        keypoint.scale = rand() % 100;
        keypoint.track_id = -1;
        container.tracksMutable().emplace_back(keypoint);
      }
    }
  }
  std::srand(unsigned(std::time(0)));
  std::vector<int> indices;

  // set some values:
  for (int i = 0; i < numKeypoints; ++i) indices.push_back(i);

  for (int i = 0; i < numVertices; ++i) {
    for (int j = 0; j < numCameras; ++j) {
      std::vector<int> keypointIndices = indices;
      std::random_shuffle(keypointIndices.begin(), keypointIndices.end());

      for (int k = 0; k < numLandmarks; ++k) {
        vio::MaplabObservationPattern observation;
        observation.vertex_index = i;
        observation.frame_index = j;
        observation.keypoint_index = keypointIndices[k];
        observation.landmark_index = k;
        container.observationsMutable().push_back(observation);

        auto& corners = expectedValidKeypoints[i * numCameras + j];
        corners.cam_id = j;
        corners.time_ns = container.vertices()[i].time_ns;
        corners.corners.push_back(
            container
                .tracks()[i * numCameras * numKeypoints + j * numKeypoints +
                          observation.keypoint_index]
                .measurement);
        corners.corner_ids.push_back(k);
        corners.radii.push_back(2);
      }
    }
  }

  container.createValidKeypoints();
  auto actualValidKeypoints = container.validKeypoints();

  for (size_t i = 0; i < actualValidKeypoints.size(); ++i) {
    auto actualKeypoints = actualValidKeypoints[i];
    auto expectedKeypoints = expectedValidKeypoints[i];
    EXPECT_TRUE(actualKeypoints.equal(expectedKeypoints))
        << "Actual:" << actualKeypoints << "\nExpected:" << expectedKeypoints
        << "\n";
  }
}
