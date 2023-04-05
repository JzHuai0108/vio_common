/*
 * file: test_vimap_container.cpp
 */
#include "gtest/gtest.h"
#include <glog/logging.h>
#include "kannala_brandt_camera4.hpp"
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

TEST(VimapContainer, ProjectionCheck) {
  std::string file_path = __FILE__;
  std::string vio_common_test_dir =
      file_path.substr(0, file_path.find_last_of("/\\"));

  Eigen::Matrix<double, 3, 4> T_BC0;
  T_BC0 << -0.999525037869675, 0.00750191850740521, -0.0298901303164331, 0.0455748356496981,
          0.0296153438858632, -0.0343973606139315, -0.998969345370176, -0.0711618018379971,
                -0.00852232821165475, -0.999380079249884,  0.0341588512738562, -0.0446812541171444;
  Eigen::Vector3d p_BC0 = T_BC0.col(3);
  Eigen::Quaterniond q_BC0(T_BC0.topLeftCorner<3, 3>());
  Eigen::Matrix<double, 8, 1> c0;
  c0 << 190.97847715128717, 190.9733070521226, 254.93170605935475, 256.8974428996504, 
  0.0034823894022493434, 0.0007150348452162257, -0.0020532361418706202, 0.00020293673591811182;
  basalt::KannalaBrandtCamera4<double> kb0(c0);

  Eigen::Matrix<double, 3, 4> T_BC1;
  T_BC1 <<  -0.999511048497858, 0.00810407926382252, -0.0301991362458914, -0.0554563391425955,
                  0.0302991163766006, 0.0125116437201924, -0.999462566741855, -0.0692500216422477,
                 -0.00772188302873336, -0.999888885162099, -0.0127510725739409, -0.0474528648075134;
  
  Eigen::Vector3d p_BC1 = T_BC1.col(3);
  std::cout << "p_BC1 " << p_BC1.transpose() << std::endl;
  Eigen::Quaterniond q_BC1(T_BC1.topLeftCorner<3, 3>());
  std::cout << "q_BC1 " << q_BC1.coeffs().transpose() << std::endl;

  Eigen::Matrix<double, 8, 1> c1;
  c1 << 190.44236969414825, 190.4344384721956, 252.59949716835982, 254.91723064636983, 
  0.0034003170790442797, 0.001766278153469831, -0.00266312569781606, 0.0003299517423931039;

  basalt::KannalaBrandtCamera4<double> kb1(c1);

  std::string vimap_folder = "/data/maplab/maplab_tumviroom1/301cc02fcde84c171300000000000000";
  vio::VimapContainer vimap(vimap_folder);
  std::cout << "number of cameras " << vimap.numberCameras() << std::endl;
  // compute projection for each keypoint using the equidistant model
  // borrow the model from basalt.
  const std::vector<vio::CornersInImage, Eigen::aligned_allocator<vio::CornersInImage>>&
    keypointFrames = vimap.validKeypoints();
  const std::vector<vio::MaplabLandmarkPattern, Eigen::aligned_allocator<vio::MaplabLandmarkPattern>>& 
    landmarks = vimap.landmarks();
  const std::vector<vio::MaplabVertexPattern, Eigen::aligned_allocator<vio::MaplabVertexPattern>>&
    vertices = vimap.vertices();
  std::vector<double> reprojerrors;
  reprojerrors.reserve(10000);
  for (size_t k = 0; k < landmarks.size(); k++) {
      CHECK(k == landmarks[k].landmark_index);
  }
  size_t good = 0, bad = 0, left = 0, right = 0;
  for (size_t i = 0; i < keypointFrames.size(); ++i) {
    for (size_t j = 0; j < keypointFrames[i].corners.size(); j++) {
      Eigen::Vector2d mz = keypointFrames[i].corners[j];
      size_t lmkId = keypointFrames[i].corner_ids[j];
      Eigen::Vector3d point = landmarks[lmkId].position;

      size_t vertexid = i / vimap.numberCameras();
      size_t camid = i % vimap.numberCameras();
      Eigen::Vector3d p_WB = vertices[vertexid].p_WS_;
      Eigen::Quaterniond q_WB = vertices[vertexid].q_WS_;
      Eigen::Vector3d pointB = q_WB.inverse() * (point - p_WB);

      Eigen::Matrix<double, 2, 1> predicted_z;
      if (camid == 0) {
        Eigen::Vector3d pointC = q_BC0.inverse() * (pointB - p_BC0);
        kb0.project(pointC, predicted_z, nullptr, nullptr);
        left++;
      } else {
        Eigen::Vector3d pointC = q_BC1.inverse() * (pointB - p_BC1);
        kb1.project(pointC, predicted_z, nullptr, nullptr);
        right++;
      }
      
      Eigen::Matrix<double, 2, 1> res = predicted_z - mz;
      
      if(res.lpNorm<Eigen::Infinity>() > 5){
        std::cout<< "warn: reproj res " << res.transpose() << std::endl;
        bad++;
      } else {
        good++;
      }
      reprojerrors.push_back(res.norm());
    }
  }
  std::cout << "Good " << good << " bad " << bad << " left " << left << " right " << right << std::endl;
}
