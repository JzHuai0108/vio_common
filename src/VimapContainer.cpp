#include "vio/VimapContainer.h"
#include <string>

namespace vio {
VimapContainer::VimapContainer(std::string vimap_folder) {
  std::string vertices_csv = vimap_folder + "/vertices.csv";
  std::string tracks_csv = vimap_folder + "/tracks.csv";
  std::string observations_csv = vimap_folder + "/observations.csv";
  std::string landmarks_csv = vimap_folder + "/landmarks.csv";
  if (fileExists(vertices_csv)) {
    loadCsvData(vertices_csv, vertices, 1);
  }
  if (fileExists(tracks_csv)) {
    loadCsvData(tracks_csv, tracks, 2);
  }
  if (fileExists(observations_csv)) {
    loadCsvData(observations_csv, observations, 1);
  }
  if (fileExists(landmarks_csv)) {
    loadCsvData(landmarks_csv, landmarks, 1);
  }
}
} // namespace vio
