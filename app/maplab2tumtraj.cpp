/**
 * @file maplab2tumtraj.cpp
 * source file for converting a maplab vertex file to a tum trajectory file
 */
#include <iostream>
#include <vio/derivedLinePatterns.h>

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cout << "Usage:" << argv[0] << " maplab-vertex.csv [tum-traj.txt]\n";
    return -1;
  }
  std::string maplabfile = argv[1];
  size_t pos = maplabfile.find_last_of("/\\");
  std::string tumfile = maplabfile.substr(0, pos) + "/tum-traj.txt";

  if (argc > 2) {
    tumfile = argv[2];
  }

  std::vector<vio::MaplabVertexPattern,
              Eigen::aligned_allocator<vio::MaplabVertexPattern>>
      maplabvertices;
  vio::loadCsv(maplabfile, maplabvertices, 1);

  std::vector<vio::TumTrajPattern,
              Eigen::aligned_allocator<vio::TumTrajPattern>>
      tumposes;
  tumposes.reserve(maplabvertices.size());
  for (const auto &vertex : maplabvertices) {
    tumposes.emplace_back(vertex);
  }

  std::cout << "Saving densified poses to " << tumfile << "." << std::endl;
  std::string header = "# timestamp tx ty tz qx qy qz qw";
  vio::saveCsv(tumposes, tumfile, header);
  return 0;
}
