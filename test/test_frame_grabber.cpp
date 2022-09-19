#include <gtest/gtest.h>
#include "vio/FrameGrabber.h"
#include "test/test_config.h"

void playVideoOnConsole(const std::string videoFile,
                        const std::string timeFile) {
  vio::FrameGrabber mFG(videoFile, timeFile, 0, -1);

  cv::Mat frame;
  double frameTime;
  std::cout << "Reading " << videoFile << " and\n" << timeFile << std::endl;
  int count = 0;
  while (mFG.grabFrame(frame, frameTime)) {
    if (count < 10 || count > 295 * 60) {
      std::cout << "read in frame at " << std::setprecision(20) << frameTime
                << " id " << mFG.getCurrentId() << std::endl;
    }
    ++count;
  }
}

TEST(FrameGrabber, ReadVideoWithTimeFile) {
  std::string videoFile = std::string(DATASET_PATH) + "/honorv10/movie.mp4";
  std::string timeFile = std::string(DATASET_PATH) + "/honorv10/frame_timestamps.txt";
  playVideoOnConsole(videoFile, timeFile);
}

TEST(FrameGrabber, ReadVideo) {
  std::string videoFile = std::string(DATASET_PATH) + "/honorv10/movie.mp4";
  playVideoOnConsole(videoFile, "");
}
