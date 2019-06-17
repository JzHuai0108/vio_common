#include <gtest/gtest.h>
#include "vio/FrameGrabber.h"

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

TEST(FrameGrabber, ReadVideo) {
  std::string videoFile =
      "/home/jhuai/Desktop/temp_android/2019_06_16_12_03_45_phab2/movie.mp4";
  std::string timeFile = "";
  playVideoOnConsole(videoFile, timeFile);
}

TEST(FrameGrabber, ReadVideoWithTimeFile) {
  std::string videoFile =
      "/home/jhuai/Desktop/temp_android/2019_06_16_12_03_45_phab2/movie.mp4";
  std::string timeFile =
      "/home/jhuai/Desktop/temp_android/2019_06_16_12_03_45_phab2/"
      "frame_timestamps.txt";
  playVideoOnConsole(videoFile, timeFile);
}

TEST(FrameGrabber, ReadVideoWithMetadataFile) {
  std::string videoFile =
      "/media/jhuai/OldWin8OS/iphone6s_visual_inertial_data/"
      "2019_06_16_14_49_53_6s/IMG_4302.MP4";
  std::string metadataFile =
      "/media/jhuai/OldWin8OS/iphone6s_visual_inertial_data/"
      "2019_06_16_14_49_53_6s/movie_metadata.csv";
  playVideoOnConsole(videoFile, metadataFile);
}
