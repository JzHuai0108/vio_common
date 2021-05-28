#ifdef LOCAL_OPENCV
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#else
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#endif

#include <iomanip>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

// http://stackoverflow.com/questions/675039/how-can-i-create-directory-tree-in-c-linux
#include <errno.h>     // errno, ENOENT, EEXIST
#include <sys/stat.h>  // stat
#include <string>
#if defined(_WIN32)
#include <direct.h>  // _mkdir
#endif

bool isDirExist(const std::string& path) {
#if defined(_WIN32)
  struct _stat info;
  if (_stat(path.c_str(), &info) != 0) {
    return false;
  }
  return (info.st_mode & _S_IFDIR) != 0;
#else
  struct stat info;
  if (stat(path.c_str(), &info) != 0) {
    return false;
  }
  return (info.st_mode & S_IFDIR) != 0;
#endif
}

bool makePath(const std::string& path) {
#if defined(_WIN32)
  int ret = _mkdir(path.c_str());
#else
  mode_t mode = 0755;
  int ret = mkdir(path.c_str(), mode);
#endif
  if (ret == 0) return true;

  switch (errno) {
    case ENOENT:
      // parent didn't exist, try to create it
      {
        int pos = path.find_last_of('/');
        if ((size_t)pos == std::string::npos)
#if defined(_WIN32)
          pos = path.find_last_of('\\');
        if (pos == std::string::npos)
#endif
          return false;
        if (!makePath(path.substr(0, pos))) return false;
      }
      // now, try to create again
#if defined(_WIN32)
      return 0 == _mkdir(path.c_str());
#else
      return 0 == mkdir(path.c_str(), mode);
#endif

    case EEXIST:
      // done!
      return isDirExist(path);

    default:
      return false;
  }
}

int main(int argc, char** argv) {

  if (argc < 3) {
    std::cout << "Dump all frames of a video into a folder\n";
    std::cout
        << "Frames are named by their timestamps in the video in nanosecs\n";
    std::cout << "Usage: " << argv[0]
              << " input_video_name /output/image/folder [start_index(=0)] "
                 "[finish_index(inclusive)]\n";
    std::cout
        << "If you are convert the video for kalibr, set start_index such that "
           "the start from has a timestamp greater than 1 sec\n";
    std::cout << "to avoid timestamp less than 9 digits causing error in "
                 "kalibr_bagcreator\n";
    std::cout << "The output does not need to exist beforehand\n";
    std::cout << "Ex: " << argv[0]
              << " /home/user/Desktop/IMG_0658.MOV /home/user/Desktop/IMG_0658/"
              << std::endl;

    return 1;
  }

  int startIndex = 0, finishIndex = 1e6;
  if (argc > 3) {
    startIndex = std::atoi(argv[3]);
  }
  if (argc > 4) {
    finishIndex = std::atoi(argv[4]);
  }
  std::cout << "start and finish index " << startIndex << " " << finishIndex
            << std::endl;
  std::string videoName(argv[1]);
  std::string imageSeqPath = argv[2];
  makePath(argv[2]);

  std::string windowName = "DisplayVideo";
  cv::namedWindow(windowName, WINDOW_AUTOSIZE);

  int downscale = 1;
  cv::VideoCapture cap(videoName);
  char buffer[100];
  double rate = cap.get(CAP_PROP_FPS);
  if (!rate) cerr << "Error opening video file " << videoName << endl;

  assert(cap.get(CAP_PROP_POS_FRAMES) == 0);
  assert(cap.get(CAP_PROP_POS_MSEC) == 0);

  cap.set(CAP_PROP_POS_FRAMES, startIndex);  // start from a 0 based index
  int totalImages =
      std::min(finishIndex, (int)cap.get(CAP_PROP_FRAME_COUNT) - 1);
  int width = cap.get(CAP_PROP_FRAME_WIDTH),
      height = cap.get(CAP_PROP_FRAME_HEIGHT);

  cv::Mat left_img, dst, last_left_img;
  double time_frame(-1);  // unit milli sec
  double last_time_frame(-1);
  for (int numImages = startIndex; numImages <= totalImages; ++numImages) {
    int currentId = cap.get(CAP_PROP_POS_FRAMES);
    if (currentId != numImages) {
      cout << "current image video id 0 based " << currentId << " and counter "
           << numImages << endl;
    }
    time_frame = cap.get(CAP_PROP_POS_MSEC);

    cout << "frame time " << std::setprecision(16) << time_frame
         << "[ms] and index " << numImages;
    if (last_time_frame != -1) {
      std::cout << " " << (time_frame - last_time_frame) * 1000000 << "[ns]";
    }
    std::cout << std::endl;
    cap.read(left_img);
    if (left_img.empty()) {  // this happens when a frame is missed
      continue;
    }
    if (downscale > 1) {
      cv::pyrDown(left_img, dst, cv::Size((width + 1) / 2, (height + 1) / 2));
      left_img = dst;
    }
    if (numImages % 30 == 0)
      std::cout << "process image of index in the video " << numImages
                << std::endl;
    sprintf(buffer, "%.0f.png", time_frame * 1e6);
    string filename = imageSeqPath + '/' + buffer;
    imwrite(filename, left_img);
    imshow(windowName, left_img);
    waitKey(3);
    last_time_frame = time_frame;
  }
  cv::destroyWindow(windowName);
}
