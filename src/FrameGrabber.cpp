
#include "vio/FrameGrabber.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "vio/utils.h"
using namespace std;

namespace vio {

using namespace cv;

FrameGrabber::FrameGrabber(const std::string visualDataPath,
                           const std::string frameTimeFile, int startFrameIndex,
                           int finishFrameIndex, int maxFrameHeight)
    : is_measurement_good(false),
      mTimestampFile(frameTimeFile),
      mTG(frameTimeFile),
      mTk(-1),
      mnStartId(startFrameIndex),
      mnFinishId(finishFrameIndex),
      mnCurrentId(mnStartId),
      mbRGB(false) {
  if (isVideoFile(visualDataPath)) {
    mVideoFile = visualDataPath;
    mCapture = cv::VideoCapture(mVideoFile);
    experim = CrowdSourcedData;
    double rate = mCapture.get(CAP_PROP_FPS);
    if (!rate) std::cerr << "Error opening video file " << mVideoFile << endl;
    mCapture.set(CAP_PROP_POS_FRAMES,
                 mnStartId);  // start from mnStartId, 0 based index
    if (mnFinishId == -1) {
      mnFinishId = (int)(mCapture.get(CAP_PROP_FRAME_COUNT) - 1);
    } else {
      mnFinishId = std::min(mnFinishId,
                            (int)(mCapture.get(CAP_PROP_FRAME_COUNT) - 1));
    }
    int width = mCapture.get(CAP_PROP_FRAME_WIDTH),
        height = mCapture.get(CAP_PROP_FRAME_HEIGHT);
    if (maxFrameHeight > 0) {
      mnDownScale = getDownScale(width, height, maxFrameHeight);
    } else {
      mnDownScale = 1;
    }
    std::cout << "Reading video " << mVideoFile << " prop_frame_width "
              << width << " prop_frame_height " << height << " mnDownScale "
              << mnDownScale << "." << std::endl;
  } else {
    mImageFolder = visualDataPath;
    mnDownScale = 1;
    experim = Tsukuba;
    std::cout << "Reading images in folder " << mImageFolder << std::endl;
  }

  std::cout << "Frame start index " << startFrameIndex << ", finish index "
            << finishFrameIndex << ".\n";
}

bool FrameGrabber::is_open() {
  if (mImageFolder.empty()) {
    return mCapture.isOpened();
  } else {
    return dirExist(mImageFolder);
  }
}

/**
 * @brief FrameGrabber::assignTimeToVideoFrame
 * @param frame_id 0 based index of the frame in a video
 * @param videoTime the time stored in the video, usually estimated from the
 * frame rate
 * @return
 */
double FrameGrabber::assignTimeToVideoFrame(int frame_id, double videoTimeSec) {
  double time_frame;
  if (mTG.isTimeAvailable()) {
    time_frame = mTG.readTimestamp(frame_id);
  } else {
    time_frame = videoTimeSec;
  }
  return time_frame;
}

static void rotateIfNeeded(cv::Mat* src) {
  if (src->cols < src->rows) {
    cv::rotate(*src, *src, cv::ROTATE_90_COUNTERCLOCKWISE);
  }
}

bool FrameGrabber::grabFrame(cv::Mat& left_img, double& tk) {
  double time_frame(-1);  // timestamp of current frame
  double time_pair[2] = {-1,
                         mTk};  // timestamps of the previous and current images
  if (mnCurrentId > mnFinishId) {
    left_img.release();
    return false;
  }

  if (experim == CrowdSourcedData) {
    cv::Mat dst;
    assert(mCapture.get(CAP_PROP_POS_FRAMES) == mnCurrentId);
    double videoTimeSec = mCapture.get(CAP_PROP_POS_MSEC) / 1000.0;
    time_frame = assignTimeToVideoFrame(mnCurrentId, videoTimeSec);
    mCapture.read(left_img);
    rotateIfNeeded(&left_img);

    while (left_img.empty()) {  // this happens when a frame is missing or at
                                // the end of video file
      ++mnCurrentId;
      if (mnCurrentId > mnFinishId) {
        return false;
      }
      int videoFrameId = mCapture.get(CAP_PROP_POS_FRAMES);
      if (videoFrameId != mnCurrentId) {
        std::cerr << "Expected frame id " << mnCurrentId
                  << " and actual one in video " << videoFrameId << " differ. "
                  << std::endl;
        std::cerr << "Likely reached end of video file. Note mnFinishId "
                  << mnFinishId << std::endl;
        return false;
      }
      double videoTimeSec = mCapture.get(CAP_PROP_POS_MSEC) / 1000.0;
      time_frame = assignTimeToVideoFrame(mnCurrentId, videoTimeSec);
      mCapture.read(left_img);
      rotateIfNeeded(&left_img);
    }

    if (mnDownScale > 1) {
      cv::pyrDown(left_img, dst);
      left_img = dst;
    }
    time_pair[0] = time_pair[1];
    time_pair[1] = time_frame;

    if (left_img.channels() == 3) {
      cv::Mat temp;
      if (mbRGB)
        cvtColor(left_img, temp, COLOR_RGB2GRAY);
      else
        cvtColor(left_img, temp, COLOR_BGR2GRAY);
      left_img = temp;
    }
  } else {
    // for other types of image sequences
    char base_name[256];  // input file names
    string left_img_file_name;
    string right_img_file_name;

    switch (experim) {
      case KITTIOdoSeq:
        sprintf(base_name, "%06d.png", mnCurrentId);
        left_img_file_name = mImageFolder + "/image_0/" + base_name;
        right_img_file_name = mImageFolder + "/image_1/" + base_name;
        time_frame = mTG.readTimestamp(mnCurrentId);
        break;
      case Tsukuba:
        sprintf(base_name, "%05d.png", mnCurrentId + 1);
        left_img_file_name = mImageFolder + "/tsukuba_daylight_L_" + base_name;
        right_img_file_name = mImageFolder + "/tsukuba_daylight_R_" + base_name;
        // time_frame=mnCurrentId/30.0;
        time_frame = mTG.readTimestamp(mnCurrentId);
        break;
      case MalagaUrbanExtract6:
        time_frame = mTG.extractTimestamp(mnCurrentId);
        left_img_file_name = mTG.last_left_image_name;
        right_img_file_name =
            left_img_file_name.substr(0, 30) + "right" +
            left_img_file_name.substr(left_img_file_name.length() - 4, 4);
        left_img_file_name = mImageFolder + "/" + left_img_file_name;
        right_img_file_name = mImageFolder + "/" + right_img_file_name;
      default:
        std::cerr << "Please implement interface fot this dataset!"
                  << std::endl;
        break;
    }
    time_pair[0] = time_pair[1];
    time_pair[1] = time_frame;
    left_img = cv::imread(left_img_file_name, 0);
    //                 cv::Mat right_img=cv::imread(right_img_file_name, 0);
  }
  tk = time_frame;
  mTk = tk;
  ++mnCurrentId;
  return true;
}

bool FrameGrabber::grabFrameByIndex(const int queryIndex, cv::Mat& left_img,
                                    double& tk) {
  double time_frame(-1);  // timestamp of current frame
  double time_pair[2] = {-1,
                         0};  // timestamps of the previous and current images
  if (queryIndex > mnFinishId || queryIndex < mnStartId) {
    std::cerr << "Trying to grab a frame of index " << queryIndex
              << " outside the range [" << mnStartId << "," << mnFinishId << "]"
              << std::endl;
    return false;
  }

  if (experim == CrowdSourcedData) {
    cv::Mat dst;
    mCapture.set(CAP_PROP_POS_FRAMES, queryIndex);
    assert(mCapture.get(CAP_PROP_POS_FRAMES) == queryIndex);
    time_frame = mCapture.get(CAP_PROP_POS_MSEC) / 1000.0;
    mCapture.read(left_img);

    if (left_img.empty()) {  // this happens when a frame is missing or at the
                             // end of video file

      std::cerr << "Empty frame at " << queryIndex << " in video " << std::endl;
      std::cerr << "Likely reached end of video file. Note mnFinishId "
                << mnFinishId << std::endl;
      return false;
    }

    if (mnDownScale > 1) {
      cv::pyrDown(left_img, dst);
      left_img = dst;
    }
    time_pair[0] = time_pair[1];
    time_pair[1] = time_frame;

    if (left_img.channels() == 3) {
      cv::Mat temp;
      if (mbRGB)
        cvtColor(left_img, temp, COLOR_RGB2GRAY);
      else
        cvtColor(left_img, temp, COLOR_BGR2GRAY);
      left_img = temp;
    }
  } else {
    // TODO: timegrabber does not support random reading timestamps
    // for other types of image sequences
    char base_name[256];  // input file names
    string left_img_file_name;
    string right_img_file_name;

    switch (experim) {
      case KITTIOdoSeq:
        sprintf(base_name, "%06d.png", queryIndex);
        left_img_file_name = mImageFolder + "/image_0/" + base_name;
        right_img_file_name = mImageFolder + "/image_1/" + base_name;
        // time_frame=mTG.readTimestamp(queryIndex);
        break;
      case Tsukuba:
        sprintf(base_name, "%05d.png", queryIndex + 1);
        left_img_file_name = mImageFolder + "/tsukuba_daylight_L_" + base_name;
        right_img_file_name = mImageFolder + "/tsukuba_daylight_R_" + base_name;
        // time_frame=queryIndex/30.0;
        // time_frame=mTG.readTimestamp(queryIndex);
        break;
      case MalagaUrbanExtract6:
        // time_frame=mTG.extractTimestamp(queryIndex);
        left_img_file_name = mTG.last_left_image_name;
        right_img_file_name =
            left_img_file_name.substr(0, 30) + "right" +
            left_img_file_name.substr(left_img_file_name.length() - 4, 4);
        left_img_file_name = mImageFolder + "/" + left_img_file_name;
        right_img_file_name = mImageFolder + "/" + right_img_file_name;
      default:
        std::cerr << "Please implement interface fot this dataset!"
                  << std::endl;
        break;
    }
    time_pair[0] = time_pair[1];
    time_pair[1] = time_frame;
    left_img = cv::imread(left_img_file_name, 0);
    //                 cv::Mat right_img=cv::imread(right_img_file_name, 0);
  }
  tk = time_frame;
  return true;
}
}  // namespace vio
