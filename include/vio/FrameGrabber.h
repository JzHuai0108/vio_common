#ifndef FRAMEGRABBER_H
#define FRAMEGRABBER_H

#include <fstream>
#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "vio/TimeGrabber.h"  //for timegrabber
namespace vio {
class FrameGrabber {
 public:
  /**
   * @brief FrameGrabber
   * @param visualDataPath: either the full path of a video or
   *    the dir containing the left and right image sequence without slash
   * @param frameTimeFile file containing the timestamps of every frames
   *    in the video or every images in the dir. If empty, the timestamps
   *    will be assigned empirically using video frame rate
   * @param startFrameIndex 0 based starting frame index within the video or
   *    the image dir
   * @param finishFrameIndex 0 based finishing frame index within the video
   *    or the image dir. If not given, the whole video or the image
   *    sequence is processed
   */
  FrameGrabber(const std::string visualDataPath,
               const std::string frameTimeFile, const int startFrameIndex,
               const int finishFrameIndex = -1);

  FrameGrabber(const FrameGrabber &) = delete;
  FrameGrabber &operator=(const FrameGrabber &) = delete;
  ~FrameGrabber() {}
  bool is_open();
  /**
   * @brief FrameGrabber::assignTimeToVideoFrame
   * @param frame_id 0 based index of the frame in a video
   * @param videoTimeSec the time stored in the video, usually estimated from
   * the frame rate
   * @return the assigned timestamp for frame_id
   */
  double assignTimeToVideoFrame(int frame_id, double videoTimeSec);

  /**
   * @brief Get next frame and its timestamp
   * @param frame grabbed frame
   * @param tk timestamp of the grabbed frame
   */
  bool grabFrame(cv::Mat &frame, double &tk);
  int getCurrentId() {
    return mnCurrentId - 1;
  }  // because it's incremented once a frame is obtained

  // queryIndex is zero based, currently only supports video not image sequences
  // DEPRECATED
  bool grabFrameByIndex(const int queryIndex, cv::Mat &left_img, double &tk);

 protected:
  bool is_measurement_good;  // does the measurement fit our requirements?
  std::string mVideoFile;
  cv::VideoCapture mCapture;

  std::string mImageFolder;
  std::string mTimestampFile;

  TimeGrabber mTG;
  // tk, timestamp of the current frame, i.e., the last grabbed frame
  double mTk;

  const int mnStartId;  /// 0 based starting frame index within the video or the
                        /// image directory
  int mnFinishId;   /// 0 based finishing frame index within the video or the
                    /// image directory
  int mnCurrentId;  /// current frame index 0 based starting frame index within
                    /// the video or the image directory
  int mnDownScale;
  bool mbRGB;  /// if color, RGB or BGR
  enum DatasetType {
    KITTIOdoSeq = 0,
    Tsukuba,
    MalagaUrbanExtract6,
    CrowdSourcedData
  };
  DatasetType experim;
};
}  // namespace vio
#endif  // FRAMEGRABBER_H
