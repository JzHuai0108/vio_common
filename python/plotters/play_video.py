#!/usr/bin/env python3
# Play a video with opencv modules
# This script works with python2 or python3

import math
import sys
import cv2


def play_video(video_filename, rate_multiplier):
    cap = cv2.VideoCapture(video_filename)
    rate = cap.get(cv2.CAP_PROP_FPS)
    print("video frame rate {} in file".format(rate))
    if rate < 5:
        rate = 25
    rate *= rate_multiplier
    interval_ms = int(math.floor(1000 / rate))
    start_id = 0
    finish_id = 1e6
    cap.set(cv2.CAP_PROP_POS_FRAMES,
            start_id)  # start from start_id, 0 based index
    if finish_id == -1:
        finish_id = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) - 1)
    else:
        finish_id = min(finish_id, int(cap.get(cv2.CAP_PROP_FRAME_COUNT) - 1))
    print('start {} finish {}'.format(start_id, finish_id))

    current_id = start_id

    while cap.isOpened():
        video_frame_id = cap.get(cv2.CAP_PROP_POS_FRAMES)
        if video_frame_id != current_id:
            print("Expected frame id {} and actual one in video {} differ.".
                  format(current_id, video_frame_id))
            print("Likely reached end of video file. Note finish_id {}".format(
                finish_id))
            break

        # time_frame= cap.get(cv2.CAP_PROP_POS_MSEC)/1000.0
        # print('currentFrameId {} and timestamp in video {:.9f}'.
        # format(current_id, time_frame))
        _, frame = cap.read()
        if frame is None:
            print('Empty frame, break the video stream')
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # h, w = gray.shape[:2]

        cv2.imshow('frame', gray)
        if cv2.waitKey(interval_ms) & 0xFF == ord('q'):
            break
        current_id += 1
        if current_id > finish_id:
            print('Exceeding finish_id %d' % finish_id +
                  ', break the video stream')
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    if len(sys.argv) < 2:
        print("Usage: {} video_name [rate]".format(sys.argv[0]))
        print('rate is play rate factor, e.g., 2 for 2 times faster')
        print('Press q to exit play')
        return
    rate = 1.0
    if len(sys.argv) > 2:
        rate = float(sys.argv[2])
    play_video(sys.argv[1], rate)


if __name__ == "__main__":
    main()
