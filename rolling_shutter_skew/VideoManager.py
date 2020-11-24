#!/usr/bin/env python
import math

import cv2


class VideoManager(object):
    def __init__(self, video_filename, rate_multiplier=1.0):
        self.cap = cv2.VideoCapture(video_filename)
        rate = self.cap.get(cv2.CAP_PROP_FPS)
        print("video frame rate {} in file".format(rate))
        if rate < 5:
            rate = 25
        rate *= rate_multiplier
        self.interval_ms = int(math.floor(1000 / rate))

        start_id = 0
        finish_id = 1e6
        self.cap.set(cv2.CAP_PROP_POS_FRAMES,
                start_id)  # start from start_id, 0 based index
        if finish_id == -1:
            finish_id = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT) - 1)
        else:
            finish_id = min(finish_id, int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT) - 1))
        print('start {} finish {}'.format(start_id, finish_id))
        self.finish_id = finish_id
        self.current_id = start_id

    def play(self):
        while self.cap.isOpened():
            status, frame = self.nextFrame()
            if not status:
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


            cv2.imshow('frame', gray)
            if cv2.waitKey(self.interval_ms) & 0xFF == ord('q'):
                break

    def isOpened(self):
        return self.cap.isOpened()


    def nextFrame(self):
        video_frame_id = self.cap.get(cv2.CAP_PROP_POS_FRAMES)
        if video_frame_id != self.current_id:
            print("Expected frame id {} and actual one in video {} differ.".
                  format(self.current_id, video_frame_id))
            print("Likely reached end of video file. Note finish_id {}".format(
                self.finish_id))
            return False, None

        # time_frame= cap.get(cv2.CAP_PROP_POS_MSEC)/1000.0
        # print('currentFrameId {} and timestamp in video {:.9f}'.
        # format(current_id, time_frame))
        _, frame = self.cap.read()
        h, w = frame.shape[:2]
        if h > w:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        if frame is None:
            print('Empty frame, break the video stream')
            return False, None

        self.current_id += 1
        if self.current_id > self.finish_id:
            print('Exceeding finish_id %d' % self.finish_id +
                  ', break the video stream')
            return False, None
        return True, frame

    def close(self):
        self.cap.release()
        cv2.destroyAllWindows()
