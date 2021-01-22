#!/usr/bin/env python
import glob

import cv2
import numpy as np
import findCircles
import kNearestNeighbor

# def testVideoManager():
#     video = '../data/movie.mp4'
#     video_manager = VideoManager.VideoManager(video)
#     video_manager.play()
#     video_manager.close()


def testRollingShutterSkew():
    dir_name = '../data/'
    frame_files = glob.glob('%s/*.png' % dir_name)
    expectedCircleDist = [60, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55]
    for image_index, image_name in enumerate(frame_files):
        if image_index != 2:
            continue
        img = cv2.imread(image_name)
        img = findCircles.android_image_to_opencv_image(img)

        circles = findCircles.find_circles_mser(img)
        centers = np.zeros((len(circles), 2), dtype=np.float32)
        for index, c in enumerate(circles):
            centers[index, 0] = c.pt[0]
            centers[index, 1] = c.pt[1]
        circleDist = expectedCircleDist[image_index]
        coordinates, _ = kNearestNeighbor.assignCoordinatesToPoints(centers, circleDist)
        kNearestNeighbor.drawCoordinates(circles, coordinates, img)
