import glob
import cv2
import numpy as np
import findCircles
import kNearestNeighbor


def testRollingShutterSkew():
    dir_name = '../data/'
    frame_files = glob.glob('%s/*.png' % dir_name)
    expectedCircleDist = [60, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55]
    for image_index, image_name in enumerate(frame_files):
        if image_index != 9:
            continue
        img = cv2.imread(image_name)
        img = findCircles.android_image_to_opencv_image(img)
        if img.shape[2] > 1:
            red_img = img[:, :, 2]
        else:
            red_img = img[:, :, 0]

        circles = findCircles.find_circles_mser(img)
        centers = np.zeros((len(circles), 2), dtype=np.float32)
        for index, c in enumerate(circles):
            centers[index, 0] = c.pt[0]
            centers[index, 1] = c.pt[1]
        circleDist = expectedCircleDist[image_index]
        coordinates = kNearestNeighbor.assignCoordinatesToPoints(centers, circleDist)
        kNearestNeighbor.drawCoordinates(circles, coordinates, img)
