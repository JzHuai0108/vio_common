#!/usr/bin/env python
import math

import cv2
import numpy as np

def android_image_to_opencv_image(img):
    if img.dtype == 'uint8':
        return img
    img *= 255
    img = img.astype(np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img


def in_circle(center_x, center_y, radius, x, y):
    square_dist = (center_x - x) ** 2 + (center_y - y) ** 2
    return square_dist < radius ** 2


def coherency(x, gray_img):
    intensities = list()
    h = gray_img.shape[0]
    w = gray_img.shape[1]
    # in opencv Point(x, y) is for (column, row)
    # see https://stackoverflow.com/questions/25642532/opencv-pointx-y-represent-column-row-or-row-column
    radius = int(x.size / 2) + 1
    top = max(0, int(x.pt[1]) - radius)
    bottom = min(h, int(x.pt[1]) + radius + 1)
    left = max(0, int(x.pt[0]) - radius) # inclusive
    right = min(w, int(x.pt[0]) + radius + 1) # not inclusive
    for i in range(top, bottom):
        for j in range(left, right):
            intensities.append(gray_img[i, j])
    return np.std(intensities)


def suppress(fs, x):
    """
    check if to suppress x based on circles in fs.
    :param x:
    :return:
    """
    for f in fs:
        distx = f.pt[0] - x.pt[0]
        disty = f.pt[1] - x.pt[1]
        dist = math.sqrt(distx * distx + disty * disty)
        if (f.size > x.size) and (dist < f.size / 2):
            return True

def find_circles_mser(img, roi = None, cull_by_size = True, draw_result = True):
    """
    find circles in the red channel of an image with the MSER.
    :param img: image should have 3 dimens
    :return: a list of detected circles
    See https://stackoverflow.com/questions/9860667/writing-robust-color-and-size-invariant-circle-detection-with-opencv-based-on
    When the detected circle has a imprecise radius, circle center may be accurate if the circle is well distributed.
    """

    if len(img.shape) < 3:
        red_img = img
    elif img.shape[2] == 1:
        red_img = img[:, :, 0]
    else:
        red_img = img[:, :, 2]

    # equ = cv2.equalizeHist(red_img) # equalize leads to worse circle detections,
    # sometimes more false positives, sometimes more false negatives.

    is_v2 = cv2.__version__.startswith("2.")
    if is_v2:
        detector = cv2.MSER()
    else:
        detector = cv2.MSER_create()

    fs = detector.detect(red_img) # use 3 channel original image causes worse results.
    fs.sort(key=lambda x: -x.size)

    sfs = [x for x in fs if not suppress(fs, x)]

    if roi:
        leftRegions = []
        for x in sfs:
            if roi[1] < x.pt[0] < roi[3] and roi[0] < x.pt[1] < roi[2]:
                leftRegions.append(x)
        sfs = leftRegions

    if cull_by_size:
        # remove circles too small or too large
        expectedDiameter = 30
        fraction = 0.75
        medianDiameter = min(max(np.median([x.size for x in sfs]), expectedDiameter),
                             expectedDiameter / fraction)

        sfs = [x for x in sfs if x.size < medianDiameter / fraction and x.size > medianDiameter * fraction]

    # coherentRegions = []
    # for x in sfs:
    #     if coherency(x, red_img) < 50:
    #         coherentRegions.append(x)
    # sfs = coherentRegions

    if draw_result:
        circle_img = img.copy()
        d_red = (65, 55, 150)
        l_red = (200, 200, 250)
        for f in sfs:
            cv2.circle(circle_img, (int(f.pt[0]), int(f.pt[1])), int(f.size / 2), d_red, 2, cv2.LINE_AA)
            cv2.circle(circle_img, (int(f.pt[0]), int(f.pt[1])), int(f.size / 2), l_red, 1, cv2.LINE_AA)

        h, w = img.shape[:2]
        vis = np.zeros((h, w * 2 + 5), np.uint8)
        vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
        vis[:h, :w] = img
        vis[:h, w + 5:w * 2 + 5] = circle_img

        cv2.imshow("Detected circles", circle_img)
        # cv2.imwrite("circles.jpg", circle_img)
        cv2.waitKey()

    return sfs


def find_circles_SimpleBlobDetector(image):
    """
    Find circles in the red channel of an image with the SimpleBlobDetector.
    :param image:
    :return:
    See https://www.geeksforgeeks.org/find-circles-and-ellipses-in-an-image-using-opencv-python/
    and https://www.learnopencv.com/blob-detection-using-opencv-python-c/.
    With preliminary tuning, this method does not work well with LED panel images.
    """
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 30
    params.maxThreshold = 255

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 40

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.2

    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.01

    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3:
        detector = cv2.SimpleBlobDetector(params)
    else:
        detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs
    if image.shape[2] > 1:
        red_img = image[:, :, 2]
    else:
        red_img = image[:, :, 0]
    keypoints = detector.detect(red_img)

    # Draw blobs on our image as red circles
    blank = np.zeros((1, 1))
    blobs = cv2.drawKeypoints(image, keypoints, blank, (0, 0, 255),
                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    number_of_blobs = len(keypoints)
    text = "Number of Circular Blobs: " + str(len(keypoints))
    cv2.putText(blobs, text, (20, 550),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 255), 2)

    # Show blobs
    cv2.imshow("Filtering Circular Blobs Only", blobs)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return keypoints

