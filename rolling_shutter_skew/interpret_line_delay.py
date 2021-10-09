#!/usr/bin/env python3

"""
compute line delay with an image of a LED panel captured by a rolling shutter camera
"""

import argparse
import sys

import cv2
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

import ioUtilities

import calibrateCamera
import findCircles
import kNearestNeighbor
from point_to_parabola import pointToParabola
import VideoManager


class ArgumentParser(object):
    """Parses command line arguments for the rolling shutter test."""
    def __init__(self):
        self.__parser = argparse.ArgumentParser(
            description='Run rolling shutter test')
        self.__parser.add_argument(
            '-d',
            '--debug',
            action='store_true',
            help='print and write data useful for debugging')
        self.__parser.add_argument(
            '-v',
            '--video',
            help='Path to the LED panel video')
        self.__parser.add_argument(
            '-l',
            '--led_time',
            type=float,
            required=True,
            help=('how many milliseconds each column of the LED array is '
                  'lit for'))
        self.__parser.add_argument(
            '-o',
            '--debug_dir',
            help=('write debugging information in a folder in the '
                  'specified directory.  Otherwise, the system\'s default '
                  'location for temporary folders is used.  --debug must '
                  'be specified along with this argument.'))
        self.__parser.add_argument(
            '--roi',
            nargs=4,
            type=int,
            help=('region of interest: top, left, bottom, right. '
                  'Note for opencv image coordinates x axis is from left to right, y top to bottom.\n'
                  'If not provided, user will be prompted to drag a bounding box on the raw image.'))

        self.__parser.add_argument(
            '--frame_all_led_on',
            type=float,
            default=1.0,
            help=('Time of a frame with all LEDs on in the video. (default: %(default)s)'))
        self.__parser.add_argument(
            '--frame_rolling_led',
            type=float,
            default=20.0,
            help=('Time of a frame with columns of LEDs rolling in the video. (default: %(default)s)'))
        
        self.__parser.add_argument('--frame_pair', type=str, nargs=2, help=('path of the image with all leds on, and path of the image with rolling LEDs columnwise'))
        # --frame_pair image.jpg rolling.jpg --roi 260 328 632 714


    def parse_args(self):
        """Returns object containing parsed values from the command line."""
        return self.__parser.parse_args()


def drawCoordinates(circles, coordinates, img):
    """
    draw coordinates of circles on the image, (x, y), x for column, y for row.
    :param circles:
    :param coordinates:
    :param img:
    :return:
    """
    circle_img = img.copy()
    d_red = (65, 55, 150)
    l_red = (200, 200, 250)
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.35
    thickness = 1
    for index, f in enumerate(circles):
        cv2.circle(circle_img, (int(f.pt[0]), int(f.pt[1])), int(f.size / 2), d_red, 2, cv2.LINE_AA)
        cv2.circle(circle_img, (int(f.pt[0]), int(f.pt[1])), int(f.size / 2), l_red, 1, cv2.LINE_AA)
        cv2.putText(circle_img, '({:.0f},{:.0f})'.format(coordinates[index, 0], coordinates[index, 1]),
                    (int(f.pt[0]-15), int(f.pt[1]+20)), font, fontScale, (0, 255, 0), thickness, cv2.LINE_AA)

    h, w = img.shape[:2]
    vis = np.zeros((h, w * 2 + 5), np.uint8)
    vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
    vis[:h, :w] = img
    vis[:h, w + 5:w * 2 + 5] = circle_img

    cv2.imshow("Circle center coordinates", circle_img)
    # cv2.imwrite("coordinates.jpg", circle_img)
    cv2.waitKey()
    cv2.destroyAllWindows()


def drawlines(coef, hegiht, width):
    """
    draw coordinates of circles on the image, (x, y), x for column, y for row.
    :param circles:
    :param coordinates:
    :param img:
    :return:
    """
    fig, ax = plt.subplots()
    x = np.arange(0, width, 1)
    y_fit = np.polyval(coef, x)
    ax.plot(x, y_fit, 'g-')
    plt.show()

def drawlines_cv(coef_list, img, xmin_list, xmax_list):
    # x = row; y = col
    image = img.copy()
    height = image.shape[0]
    width = image.shape[1]
    # print(xmax_list)

    # print(height)
    ind_coef = 0
    for coef in coef_list:
        x = np.arange(xmin_list[ind_coef], xmax_list[ind_coef], 0.5)
        y_fit = np.polyval(coef, x)
        # print(coef)
        for index, xi in enumerate(x):
            if index == 0:
                x_pre = xi
                y_pre = y_fit[index]
                # print('({:.0f},{:.0f})'.format(x_pre, y_pre))
            else:
                # print('({:.0f},{:.0f})'.format(x_pre, y_pre))
                if y_pre<float(width) and y_fit[index]<float(width):
                    cv2.line(image, (int(y_pre), int(x_pre)), (int(y_fit[index]), int(xi)), (0,255,0), 1, 4)
                x_pre = xi
                y_pre = y_fit[index]
        ind_coef += 1
        
    cv2.imshow("image", image)
    cv2.waitKey()
    cv2.destroyAllWindows()


def drawPoints(pts, img):
    """

    :param pts: NX2 numpy array
    :param img:
    :return:
    """
    radius = 2
    # color in BGR
    color = (0, 255, 0)

    # Line thickness in px
    thickness = 1

    vis = img.copy()
    for pt in pts:
        vis = cv2.circle(vis, (pt[0], pt[1]), radius, color, thickness)

    cv2.imshow("Projected world points", vis)
    cv2.waitKey()


def find_boundary(coef, grey_img):
    '''
    For step 4, we need to optimize a function
    a,b,c = argmax mean(I(S_b(a, b, c))) - mean(I(S_d(a, b, c))) = argmax f(a, b, c)
    where a,b,c are the coefficients for the quadratic curve we are trying to fit to the boundary of
    the bright side S_b(a,b,c) and the dark side S_d(a, b, c), I(x) is the intensity at pixel x.
    '''

    height = grey_img.shape[0]
    width = grey_img.shape[1]
    total_diff = 0
    rows = 0

    # TODO(jhuai): implement as the pseudo code (binliang)
    for r in range(0, height, 1): # for each row, given the line equation, compute the intersection point on the row, say [c, r]
        c = int(np.polyval(coef, r) +0.5)
        cmin = max(c-50, 0)
        cmax = min(c+50, width)
        # if [c -50, c+ 50] overlap the row r with more than 50 pixels:
        m_bright = 0
        m_dark = 0
        if cmax-cmin > 50 and c-cmin > 20 and cmax-c > 20:
            # the left side from [c-50, r] to [c, r] is the bright side (check the image boundary), get the mean m_bright
            # the right side from [c, r] to [c+50, r] is the dark side, get the mean m_dark
            for i in range(cmin, cmax, 1):
                if i < c:
                    m_bright += grey_img[r, i]
                else:
                    m_dark += grey_img[r, i]
            # compute the difference d = m_bright - m_dark
            d = m_bright/float(c-cmin) - m_dark/float(cmax-c)
            total_diff += d
            rows += 1
    # compute the average overall difference
    if rows <= 0 or total_diff <= 0:
        return 1000.0
    avg_diff = 1.0/(total_diff / rows)
    return avg_diff


def get_intersection(coef_a, coef_b, x_min, x_max, coef_a_):
    x = np.arange(x_min, x_max, 0.005)
    ya_fit = np.polyval(coef_a, x)
    ya_fit_ = np.polyval(coef_a_, x)
    yb_fit = np.polyval(coef_b, x)

    for index, xi in enumerate(x):
        if abs(ya_fit[index]-yb_fit[index]) < 1 and abs(ya_fit[index]-ya_fit_[index]) < 50:
            print(ya_fit[index]-yb_fit[index])
            return int(xi+0.5),int(ya_fit[index]+0.5)

    return -1, -1


class BoundingBoxWidget(object):
    def __init__(self, image, winname):
        self.original_image = image
        self.clone = self.original_image.copy()
        self.winname = winname
        cv2.namedWindow(winname)
        cv2.setMouseCallback(winname, self.extract_coordinates)

        # Bounding box reference points
        self.image_coordinates = []

    def extract_coordinates(self, event, x, y, flags, parameters):
        # Record starting (x,y) coordinates on left mouse button click
        if event == cv2.EVENT_LBUTTONDOWN:
            self.image_coordinates = [(x,y)]

        # Record ending (x,y) coordintes on left mouse button release
        elif event == cv2.EVENT_LBUTTONUP:
            if x == self.image_coordinates[-1][0] and y == self.image_coordinates[-1][1]:
                self.image_coordinates = []
                return
            self.image_coordinates.append((x, y))
            print('top left bottom right: {} {} {} {}'.format(self.image_coordinates[0][1], self.image_coordinates[0][0],
                                                              self.image_coordinates[1][1], self.image_coordinates[1][0]))
            print('x,y,w,h : ({}, {}, {}, {})'.format(self.image_coordinates[0][0], self.image_coordinates[0][1],
                                                      self.image_coordinates[1][0] - self.image_coordinates[0][0],
                                                      self.image_coordinates[1][1] - self.image_coordinates[0][1]))

            # Draw rectangle
            cv2.rectangle(self.clone, self.image_coordinates[0], self.image_coordinates[1], (36,255,12), 2)
            cv2.imshow(self.winname, self.clone)

        # Clear drawing boxes on right mouse button click
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.clone = self.original_image.copy()

    def show_image(self):
        return self.clone


class TiltedBlock(object):
    """
    The block represents the left and right contours of bright blocks tilted to the right.
    """
    def __init__(self, frontierRow, frontier):
        self.frontierRow = frontierRow   # row of the frontier
        self.frontier = frontier  # the segment belonging to this block on the frontier row
        self.leftPointList = [[frontierRow, frontier[0]]]
        self.rightPointList = [[frontierRow, frontier[1]]]

    def getLeftContour(self):
        return np.array(self.leftPointList)

    def getRightContour(self):
        return np.array(self.rightPointList)

    def belongToBlock(self, rowId, newSegment, maxShift):
        """
        Does the new segment belong to this block?
        :param rowId:
        :param newSegment:
        :return:
        """
        # check distance between the newSegment and frontier
        dm = newSegment[0]+newSegment[1] - (self.frontier[0]+self.frontier[1])
        # IOU = (min(newSegment[1],self.frontier[1]) - max(newSegment[0],self.frontier[0])) / \
        #     (max(newSegment[1],self.frontier[1]) - min(newSegment[0],self.frontier[0]))
        # print(IOU)
        if abs(dm) > maxShift:
            return False
        return True

    def updateFrontier(self, rowId, newSegment = None):
        """

        :param rowId:
        :param newSegment:
        :return:
        """
        assert rowId >= self.frontierRow + 1
        self.frontierRow = rowId
        if newSegment:
            # remember that moving to the right is allowed, but not left,
            # if newSegment[0] < self.frontier[0] + 10:
            #     return
            # print(self.frontierRow)
            self.frontier = newSegment
            # also record the left and right points where moves to the right occur.
            # These left points (right points) are contours used for finding the min area rects.
            self.leftPointList.append([rowId, newSegment[0]])
            self.rightPointList.append([rowId, newSegment[1]])



def findSegmentsInRow(grayimage, row, mingap, minlen):
    """
    Find the bright segments in a row of grayimage, two segments should have minimum gap, otherwise, they are merged.
    The segment list may be empty for a row if all its pixels are dark.
    :param grayimage: maybe binary image?
    :param row:
    :param mingap: minimum gap between consecutive segments
    :param minlen: the minimum length of a segment. A small segment should be dismissed, like a single bright pixel.
    :return: segmentlist = [segment0, segment1]; segment0 = [l0, r0], segment1 = [l1, r1]
    """
    Segments = []
    width = grayimage.shape[1]
    beg = -1
    for c in range(width):
        if grayimage[row, c] == 0:
            if beg != -1:
                if c - beg > minlen:
                    Segments.append([beg, c-1])
                beg = -1
            continue
        if beg == -1:
            beg = c
    
    result = []
    for i in range(len(Segments)):
        if i == 0:
            result.append(Segments[i])
            continue
        if Segments[i][0]-result[-1][1] < mingap:
            result[-1][1] = Segments[i][1]
        else:
            result.append(Segments[i])

            
    return result


def compute_line_delay(image_all_leds_on, image_rolling_leds, led_time, roi, debug_dir):
    """
    calculate line delay given an image of a LED panel

    algorithm:
    1. detect circles of the LED panel on image_all_leds_on, and assign coordinates
    2. estimate the camera intrinsics and extrinsics with these cicle centers,
    3. rectifiy image_rolling_leds.
    4. find the tilted side of the LED light clusters, and fit a line to it, denote the line by L.
    5. the line delay can be computed by d = t_led / (tan(theta) * circle_distance)
     where theta is the angle between the line and the x axis.

    For step 4, we need to optimize a function
    a,b = argmax mean(I(S_b(a, b))) - mean(I(S_d(a, b))) = argmax f(a, b)
    where a,b are the coefficients for the line we are trying to fit to the boundary of
    the bright side S_b(a,b) and the dark side S_d(a, b), I(x) is the intensity at pixel x.
    You may use some optimization routines without computing gradient, referring to
    https://stackoverflow.com/questions/26739550/optimize-a-function-in-scipy-without-explicitly-defining-the-gradient
    You just need to define the cost function f.

    :param image_all_leds_on: image where all LEDs are on, h x w x 3
    :param image_rolling_leds: image where the LED columns are rolling on, h x w x 3
    :param led_time: the duration a LED light is bright.
    :param roi: region of interest, used to limit the area to search for the LED panel in image_all_leds_on
    :param debug_dir: directory to save the debug info
    :return: line_delay
    """
    height = image_all_leds_on.shape[0]
    width = image_all_leds_on.shape[1]
    assert image_all_leds_on.shape == image_rolling_leds.shape, \
        "The image with all LEDs on and the image of rolling columns of LEDs should have the same shape"

    # 1.detect circles on the LED panel
    circles = findCircles.find_circles_mser(image_all_leds_on, roi)
    numPoints = len(circles)
    # 2.identify coordinates of circles
    centers = np.zeros((numPoints, 2), dtype=np.float32)
    for index, c in enumerate(circles):
        centers[index, 0] = c.pt[0]
        centers[index, 1] = c.pt[1]
    coordinates, _ = kNearestNeighbor.assignCoordinatesToPoints(centers, -1)
    drawCoordinates(circles, coordinates, image_all_leds_on)

    objp = np.zeros((1, numPoints, 3), np.float32)
    corners = np.zeros((1, numPoints, 2), np.float32)
    for i in range(numPoints):
        objp[0, i, :2] = coordinates[i, :]
        corners[0, i, :] = centers[i, :]
    K, D, R_CW, t_CW = calibrateCamera.calibrate_camera(objp, corners, (height, width))
    undistorted_rolling_leds = calibrateCamera.undistort(K, D, R_CW, image_rolling_leds)

    # Visualize circles in the rectified image.
    projected = calibrateCamera.projectPoints(coordinates, R_CW, t_CW, K)
    drawPoints(projected, undistorted_rolling_leds)

    # find min and max world coordinates in order to get the ROI
    roi_rectified = kNearestNeighbor.boundingRect(coordinates)
    pad = 0.35 # The pad is set a bit larger than the radius of a circle considering that the circle distance is 1.
    lefttop = [roi_rectified[1] - pad, roi_rectified[0] - pad]
    rightbottom = [roi_rectified[3] + pad, roi_rectified[2] + pad]
    extremes = np.array([lefttop, rightbottom])
    projected = calibrateCamera.projectPoints(extremes, R_CW, t_CW, K)
    drawPoints(projected, undistorted_rolling_leds)
    roi_rectified_px = [projected[0, 1], projected[0, 0], projected[1, 1], projected[1, 0]]
    print("rectified top left bottom right {} in pixels".format(roi_rectified_px))

    # get circle distance in pixels in the rectified image
    circle_dist_x = (roi_rectified_px[2] - roi_rectified_px[0]) / (roi_rectified[2] - roi_rectified[0])
    circle_dist_y = (roi_rectified_px[3] - roi_rectified_px[1]) / (roi_rectified[3] - roi_rectified[1])
    circle_distance = (circle_dist_x + circle_dist_y) * 0.5
    print("Circle distance in the rectified image along x {} along y {} mean {}".format(circle_dist_x, circle_dist_y, circle_distance))

    # TODO(jhuai): Binliang crop the image with roi_rectified_px, and work on the cropped image from now on.
    # Because cropping does not affect the line delay estimate.
    image=undistorted_rolling_leds[int(roi_rectified_px[0]):int(roi_rectified_px[2]),
                    int(roi_rectified_px[1]):int(roi_rectified_px[3])]

    # TODO(jhuai): Binliang Select the red channel, and threshold to get the bright circles.
    crop_red = image[:, :, 0]
    if len(image.shape) < 3:
        crop_red = image
    elif image.shape[2] == 1:
        crop_red = image[:, :, 0]
    else:
        crop_red = image[:, :, 2]
    
    _, binary_red = cv2.threshold(crop_red, 127, 255, cv2.THRESH_TRIANGLE) # Binliang, try THRESH_BINARY, and THRESH_TRIANGLE to get the best result.
    cv2.imshow("binary_red", binary_red)
    cv2.waitKey()
    rectangle_red = cv2.cvtColor(binary_red, cv2.COLOR_GRAY2BGR)

    # Binliang:
    rows = binary_red.shape[0]
    cols = binary_red.shape[1]
    maxshift = 50
    blocklists = [] 
    # for each row of the rectified ROI
    for r in range(rows):
        # print(r)
        # find the bright segments assuming that the segments are separated by at least two circle distances,
        segmentlist = findSegmentsInRow(binary_red, r, 2*circle_distance, 10)
        # print(segmentlist)
        if segmentlist:
            # cv2.rectangle(rectangle_red, (segmentlist[-1][0], r), (segmentlist[-1][1], r), (0,255,0))
            # cv2.imshow("rectangle", rectangle_red)
            # cv2.waitKey()
            # merge the segmentlist to blocklists, i.e., update frontiers of blocks or initialize blocks
            if not blocklists:
                for seg in segmentlist:
                    blocklists.append(TiltedBlock(r, seg))
            else:
                for seg in segmentlist:
                    mark=0
                    for block in blocklists:
                        if block.belongToBlock(r, seg, maxshift):
                            mark=1
                            block.updateFrontier(r, seg)
                    if mark==0:
                        blocklists.append(TiltedBlock(r, seg))
        else:
            for block in blocklists:
                block.updateFrontier(r)


    # Binliang: initialize the boundary search with theta and intercept.
    # for each block
    coef_c = np.array([1.0, 0.0])
    coef1 = [0.0] *2
    coef2 = [0.0] *2
    for block in blocklists:
        # get the left contour, and fit the min area rect to the contour by cv2.minAreaRect
        # and compute theta and intercept
        pointList = np.array(block.leftPointList)
        rect1 = cv2.minAreaRect(pointList)
        box1 = cv2.boxPoints(rect1)
        cv2.drawContours(rectangle_red, [box1[:,[1,0]].astype(int)], 0, (0,255,0), 2)
        cv2.imshow("rectangle", rectangle_red)
        cv2.waitKey()

        # get the right contour, and fit the min area rect to the contour by cv2.minAreaRect
        # and compute theta and intercept.
        pointList = np.array(block.rightPointList)
        rect2 = cv2.minAreaRect(pointList)
        box2 = cv2.boxPoints(rect2)
        cv2.drawContours(rectangle_red, [box2[:,[1,0]].astype(int)], 0, (0,255,0), 2)
        cv2.imshow("rectangle", rectangle_red)
        cv2.waitKey()

        print('{},{}'.format(rect1[2],rect2[2]))
        # check the two theta's.
        if abs(rect1[2]-rect2[2]) < 5 and rect1[2] < -10:
            coef1[0] = ((box1[0][1]+box1[3][1])-(box1[1][1]+box1[2][1])) / \
                ((box1[0][0]+box1[3][0])-(box1[1][0]+box1[2][0]))
            coef1[1] = (box1[0][1]+box1[3][1])/2 - coef1[0]*(box1[0][0]+box1[3][0])/2
            coef2[0] = ((box2[0][1]+box2[3][1])-(box2[1][1]+box2[2][1])) / \
                ((box2[0][0]+box2[3][0])-(box2[1][0]+box2[2][0]))
            coef2[1] = (box2[0][1]+box2[3][1])/2 - coef2[0]*(box2[0][0]+box2[3][0])/2
            # coef_c[0] = (coef1[0]+coef2[0])/2
            # coef_c[1] = (coef1[1]+coef2[1])/2
            coef_c = coef2
            break


    # refine the boundary line by scipy optimize
    drawlines_cv([coef_c], image, [0], [height])
    # Use bounds to limit the search region.
    res = minimize(find_boundary, coef_c, (crop_red), method='powell', options={'xtol': 1e-4, 'disp': True})
    drawlines_cv([res.x], image, [0], [height])

    # d = t_led / (tan(theta) * circle_distance)
    linedelay = led_time / (coef_c[0] * circle_distance)
    return linedelay


def main():
    """
    run example: interpret_line_delay.py -l 0.03 -i 2020_07_15_19_21_19/raw/00010.jpg -d
    :return:
    """
    global DEBUG
    parser = ArgumentParser()
    args = parser.parse_args()
    DEBUG = args.debug
    if not DEBUG and args.debug_dir:
        print('argument --debug_dir requires --debug')
        sys.exit()
    if args.video:
        video_manager = VideoManager.VideoManager(args.video)
        if not video_manager.isOpened():
            video_manager.close()
            print("Failed to open video {}".format(args.video))
            return

        frame_all_led_on_id = video_manager.rate * args.frame_all_led_on
        frame_rolling_led_id = video_manager.rate * args.frame_rolling_led

        _, image_all_led = video_manager.frameAt(frame_all_led_on_id)
        _, image_rolling_led = video_manager.frameAt(frame_rolling_led_id)

        video_manager.close()
    elif args.frame_pair:
        image_all_led = cv2.imread(args.frame_pair[0])
        image_rolling_led = cv2.imread(args.frame_pair[1])
    
    winname = "Please drag the bounding box, then press q to quit."
    if args.roi and len(args.roi) == 4:
        pass
    else:
        boundingbox_widget = BoundingBoxWidget(image_all_led, winname)
        while True:
            cv2.imshow(winname, boundingbox_widget.show_image())
            key = cv2.waitKey(1)

            # Close program with keyboard 'q'
            if key == ord('q'):
                cv2.destroyAllWindows()
                break

        left = boundingbox_widget.image_coordinates[-2][0]
        top = boundingbox_widget.image_coordinates[-2][1]
        right = boundingbox_widget.image_coordinates[-1][0]
        bottom = boundingbox_widget.image_coordinates[-1][1]
        args.roi = [top, left, bottom, right]
    print("args roi {}".format(args.roi))

    linedelay = compute_line_delay(image_all_led, image_rolling_led, args.led_time, args.roi, args.debug_dir)
    if DEBUG:
        print('Found line delay {} for {}'.format(linedelay, args.img))

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
