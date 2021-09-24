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

import findCircles
import kNearestNeighbor
from point_to_parabola import pointToParabola

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
            '-i',
            '--img',
            help='Path to the LED panel image')
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

    cv2.imshow("image", vis)
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
    image = img.copy()
    height = image.shape[0]
    width = image.shape[1]
    # print(xmax_list)

    print(height)
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
                if y_pre<float(height) and y_fit[index]<float(height):
                    cv2.line(image, (int(x_pre), int(y_pre)), (int(xi), int(y_fit[index])), (0,255,0), 1, 4)
                x_pre = xi
                y_pre = y_fit[index]
        ind_coef += 1
        
    cv2.imshow("image", image)
    cv2.waitKey()
    cv2.destroyAllWindows()


def find_boundary(coef, image, xmin, xmax, ymin, ymax):
    '''
    For step 4, we need to optimize a function
    a,b,c = argmax mean(I(S_b(a, b, c))) - mean(I(S_d(a, b, c))) = argmax f(a, b, c)
    where a,b,c are the coefficients for the quadratic curve we are trying to fit to the boundary of
    the bright side S_b(a,b,c) and the dark side S_d(a, b, c), I(x) is the intensity at pixel x.
    '''
    if len(image.shape) < 3:
        grey_img = image
    elif image.shape[2] == 1:
        grey_img = image[:, :, 0]
    else:
        grey_img = image[:, :, 2]
    
    # cv2.line(image, (int(xmin), int(ymin)), (int(xmin), int(ymax)), (0,255,0), 1, 4)
    # cv2.line(image, (int(xmin), int(ymin)), (int(xmax), int(ymin)), (0,255,0), 1, 4)
    # cv2.line(image, (int(xmin), int(ymax)), (int(xmax), int(ymax)), (0,255,0), 1, 4)
    # cv2.line(image, (int(xmax), int(ymin)), (int(xmax), int(ymax)), (0,255,0), 1, 4)
    # cv2.imshow("image", grey_img)
    # cv2.waitKey()
    # cv2.destroyAllWindows()

    height = grey_img.shape[0]
    width = grey_img.shape[1]
    I_sb = 0.0
    n_sb = 0.0
    I_sd = 0.0
    n_sd = 0.0
    for r in range(ymin, ymax, 1):
        for c in range(xmin, xmax, 1):
            # two sides of the curve C: s_b and s_c
            cand, dist = pointToParabola(coef, (c, r))
            if r >= np.polyval(coef, c) and dist < 100: #and r<np.polyval(coef, c)+100:
                I_sb += grey_img[r,c]
                n_sb += 1
            elif r < np.polyval(coef, c) and dist < 100: #and r>=np.polyval(coef, c)-100:
                I_sd += grey_img[r,c]
                n_sd += 1
    if n_sb<=10 or n_sd<=10:
        print(1000)
        return 1000.0
    res = abs(I_sb/n_sb - I_sd/n_sd)
    if res > 0:
        res = 1.0/res
    else:
        res = 1000
    print(res)
    return res

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

def rosen(x, image):
    """The Rosenbrock function"""
    return sum(100.0*(x[1:]-x[:-1]**2.0)**2.0 + (1-x[:-1])**2.0)

def compute_line_delay(image, led_time, debug_dir):
    """
    calculate line delay given an image of a LED panel
    assumptions:
    1. The image is roughly aligned to the LED panel, though there may be lens distortion

    algorithm:
    1. detect circles on the LED panel
    2. identify columns of circles by using assumption 1.
    3. fit quadratic curves to at least two columns of circles, say column A and column B. It's better to have a large
      gap between the two columns.
    4. find the tilted side of the LED light clusters, and fit a quadratic curve to it, denote the curve by C.
    5. find the intersection of column A and curve C, denoted by (r1, c1), and the intersection of column B and curve C,
     denoted by (r2, c2), then the line_delay * (r2 - r1) = led_time * (B - A)

    For step 1, refer to find_circles_mser in this repo and
    https://scikit-image.org/docs/dev/auto_examples/features_detection/plot_blob.html

    For step 2, refer to assignCoordinatesToPoints in this repo
    You may have a better way to do this, be creative.

    Step 3 is fairly easy.

    For step 4, we need to optimize a function
    a,b,c = argmax mean(I(S_b(a, b, c))) - mean(I(S_d(a, b, c))) = argmax f(a, b, c)
    where a,b,c are the coefficients for the quadratic curve we are trying to fit to the boundary of
    the bright side S_b(a,b,c) and the dark side S_d(a, b, c), I(x) is the intensity at pixel x.
    You may use some optimization routines without computing gradient, referring to
    https://stackoverflow.com/questions/26739550/optimize-a-function-in-scipy-without-explicitly-defining-the-gradient
    You just need to define the cost function f.

    Step 5 is fairly easy.

    :param image:
    :param led_time:
    :param debug_dir:
    :return: line_delay
    """
    height = image.shape[0]
    width = image.shape[1]
    # 1.detect circles on the LED panel
    circles = findCircles.find_circles_mser(image)
    # print(sfs)

    # 2.identify columns of circles by using assumption 1.
    centers = np.zeros((len(circles), 2), dtype=np.float32)
    for index, c in enumerate(circles):
        centers[index, 0] = c.pt[0]
        centers[index, 1] = c.pt[1]
    coordinates, dxy = kNearestNeighbor.assignCoordinatesToPoints(centers, 45)
    drawCoordinates(circles, coordinates, image)
    # print(dxy)

    # 3. fit quadratic curves to at least two columns of circles, say column A and column B. It's better to have a large
    # gap between the two columns.
    minC = 1000
    maxC = -1000
    xmin = ymin = 1000000
    xmax = ymax = -1000000
    for index, f in enumerate(circles):
        if coordinates[index, 0] == 1000:
            continue
        if coordinates[index, 0] < minC:
            minC = coordinates[index, 0]
        if coordinates[index, 0] > maxC:
            maxC = coordinates[index, 0]
        if f.pt[0] < xmin:
            xmin = int(f.pt[0])
        if f.pt[0] > xmax:
            xmax = int(f.pt[0])
        if f.pt[1] < ymin:
            ymin = int(f.pt[1])
        if f.pt[1] > ymax:
            ymax = int(f.pt[1])
    
    col_a = minC
    col_b = maxC
    if maxC-minC > 4:
        col_a += 1
        col_b -= 1

    xy_a = []
    xy_b = []
    for index, f in enumerate(circles):
        if coordinates[index, 0] == col_a:
            xy_a.append([f.pt[0],f.pt[1]])
        if coordinates[index, 0] == col_b:
            xy_b.append([f.pt[0],f.pt[1]])
    xy_a=np.array(xy_a)
    xy_a = xy_a[np.lexsort([xy_a[:,0]]), :]
    xy_b=np.array(xy_b)
    xy_b = xy_b[np.lexsort([xy_b[:,0]]), :]
    # fig, ax = plt.subplots()
    # ax.plot(xy_a[:,0], xy_a[:,1], 'bx')
    # ax.plot(xy_b[:,0], xy_b[:,1], 'rx')

    # fit quadratic curves
    coef_a = np.polyfit(xy_a[:,0], xy_a[:,1], 2)
    ya_fit = np.polyval(coef_a, xy_a[:,0])
    coef_b = np.polyfit(xy_b[:,0], xy_b[:,1], 2)
    yb_fit = np.polyval(coef_b, xy_b[:,0])
    # ax.plot(xy_a[:,0], ya_fit, 'gx-')
    # ax.plot(xy_b[:,0], yb_fit, 'gx-')
    # plt.show()

    # 4. find the tilted side of the LED light clusters, and fit a quadratic curve to it, denote the curve by C.
    coef_c = np.array([0.0, 1.0, 0.0])
    res = minimize(find_boundary, coef_c, (image, xmin, xmax, ymin, ymax), method='powell', options={'xtol': 1e-4, 'disp': True})
    # res = minimize(rosen, x0, image, method='nelder-mead', options={'xtol': 1e-8, 'disp': True})
    drawlines_cv([coef_a, coef_b, res.x], image, [xy_a[0,0], xy_b[0,0], 0], [xy_a[-1,0], xy_b[-1,0], width])
    print(res.x)

    # 5. find the intersection of column A and curve C, denoted by (r1, c1), and the intersection of column B and curve C,
    #  denoted by (r2, c2), then the line_delay * (r2 - r1) = led_time * (B - A)
    coef_a_ = np.polyfit(xy_a[:,0], xy_a[:,1], 1)
    coef_b_ = np.polyfit(xy_b[:,0], xy_b[:,1], 1)
    c1, r1 = get_intersection(coef_a, res.x, 0, width, coef_a_)
    c2, r2 = get_intersection(coef_b, res.x, 0, width, coef_b_)

    # calculate the result
    A = col_a
    B = col_b
    if r1==-1 or r2==-1:
        print(r1)
        print(r2)
        print('Error!')
        return -1
    # r2 = 430
    # r1 = 120
    # B = 8
    # A = 1
    linedelay = led_time * (B - A) / (r2 - r1)
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

    image = cv2.imread(args.img)
    cv2.imshow("raw image", image)
    cv2.waitKey(0)

    linedelay = compute_line_delay(image, args.led_time, args.debug_dir)
    if DEBUG:
        print('Found line delay {} for {}'.format(linedelay, args.img))


if __name__ == '__main__':
    main()
