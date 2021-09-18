#!/usr/bin/env python3

"""
compute line delay with an image of a LED panel captured by a rolling shutter camera
"""

import argparse
import sys

import cv2

import ioUtilities

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
    r2 = 430
    r1 = 120
    B = 8
    A = 1
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
