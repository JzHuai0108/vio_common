#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Display images in a rosbag on a specific image topic
"""
import os
import argparse

import cv2

import rosbag
from cv_bridge import CvBridge


def print_image_info(cv_img):
    h, w = cv_img.shape[:2]
    dtype = cv_img.dtype
    lenshape = len(cv_img.shape)
    if lenshape == 2:
        channels = 1
    else:
        channels = cv_img.shape[2]
    print('Image h {} w {} data type {} channels {}'.format(h, w, dtype, channels))


def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Display images in a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("--image_topic", help="Display images from which topic.",
                        default='/cam0/image_raw')

    args = parser.parse_args()

    in_bag = rosbag.Bag(args.bag_file, "r")
    topic_list = in_bag.get_type_and_topic_info()[1].keys()
    print('Topics {} are in {}'.format(topic_list, args.bag_file))

    print('Press q key to quit.')
    bridge = CvBridge()
    count = 0
    first_image_time = None
    last_image_time = None
    for _, msg, t in in_bag.read_messages(topics=[args.image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if count == 0:
            print('Video frame info:')
            print_image_info(cv_img)
            first_image_time = t
        last_image_time = t
        cv2.imshow('Frame', cv_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        count += 1
    cv2.destroyAllWindows()
    in_bag.close()
    print("Displayed {} images of {} on topic {} first image time {} last image time {}".
          format(count, args.bag_file, args.image_topic, first_image_time, last_image_time))


if __name__ == '__main__':
    main()
