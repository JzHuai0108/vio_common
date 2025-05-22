#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Display images in a rosbag on a specific image topic,
and optionally save image messages and their timestamps.
"""
from __future__ import print_function

import argparse
import os

from sensor_msgs.msg import CompressedImage
import numpy as np

import rosbag
import rospy
from cv_bridge import CvBridge
import cv2


def print_image_info(cv_img):
    h, w = cv_img.shape[:2]
    dtype = cv_img.dtype
    lenshape = len(cv_img.shape)
    if lenshape == 2:
        channels = 1
    else:
        channels = cv_img.shape[2]
    print('Image h {} w {} data type {} channels {}'.format(
        h, w, dtype, channels))

def compressed_imgmsg_to_cv2(compressed_msg):
    np_arr = np.frombuffer(compressed_msg.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
    return cv_image

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(
        description="Display images in a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("--image_topic",
                        help="Display images from which topic.",
                        default='/cam0/image_raw')
    parser.add_argument("--outputdir",
                        default=None,
                        help="Dump the images to the directory.")
    parser.add_argument('--time_from_to',
                        type=float,
                        nargs=2,
                        default=[0, 1e8],
                        help='Show the frames starting from up to this'
                        ' time [s] relative to the bag start time.')
    args = parser.parse_args()

    in_bag = rosbag.Bag(args.bag_file, "r")
    startTime = in_bag.get_start_time()

    topic_list = in_bag.get_type_and_topic_info()[1].keys()
    print('Topics {} are in {}'.format(topic_list, args.bag_file))

    print('Press q key to quit.')
    bridge = CvBridge()
    count = 0
    image_local_time = [None, None]
    image_remote_time = [None, None]
    time_stream = None
    for _, msg, t in in_bag.read_messages(topics=[args.image_topic], start_time=rospy.Time(startTime + args.time_from_to[0]),
                                          end_time=rospy.Time(startTime + args.time_from_to[1])):
        if 'compressed' in args.image_topic:
            cv_img = compressed_imgmsg_to_cv2(msg)
        else:
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if count == 0:
            print('Video frame info:')
            print_image_info(cv_img)
            image_local_time[0] = t
            image_remote_time[0] = msg.header.stamp
            if args.outputdir:
                time_stream = open(os.path.join(args.outputdir, "timestamps.txt"), 'w')
                time_stream.write('#sensor time, computer time\n')
        if time_stream:
            time_stream.write('{},{}\n'.format(msg.header.stamp, t))
        image_local_time[1] = t
        image_remote_time[1] = msg.header.stamp
        if args.outputdir:
            imagename = os.path.join(args.outputdir, '{}.jpg'.format(msg.header.stamp))
            cv2.imwrite(imagename, cv_img)

        cv2.imshow('Frame', cv_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        count += 1
    if time_stream:
        time_stream.close()
    cv2.destroyAllWindows()
    in_bag.close()
    print("Displayed {} images of {} on topic {}\n\t\t\tlocal time\t\t\tremote time\nfirst image\t{}\t{}\n"
          "last image\t{}\t{}\n".format(count, args.bag_file, args.image_topic,
                                        image_local_time[0], image_remote_time[0],
                                        image_local_time[1], image_remote_time[1]))


if __name__ == '__main__':
    main()
