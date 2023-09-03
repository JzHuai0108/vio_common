#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Display point clouds in a rosbag on a specific topic,
and optionally transform these point clouds to a world frame.
The transfrom is from an oodmetry topic.
"""
from __future__ import print_function

import argparse
import os

import rosbag
import rospy


def parse_args():
    parser = argparse.ArgumentParser(
        description="Display images in a ROS bag. compressedImage is not supported.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("--lidar_topic",
                        help="Display images from which topic.",
                        default='/cam0/image_raw')
    parser.add_argument("--gt_topic", default=None, help="Ground truth pose world_T_lidar topic.")
    parser.add_argument("--gt_bag", default=None, help="Ground truth pose world_T_lidar bag.")

    parser.add_argument('--time_from_to',
                        type=float,
                        nargs=2,
                        default=[0, 1e8],
                        help='Show the frames starting from up to this'
                        ' time [s] relative to the bag start time.')
    return parser.parse_args()

def main():
    """Extract a folder of images from a rosbag.
    """
    args = parse_args()

    in_bag = rosbag.Bag(args.bag_file, "r")
    startTime = in_bag.get_start_time()

    topic_list = in_bag.get_type_and_topic_info()[1].keys()
    print('Topics {} are in {}'.format(topic_list, args.bag_file))

    gt_bag = rosbag.Bag(args.gt_bag, 'r')
    gt_topic_list = gt_bag.get_type_and_topic_info()[1].keys()
    print('Topics {} are in {}'.format(gt_topic_list, args.gt_bag))

    gt_poses = {}
    for _, msg, t in gt_bag.read_messages(topics=[args.gt_topic], start_time=rospy.Time(startTime + args.time_from_to[0]),
                                            end_time=rospy.Time(startTime + args.time_from_to[1])):
        print(msg)
        gt_poses[t] = msg
    gt_bag.close()

    count = 0
    publisher = rospy.Publisher(args.lidar_topic, PointCloud2, queue_size=1)

    for _, msg, t in in_bag.read_messages(topics=[args.lidar_topic], start_time=rospy.Time(startTime + args.time_from_to[0]),
                                            end_time=rospy.Time(startTime + args.time_from_to[1])):
        print(msg)
        count += 1
        publisher.publish(msg)


    in_bag.close()
    print("Displayed {} scans of {} on topic {}\n".format(count, args.bag_file, args.lidar_topic))


if __name__ == '__main__':
    main()
