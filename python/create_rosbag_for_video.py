#!/usr/bin/env python
# create a rosbag from a video and its timestamp file.

from __future__ import print_function

import sys
import argparse

import numpy as np
import rosbag
import rospy

import kalibr_bagcreater
import utility_functions


def parseArgs():
    parser = argparse.ArgumentParser(
        description="Write images of a video into a rosbag.")
    parser.add_argument("video", help="Input video file.")
    parser.add_argument("--image_topic",
                        help="Save to which topic.",
                        default='/cam0/image_raw')
    parser.add_argument("--time_file",
                        default=None,
                        help="Timestamp file for every frame in the video.")
    parser.add_argument("--output_bag",
                        default="",
                        help="Bag to append the images.")

    parser.add_argument("--start_seconds",
                        default=0,
                        help="Start time within the video.")
    parser.add_argument("--finish_seconds",
                        default=1e8,
                        help="Finish time within the video.")

    parser.add_argument("--ratio",
                        default=1.0,
                        help="Select a portion of all frames at ratio in [0, 1.0].")

    args = parser.parse_args()
    return args


def main():
    args = parseArgs()
    video_from_to = [args.start_seconds, args.finish_seconds]
    utility_functions.check_file_exists(args.video)
    print('Frame time range within the video: {}'.format(video_from_to))

    frame_timestamps = np.loadtxt(args.time_file)
    rostimestamps = []
    for row in frame_timestamps:
        rostimestamps.append(rospy.Time(row[0]))

    print('Loaded {} timestamps for frames'.format(len(rostimestamps)))
    first_frame_imu_time = rostimestamps[0].to_sec()
    bag = rosbag.Bag(args.output_bag, 'a')
    videotimerange = kalibr_bagcreater.write_video_to_rosbag(
        bag,
        args.video,
        video_from_to,
        first_frame_imu_time,
        rostimestamps,
        frame_remote_timestamps=None,
        max_video_frame_height=100000,
        shift_in_time=0.0,
        topic=args.image_topic,
        ratio=args.ratio)
    bag.close()
    print('Saved to bag file {}'.format(args.output_bag))


if __name__ == "__main__":
    main()
