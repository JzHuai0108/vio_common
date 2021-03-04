#!/usr/bin/env python
# create a rosbag from a video and its timestamp file.

from __future__ import print_function

import os
import sys
import warnings

import rosbag
import kalibr_bagcreater
import utility_functions


def main():
    if len(sys.argv) < 5:
        print("Usage: {} <folder where only one video avi and frame timestamp txt for bluefox reside> "
              "<output bag fullpath> <start seconds> <finish seconds>".format(sys.argv[0]))
        sys.exit(1)

    video = None
    for root, dirnames, filenames in os.walk(sys.argv[1]):
        for filename in filenames:
            if filename.endswith('.avi'):
                video = os.path.join(root, filename)
                break

    if not video:
        warnings.warn("Failed to find any avi file under {}. Please make sure the correct folder is passed in.".format(sys.argv[1]))
        sys.exit(1)
    video_time_file = video.replace('.avi', '.txt')
    output_bag = sys.argv[2]
    video_from_to = [float(sys.argv[3]), float(sys.argv[4])]
    bag = rosbag.Bag(output_bag, 'w')
    utility_functions.check_file_exists(video)
    print('Frame time range within the video: {}'.format(video_from_to))

    frame_timestamps = list()

    frame_timestamps = kalibr_bagcreater.loadtimestamps(video_time_file)
    print('Loaded {} timestamps for frames'.format(len(frame_timestamps)))
    first_frame_imu_time = frame_timestamps[0].to_sec()
    videotimerange = kalibr_bagcreater.write_video_to_rosbag(
        bag,
        video,
        video_from_to,
        first_frame_imu_time,
        frame_timestamps,
        frame_remote_timestamps=None,
        max_video_frame_height=100000,
        shift_in_time=0.0,
        topic="/cam0/image_raw")
    bag.close()
    print('Saved to bag file {}'.format(output_bag))


if __name__ == "__main__":
    main()
