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
        print("Usage: {} <video avi> "
              "<output bag fullpath> <start seconds> <finish seconds> [frame select ratio, (0-1)]".format(sys.argv[0]))
        sys.exit(1)

    ratio = 1.0
    if len(sys.argv) > 5:
        ratio = float(sys.argv[5])

    video = sys.argv[1]
    potential_exts = [".txt", ".csv"]
    video_time_file = None
    for ext in potential_exts:
        video_time_file = video.replace('.avi', ext)
        if os.path.isfile(video_time_file):
            break
    if video_time_file is None:
        print("Failed to find timestamp file for {}".format(video))
    output_bag = sys.argv[2]
    video_from_to = [float(sys.argv[3]), float(sys.argv[4])]
    bag = rosbag.Bag(output_bag, 'w')
    utility_functions.check_file_exists(video)
    print('Frame time range within the video: {}'.format(video_from_to))

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
        topic="/cam0/image_raw",
        ratio=ratio)
    bag.close()
    print('Saved to bag file {}'.format(output_bag))


if __name__ == "__main__":
    main()
