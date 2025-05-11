#!/usr/bin/env python3
"""
Create a rosbag for data collected by an openlog artemis and a video camera
The video camera records a video and a timestamp file.
Each line in the timestamp file: frame time by sensor clock (ns), ...., frame unix time by computer (sec).
The artemis records an IMU file, each line
unix time by computer (sec), ax, ay, az, gx, gy, gz, time by sensor (sec)

Sometimes, the unix time of the video has to plus an offset say 8 hours to account for the local time shift.
"""
import argparse
import os
import sys

import rospy
import rosbag

import kalibr_bagcreator
import utility_functions


def parse_args():
    parser = argparse.ArgumentParser(
        description='create a ROS bag containing image and imu topics '
        'from a video clip and inertial data.')

    parser.add_argument('--output_bag',
                        default="output.bag",
                        help='ROS bag file %(default)s')

    parser.add_argument('video_file',
                        nargs='?',
                        help='Video filename')

    parser.add_argument(
        'imu_file',
        help='Imu filename.')

    parser.add_argument('--video_file_time_offset',
                        type=float,
                        default=0.0,
                        help='When the time file for the video is provided, '
                             'the video_file_time_offset may be added to '
                             'these timestamps.(default: %(default)s)',
                        required=False)

    parser.add_argument('--video_from_to',
                        type=float,
                        nargs=2,
                        help='Use the video frames starting from up to this'
                        ' time [s] relative to the video beginning.')

    parser.add_argument('--video_time_file',
                        default='',
                        nargs='?',
                        help='The csv file containing timestamps of every '
                        'video frames in IMU clock(default: %(default)s).'
                        ' Except for the header, each row has the '
                        'device timestamp in nanoseconds as the first component '
                        'and the host timestamp in seconds as the last component',
                        required=False)

    parser.add_argument(
        '--max_video_frame_height',
        type=int,
        default=100000,
        help='For a video frame, if min(rows, cols) > %(default)s, '
        'it will be downsampled by 2. If the resultant bag is '
        'used for photogrammetry, the original focal_length and '
        'principal_point should be half-sized, but the '
        'distortion parameters should not be changed. (default: %(default)s)',
        required=False)

    if len(sys.argv) < 2:
        msg = 'Example usage: {} --video /dataset/' \
               'IMG_2805.avi --imu /dataset/gyro_accel.csv ' \
               '--video_time_file /dataset/movie_metadata.csv ' \
               '--output_bag /dataset/IMG_2805.bag\n'. \
            format(sys.argv[0])

        print(msg)
        parser.print_help()
        sys.exit(1)

    parsed = parser.parse_args()
    return parsed


def main():
    parsed = parse_args()
    bag = rosbag.Bag(parsed.output_bag, 'w')

    # write video
    utility_functions.check_file_exists(parsed.video_file)
    if parsed.video_from_to:
        print('Frame time range within the video: {}'.format(parsed.video_from_to))
    frame_local_timestamps = None
    frame_remote_timestamps = None

    if parsed.video_time_file:
        frame_timestamps = kalibr_bagcreator.load_local_and_remote_times(parsed.video_time_file)
        aligned_local_timestamps = [time[0] + rospy.Duration.from_sec(parsed.video_file_time_offset)
                                    for time in frame_timestamps]
        frame_local_timestamps = aligned_local_timestamps
        frame_remote_timestamps = [time[1] for time in frame_timestamps]
        print('Loaded {} timestamps for frames'.format(len(frame_timestamps)))
        first_frame_imu_time = frame_local_timestamps[0].to_sec()
    videotimerange = kalibr_bagcreator.write_video_to_rosbag(
        bag,
        parsed.video_file,
        parsed.video_from_to,
        first_frame_imu_time,
        frame_local_timestamps,
        frame_remote_timestamps,
        parsed.max_video_frame_height,
        shift_in_time=0.0,
        topic="/cam0/image_raw")
    print('video time range {}'.format(videotimerange))

    # write IMU data that falling into the unix time range
    kalibr_bagcreator.write_imufile_remotetime_to_rosbag(bag, parsed.imu_file, videotimerange, 5, "/imu0")

    bag.close()
    print('Saved to bag file {}'.format(parsed.output_bag))


if __name__ == "__main__":
    main()
