# !/usr/bin/env python
import argparse
import os
import sys

import rosbag
import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import utility_functions
import tf_helpers
import rviz_camera_frustum

MAX_POSE_FREQUENCY = 15


def parseArgs():
    # setup the argument list
    parser = argparse.ArgumentParser(
        description='Convert the poses in a csv file of TUM_RGBD or KALIBR format to ROS path messages in a rosbag.')
    # case 1 arguments used by the kalibr to create a rosbag from a folder
    parser.add_argument('infile',
                        help='A file containing rows of states, each state including '
                             'time[ns or sec], position and quaternion.\n')

    parser.add_argument('--outfile',
                        default="infile.bag", help='ROS bag file (default: %(default)s)')

    parser.add_argument('--plus_time_file', default="",
                        help='The file of the time offset in secs added to the timestamps in the csv file.'
                             ' If not provided, the time offset is set to zero. (default: %(default)s)')

    parser.add_argument('--transform_json',
                        default="",
                        help='json file containing the transform to be multiplied at the left\n'
                             '(default: %(default)s)')
    parser.add_argument('--child_frame',
                        default="vins",
                        help='The signiture of the source frame of the poses (default: %(default)s)')
    parser.add_argument('--red', default=1.0, type=float,
                        help='marker redness (default: %(default)s)')
    parser.add_argument('--green', default=1.0, type=float,
                        help='marker redness (default: %(default)s)')
    parser.add_argument('--blue', default=0.0, type=float,
                        help='marker redness (default: %(default)s)')

    if len(sys.argv) < 2:
        parser.print_help()
        sys.exit(1)
    parsed = parser.parse_args()
    print(parsed)
    return parsed


def load_time_trans_quat(tffile):
    """
    load a txt file

    :param tffile:
    :param line_format: TUM_RGBD format t[s] x y z qx qy qz qw
        or KALIBR format t[ns] x y z qx qy qz qw
        Because this function does not do normalization,
            for better precision, qx qy qz qw should have 9 decimal digits

    :return: a list of timed poses, each entry is
        [rospy time, tx, ty, tz, qx, qy, qz, qw]
    """
    chunk = []
    delimiter = None
    with open(tffile, "r") as stream:
        for index, line in enumerate(stream):
            if utility_functions.is_header_line(line):
                continue
            if delimiter is None:
                delimiter = utility_functions.decide_delimiter(line)
            if delimiter == " ":
                time_pose_str = line.split()
            else:
                time_pose_str = line.split(delimiter)

            secs, nsecs = utility_functions.parse_time(time_pose_str[0])
            timestamp = rospy.Time(secs, nsecs)
            row = [timestamp] + [float(x) for x in time_pose_str[1:]]
            chunk.append(row)
    return chunk


def downsample_if_needed(timed_data, max_freq=MAX_POSE_FREQUENCY):
    """

    :param timed_data: a list of elements [rospy time, col1, col2, ...]
    :param max_freq:
    :return: downsampled timed data of the same format
    """
    min_delta = rospy.Duration.from_sec(1.0 / max_freq)
    out_timed_data = []
    start_epoch = timed_data[0][0]
    out_timed_data.append(timed_data[0])
    for entry in timed_data:
        if entry[0] - start_epoch >= min_delta:
            start_epoch = entry[0]
            out_timed_data.append(entry)
        # else do nothing
    return out_timed_data


def bag_tf_poses(timed_pos_quat, bagpath, child_frame_id="vins", red=1.0, green=1.0, blue=0.0):
    """

    :param timed_pos_quat:
    :param bagpath:
    :return:
    """
    if os.path.isfile(bagpath):
        bag = rosbag.Bag(bagpath, "a")
    else:
        bag = rosbag.Bag(bagpath, "w")

    path = Path()
    for index, entry in enumerate(timed_pos_quat):
        odometry = Odometry()
        odometry.header.stamp = entry[0]
        odometry.header.seq = index
        odometry.header.frame_id = "world"
        odometry.child_frame_id = child_frame_id

        odometry.pose.pose.position.x = entry[1]
        odometry.pose.pose.position.y = entry[2]
        odometry.pose.pose.position.z = entry[3]
        odometry.pose.pose.orientation.x = entry[4]
        odometry.pose.pose.orientation.y = entry[5]
        odometry.pose.pose.orientation.z = entry[6]
        odometry.pose.pose.orientation.w = entry[7]

        odometry.twist.twist.linear.x = 0
        odometry.twist.twist.linear.y = 0
        odometry.twist.twist.linear.z = 0

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = entry[0]
        pose_stamped.header.seq = index
        pose_stamped.header.frame_id = "world"
        pose_stamped.pose = odometry.pose.pose

        path.header.stamp = entry[0]
        path.header.seq = index
        path.header.frame_id = "world"
        path.poses.append(pose_stamped)
        if len(path.poses) > 2:
            path.poses.pop(0)

        marker = rviz_camera_frustum.generate_frustum_marker(entry[1:], 1.0, entry[0], red, green, blue)
        marker.header.frame_id = "world"
        marker.header.stamp = entry[0]
        marker.header.seq = index

        # see https://wiki.ros.org/rviz/DisplayTypes/Marker#Triangle_List_.28TRIANGLE_LIST.3D11.29_.5B1.1.2B-.5D for drawing complex markers
        bag.write("/path_{}".format(child_frame_id), path, entry[0])
        bag.write("/odom_{}".format(child_frame_id), marker, entry[0])

    bag.close()


def main():
    parsed = parseArgs()

    data = load_time_trans_quat(parsed.infile)
    if parsed.transform_json:
        transform = utility_functions.read_pose_from_json(parsed.transform_json)
        transform_4x4 = tf_helpers.transformtransformation(transform)
        transformed_data = tf_helpers.left_multiply_transform(data, transform_4x4)
    else:
        transformed_data = data

    if parsed.plus_time_file:
        with open(parsed.plus_time_file) as stream:
            for index, line in enumerate(stream):
                time_offset = float(line.rstrip("\n"))
                if index == 0:
                    break

        for index in range(len(transformed_data)):
            transformed_data[index][0] -= rospy.Duration.from_sec(time_offset)
    transformed_data = downsample_if_needed(transformed_data)
    bag_tf_poses(transformed_data, parsed.outfile, parsed.child_frame, parsed.red, parsed.green, parsed.blue)


if __name__ == "__main__":
    main()
