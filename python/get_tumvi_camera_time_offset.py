#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Given a raw sequence in rosbag and corresponding calibrated sequence in rosbag
find the camera time offset applied to images in image topics, /cam0/image_raw,
/cam1/image_raw
"""
from __future__ import print_function

import numpy as np
# import rosbag
import rospy


def example_rospy_time_in_array():
    a = np.array([rospy.Time(2, 432985), rospy.Time(213908, 324342)])
    b = np.array([rospy.Time(1, 432985), rospy.Time(213907, 900324342)])
    c = a - b
    assert c[0].to_nsec() == 1e9
    assert c[1].to_nsec() == 1e8


def get_timestamps_of_topic(bagfile, topic):
    """

    :param bagfile:
    :param topic:
    :return: headerstamps, times. They are obtained as below.
    for topic, msg, time in bag.read_messages(topics=['chatter', 'numbers']):
        headerstamps.append(msg.header.stamp)
        times.appennd(time)
    """
    print("reading from {} at topic {}".format(bagfile, topic))
    headerstamps = np.array(
        [rospy.Time(2, 432985),
         rospy.Time(213908, 324342)])
    times = np.array([rospy.Time(1, 432985), rospy.Time(213907, 900324342)])
    return headerstamps, times


def main():
    rawbag = ''
    calibratedbag = ''
    bagfile_list = [rawbag, calibratedbag]
    topic_list = ['/cam0/image_raw', '/cam1/image_raw', '/imu0']

    headerstamps = None
    times = None
    for bagfile in bagfile_list:
        for topic in topic_list:
            headerstamps, times = get_timestamps_of_topic(bagfile, topic)

    print('# headerstamps {} # times {}'.format(len(headerstamps), len(times)))
    # start analysis of the timestamps

    # assert the times and header stamps are not equal for topics in raw bag,
    # but are equal for topics in calibrated bag.
    # for topic in topic_list:
    #   assert topic raw header stamps != topic raw times
    #   assert topic calibrated header stamps == topic calibrated times

    # The below relations should hold for header stamps of these topics.
    # cam0 raw header stamp + cam0 time delay + overall time shift =
    #     cam0 calibrated header stamp
    # cam1 raw header stamp + cam1 time delay + overall time shift =
    #     cam1 calibrated header stamp
    # imu0 raw header stamp + overall time shift = imu0 calibrated header stamp

    # assert the raw times are not consistent.
    # assert cam0 calibrated times - cam0 raw times != const
    # assert cam1 calibrated times - cam1 raw times != const
    # assert imu0 calibrated times - imu0 raw times != const

    # cam0 calibrated header stamp - cam0 raw header stamp  = const deltac0
    # cam1 calibrated header stamp - cam1 raw header stamp  = const deltac1
    # imu0 calibrated header stamp - imu0 raw header stamp = overall_time_shift

    # assert the below for header stamps,
    # assert deltac0 == const
    # assert deltac1 == const
    # assert overall_time_shift == const

    # cam0_time_delay = deltac0 - overall_time_shift
    # cam1_time_delay = deltac1 - overall_time_shift
    cam0_time_delay = 0
    cam1_time_delay = 0

    print('Time delay for rosbag {}'.format(bagfile_list[0]))
    print('cam {}: {}'.format(0, cam0_time_delay))
    print('cam {}: {}'.format(1, cam1_time_delay))


if __name__ == '__main__':
    main()
