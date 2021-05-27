#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Given a raw sequence in rosbag and corresponding calibrated sequence in rosbag
find the camera time offset applied to images in image topics, /cam0/image_raw,
/cam1/image_raw
"""
from __future__ import print_function

import argparse

import numpy as np
import rosbag
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
    headerstamps = []
    times = []
    in_bag = rosbag.Bag(bagfile, "r")
    max_samples = 100
    for _, msg, t in in_bag.read_messages(topics=[topic]):
        headerstamps.append(msg.header.stamp.to_nsec())
        times.append(t.to_nsec())
        if len(times) >= max_samples:
            break

    in_bag.close()

    headerstamps = np.array(headerstamps)
    times = np.array(times)
    return headerstamps, times


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("rawbag", help="Raw bag.")
    parser.add_argument("calibratedbag", help="Calibrated bag.")
    args = parser.parse_args()
    rawbag = args.rawbag
    calibratedbag = args.calibratedbag
    bagfile_list = [rawbag, calibratedbag]
    topic_list = ['/cam0/image_raw', '/cam1/image_raw', '/imu0']

    headerstamps_all = dict()
    times_all = dict()
    for bagfile in bagfile_list:
        headerstamps_all[bagfile] = dict()
        times_all[bagfile] = dict()
        for topic in topic_list:
            headerstamps, times = get_timestamps_of_topic(bagfile, topic)
            print('# headerstamps {} # times {}'.format(
                len(headerstamps), len(times)))
            headerstamps_all[bagfile][topic] = headerstamps
            times_all[bagfile][topic] = times

    # start analysis of the timestamps

    # assert the times and header stamps are not equal for topics in raw bag,
    # but are equal for topics in calibrated bag.
    # for topic in topic_list:
    #   assert topic raw header stamps != topic raw times
    #   assert topic calibrated header stamps == topic calibrated times

    for topic in topic_list:
        assert (abs(headerstamps_all[rawbag][topic] - times_all[rawbag][topic])
                > 1e5).all()
        assert (abs(headerstamps_all[calibratedbag][topic] -
                    times_all[calibratedbag][topic]) < 1e3).all()

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
    for topic in topic_list:
        # print(times_all[calibratedbag][topic] - times_all[rawbag][topic])
        diff = times_all[calibratedbag][topic] - times_all[rawbag][topic]
        assert (diff[1:] != diff[0]).all()
    # cam0 calibrated header stamp - cam0 raw header stamp  = const deltac0
    # cam1 calibrated header stamp - cam1 raw header stamp  = const deltac1
    # imu0 calibrated header stamp - imu0 raw header stamp = overall_time_shift

    # assert the below for header stamps,
    # assert deltac0 == const
    # assert deltac1 == const
    # assert overall_time_shift == const
    for topic in topic_list:
        # print(headerstamps_all[calibratedbag][topic] -
        #     headerstamps_all[rawbag][topic])
        diff = headerstamps_all[calibratedbag][topic] - headerstamps_all[
            rawbag][topic]
        assert (diff[1:] == diff[0]).all()

    deltac0 = headerstamps_all[calibratedbag]['/cam0/image_raw'][
        0] - headerstamps_all[rawbag]['/cam0/image_raw'][0]
    deltac1 = headerstamps_all[calibratedbag]['/cam1/image_raw'][
        0] - headerstamps_all[rawbag]['/cam1/image_raw'][0]
    overall_time_shift = headerstamps_all[calibratedbag]['/imu0'][
        0] - headerstamps_all[rawbag]['/imu0'][0]

    cam0_time_delay = deltac0 - overall_time_shift
    cam1_time_delay = deltac1 - overall_time_shift

    print('Time delay for rosbag {}'.format(bagfile_list[0]))
    print('cam {}: {}'.format(0, cam0_time_delay))
    print('cam {}: {}'.format(1, cam1_time_delay))
    print('{} time delay: {}, such that td + raw.header.stamp = '
          'calibrated.header.stamp.'.format(rawbag, overall_time_shift))


if __name__ == '__main__':
    main()
