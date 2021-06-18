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


def get_timestamps_of_topic(bagfile, topic, max_samples):
    """

    :param bagfile:
    :param topic:
    :return: headerstamps, times. They are obtained as below.
    for topic, msg, time in bag.read_messages(topics=['chatter', 'numbers']):
        headerstamps.append(msg.header.stamp)
        times.appennd(time)
    """
    headerstamps = []
    times = []
    in_bag = rosbag.Bag(bagfile, "r")
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
    topic_list = ['/cam0/image_raw', '/cam1/image_raw', '/imu0', '/vrpn_client/raw_transform']

    headerstamps_all = dict()
    times_all = dict()
    max_samples = 200
    for bagfile in bagfile_list:
        headerstamps_all[bagfile] = dict()
        times_all[bagfile] = dict()
        for topic in topic_list:
            headerstamps, times = get_timestamps_of_topic(bagfile, topic, max_samples)
            print('Check #header.stamps {} #message times {} of topic {} in {}'.format(
                len(headerstamps), len(times), topic, bagfile))
            headerstamps_all[bagfile][topic] = headerstamps
            times_all[bagfile][topic] = times

    # assert that times and header stamps are not equal for topics in raw bag,
    # but are equal for topics in calibrated bag.
    for topic in topic_list:
        assert (abs(headerstamps_all[rawbag][topic] - times_all[rawbag][topic])
                > 1e5).all()
        assert (abs(headerstamps_all[calibratedbag][topic] -
                    times_all[calibratedbag][topic]) < 1e3).all()

    # assert calibrated times - raw times != const
    for topic in topic_list:
        diff = times_all[calibratedbag][topic] - times_all[rawbag][topic]
        assert (diff[1:] != diff[0]).all()

    # calibrated header stamps - raw header stamps  = const time_shift.
    print('Time delay for topics in rosbag {}'.format(bagfile_list[0]))
    print('time delay + raw.header.stamp = calibrated.header.stamp.')
    timedelays = {}
    for index, topic in enumerate(topic_list):
        diff = headerstamps_all[calibratedbag][topic] - headerstamps_all[
            rawbag][topic]
        if index < 3:
            assert (diff[1:] == diff[0]).all()
        else:
            assert max(abs(diff[1:] - diff[0])) < 1100

        delay = headerstamps_all[calibratedbag][topic][0] - headerstamps_all[rawbag][topic][0]
        timedelays[topic] = delay
        print('{}: {}'.format(topic, delay))
    if timedelays['/cam0/image_raw'] == timedelays['/cam1/image_raw'] and \
            timedelays['/cam0/image_raw'] == timedelays['/imu0']:
        print('Stereo cameras and IMU topics have the same time offset.')
    else:
        print('Stereo cameras and IMU topics do NOT have the same time offset.')
    print('IMU calibrated.header.stamp + {} = mocap calibrated.header.stamp'.format(
        timedelays['/vrpn_client/raw_transform'] - timedelays['/imu0']))
    print('Warn: Do not use the estimated time delay between mocap and IMU which '
        'is slightly different from values given by the authors.')

if __name__ == '__main__':
    main()
