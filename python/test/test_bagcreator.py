# -*- coding: utf-8 -*-
#!/usr/bin/env python

'''
This module tests methods from folder_helper
To run tests, $python3 -m pytest
'''
from __future__ import print_function

import os
import sys
import rospy

import kalibr_bagcreater

def test_loadtimestamps():
    time_file = "/home/jhuai/Desktop/temp_android/2019_06_16_12_03_45_phab2/frame_timestamps.txt"
    timestamps = kalibr_bagcreater.loadtimestamps(time_file)
    assert timestamps[0] == rospy.Time(675013, 982384000)
    assert timestamps[-1] == rospy.Time(675656, 506415000)
    time_file = "/home/jhuai/Desktop/temp_android/2019_06_16_12_03_45_phab2/movie_metadata.csv"
    timestamps = kalibr_bagcreater.loadtimestamps(time_file)
    assert timestamps[0] == rospy.Time(675014, 15769968)
    assert timestamps[-1] == rospy.Time(675656, 506415968)

