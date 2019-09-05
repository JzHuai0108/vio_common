# -*- coding: utf-8 -*-
#!/usr/bin/env python

'''
This module tests methods from folder_helper
To run tests, $python -m pytest
'''
from __future__ import print_function

import os
import sys
import rospy

import kalibr_bagcreater

def test_loadtimestamps():
    file_path = os.path.abspath(__file__)
    src_dir = os.path.dirname(file_path)
    assert src_dir.endswith("test")

    time_file = os.path.join(src_dir, "data/frame_timestamps.txt")
    timestamps = kalibr_bagcreater.loadtimestamps(time_file)
    assert timestamps[0] == rospy.Time(464681, 846889000)
    assert timestamps[-1] == rospy.Time(464682, 180889000)
    time_file = os.path.join(src_dir, "data/movie_metadata.csv")
    timestamps = kalibr_bagcreater.loadtimestamps(time_file)
    assert timestamps[0] == rospy.Time(464681, 846889000)
    assert timestamps[-1] == rospy.Time(464682, 180889000)

