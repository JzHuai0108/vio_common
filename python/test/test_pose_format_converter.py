# -*- coding: utf-8 -*-
# !/usr/bin/env python

'''
This module tests methods from pose_format_converter
To run tests, $python -m pytest
'''
from __future__ import print_function

import os

import numpy as np

import utility_functions
import convert_pose_format

def create_sample_pose_file(output_file, format_id):
    file_path = os.path.abspath(__file__)
    src_dir = os.path.dirname(file_path)
    assert src_dir.endswith("test")
    sample_data = np.loadtxt(os.path.join(src_dir, "data/sample_trajectory.csv"), delimiter=',', skiprows=1)
    sample_pose = sample_data[:, [0] + list(range(2, 9))]
    with open(output_file, 'w') as stream:
        if format_id == "TUM_RGBD":
            delimiter = " "
            for pose in sample_pose:
                timestr = str(int(pose[0]))
                stream.write("{}.{}".format(timestr[:-9], timestr[-9:]))
                for i in range(3):
                    stream.write("{}{}".format(delimiter, pose[1 + i]))
                for i in range(4):
                    stream.write("{}{}".format(delimiter, pose[4 + i]))
                stream.write("\n")
        elif format_id == "KALIBR":
            delimiter = ","
            for pose in sample_pose:
                timestr = str(int(pose[0]))
                stream.write("{}{}".format(timestr[:-9], timestr[-9:]))
                for i in range(3):
                    stream.write("{}{}".format(delimiter, pose[1 + i]))
                for i in range(4):
                    stream.write("{}{}".format(delimiter, pose[4 + i]))
                stream.write("\n")
        else:
            raise ValueError("Unsupported output format {}!".format(format_id))

def test_convert_pose_format():
    file_path = os.path.abspath(__file__)
    src_dir = os.path.dirname(file_path)
    assert src_dir.endswith("test")
    output_format_id = "TUM_RGBD"
    infile = os.path.join(src_dir, "data/maplab_vertices.csv")
    outfile = os.path.join(src_dir, "data/maplab_vertices.out")
    convert_pose_format.convert_pose_format(infile, outfile, output_format=output_format_id)
    data = np.loadtxt(outfile, delimiter=",", skiprows=0)

    expected_data = np.array(
        [[1403636579.963555500,  1.91623173e-03, - 6.51158784e-03, - 1.11441323e-01,
          - 4.18518361e-02, - 8.42234978e-01, - 2.92401013e-02,   5.36687695e-01],
         [1403636580.963555500, - 3.33400159e-02, 1.85054334e-02, - 1.74646891e-01,
          - 4.98435698e-02, - 8.28262849e-01, - 2.63351592e-02,    5.57496843e-01]])
    assert np.allclose(expected_data, data[[0, -1], :])
    os.remove(outfile)

    sample_file = os.path.join(src_dir, "data/sample_trajectory_kalibr.csv")
    create_sample_pose_file(sample_file, "KALIBR")
    outfile = os.path.join(src_dir, "data/sample_trajectory_tum.out")
    output_format_id = "TUM_RGBD"
    convert_pose_format.convert_pose_format(sample_file, outfile, output_format=output_format_id)
    data = np.loadtxt(outfile, delimiter=",", skiprows=0)

    expected_data = np.array([[1021383.336666, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                               -7.71444894e-02, -7.15149901e-01, 0.00000000e+00, 6.94700904e-01],
                              [1021383.637266, 1.74120000e-02, -4.36136000e-04, 7.12635000e-03,
                               -9.11294589e-02, -7.06687681e-01, 1.93058913e-02, 7.01366684e-01]])
    assert np.allclose(expected_data, data[[0, -1], :])
    os.remove(sample_file)
    os.remove(outfile)

    sample_file = os.path.join(src_dir, "data/sample_trajectory_tum.csv")
    create_sample_pose_file(sample_file, "TUM_RGBD")
    outfile = os.path.join(src_dir, "data/sample_trajectory_kalibr.out")
    output_format_id = "KALIBR"
    convert_pose_format.convert_pose_format(sample_file, outfile, output_format=output_format_id)
    data = np.loadtxt(outfile, delimiter=",", skiprows=0)
    expected_data = np.array([[1021383336666000, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                               -7.71444894e-02, -7.15149901e-01, 0.00000000e+00, 6.94700904e-01],
                              [1021383637266000, 1.74120000e-02, -4.36136000e-04, 7.12635000e-03,
                               -9.11294589e-02, -7.06687681e-01, 1.93058913e-02, 7.01366684e-01]])
    assert np.allclose(expected_data, data[[0, -1], :])
    os.remove(sample_file)
    os.remove(outfile)
