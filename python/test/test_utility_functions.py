# -*- coding: utf-8 -*-
# !/usr/bin/env python

from __future__ import print_function

import os

import numpy as np

import utility_functions


def test_decide_time_index_and_unit():
    lines = ["34293842, 123890121930",
             "34293845, 123890122040"]
    index, unit, r_index = utility_functions.decide_time_index_and_unit(lines, ',')
    assert index == 1 and unit == 'ns' and r_index == 2

    lines = ["1021383336666000, 3",
             "1021383336766000, 4"]
    index, unit, r_index = utility_functions.decide_time_index_and_unit(lines, ',')
    assert index == 0 and unit == 'ns' and r_index == 2

    lines = ["34293842, 12389.0121930",
             "34293856, 12389.0151930"]
    index, unit, r_index = utility_functions.decide_time_index_and_unit(lines, ',')
    assert index == 1 and unit == 's' and r_index == 2

    lines = ["1021383.336666000, 332189021",
             "1021383.346666000, 332189031"]
    index, unit, r_index = utility_functions.decide_time_index_and_unit(lines, ',')
    assert index == 0 and unit == 's' and r_index == 2

    lines = ["123890.121930, 123.23910",
             "123890.131930, 233.23910"]
    index, unit, r_index = utility_functions.decide_time_index_and_unit(lines, ',')
    assert index == 0 and unit == 's' and r_index == 1

    lines = ["1021383336666000, 2139342.0342",
             "1021383336766000, 2139542.0332"]
    index, unit, r_index = utility_functions.decide_time_index_and_unit(lines, ',')
    assert index == 0 and unit == 'ns' and r_index == 1

def test_parse_time():
    timestr_list = ['0.0023', '0.983122', '0.99979', '1.016458', '1.033126']

    for str in timestr_list:
        sec, nsec = utility_functions.parse_time(str)
        assert sec + nsec * 1e-9 - float(str) < 1e-8


def test_interpolate_imu_data():
    time_gyro_data = np.array(
        [[0.000333, -0.034319, 0.036195, -0.054551],
         [0.010343, -0.022618, 0.003453, -0.074618],
         [0.027341, 0.00509, -0.03562, -0.089374],
         [0.037351, 0.018964, -0.054656, -0.090354],
         [0.04736, 0.018997, -0.063173, -0.083883],
         [0.05734, -0.004396, -0.064382, -0.070958],
         [0.067349, -0.022463, -0.060278, -0.055966]])
    time_acc_data = np.array([
        [0.007932, -0.728685, 8.044571, 6.028114],
        [0.017697, -0.64441, 8.045469, 5.911657],
        [0.027463, -0.604742, 8.087981, 5.776787],
        [0.037229, -0.619113, 8.164622, 5.709277],
        [0.046994, -0.598755, 8.195607, 5.521119],
        [0.05676, -0.511187, 8.21806, 5.422773],
        [0.066526, -0.539478, 8.233029, 5.339846],
        [0.076261, -0.493224, 8.220156, 5.299879],
        [0.086036, -0.474962, 8.204738, 5.212461]])
    res = utility_functions.interpolate_imu_data(time_gyro_data, time_acc_data)
    res_expected = np.array(
        [[3.33000000e-04, -3.43190000e-02, 3.61950000e-02, -5.45510000e-02
             , -7.28685000e-01, 8.04457100e+00, 6.02811400e+00],
         [1.03430000e-02, -2.26180000e-02, 3.45300000e-03, -7.46180000e-02
             , -7.07877317e-01, 8.04479272e+00, 5.99936051e+00],
         [2.73410000e-02, 5.09000000e-03, -3.56200000e-02, -8.93740000e-02
             , -6.05237545e-01, 8.08744993e+00, 5.77847184e+00],
         [3.73510000e-02, 1.89640000e-02, -5.46560000e-02, -9.03540000e-02
             , -6.18858655e-01, 8.16500911e+00, 5.70692623e+00],
         [4.73600000e-02, 1.89970000e-02, -6.31730000e-02, -8.38830000e-02
             , -5.95473217e-01, 8.19644847e+00, 5.51743329e+00],
         [5.73400000e-02, -4.39600000e-03, -6.43820000e-02, -7.09580000e-02
             , -5.12867195e-01, 8.21894900e+00, 5.41784799e+00],
         [6.73490000e-02, -2.24630000e-02, -6.02780000e-02, -5.59660000e-02
             , -5.35567672e-01, 8.23194071e+00, 5.33646718e+00]])
    assert np.linalg.norm(res - res_expected) < 1e-8
