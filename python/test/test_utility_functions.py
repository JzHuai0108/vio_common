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
