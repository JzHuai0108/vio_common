import json
import numpy as np
from numpy import genfromtxt

import os


SECOND_TO_MILLIS = 1000
SECOND_TO_MICROS = 1000000
SECOND_TO_NANOS = 1000000000

TIME_UNIT_TO_DECIMALS = {'s': 0,
                         "ms": 3,
                         "us": 6,
                         "ns": 9}

def parse_time(timestamp_str, time_unit="ns"):
    """
    convert a timestamp string to a rospy time
    if a dot is not in the string, the string is taken as an int in time_unit
    otherwise, taken as an float in secs
    :param timestamp_str:
    :return:
    """
    secs = 0
    nsecs = 0
    if '.' in timestamp_str:
        index = timestamp_str.find('.')
        if index == 0:
            nsecs = int(float(timestamp_str[index:]) * SECOND_TO_NANOS)
        elif index == len(timestamp_str) - 1:
            secs = int(timestamp_str[:index])
        else:
            secs = int(timestamp_str[:index])
            nsecs = int(float(timestamp_str[index:]) * SECOND_TO_NANOS)
        return secs, nsecs
    else:
        decimal_count = TIME_UNIT_TO_DECIMALS[time_unit]
        if len(timestamp_str) <= decimal_count:
            return 0, int(timestamp_str) * 10 ** (9 - decimal_count)
        else:
            return int(timestamp_str[0:-decimal_count]),\
                   int(timestamp_str[-decimal_count:]) * 10 ** (9 - decimal_count)

def is_float(element_str):
    """check if a string represent a float. To this function, 30e5 is float, but 2131F or 2344f is not float"""
    try:
        float(element_str)
        return True
    except ValueError:
        return False


def is_header_line(line):
    common_header_markers = ['%', '#', '//']
    has_found = False
    for marker in common_header_markers:
        if line.startswith(marker):
            has_found = True
            break
        else:
            continue
    if not has_found:
        if line[0].isdigit():
            return False
        else:
            return True
    return has_found


def decide_delimiter(line):
    common_delimiters = [',', ' ']
    occurrences = []
    for delimiter in common_delimiters:
        occurrences.append(line.count(delimiter))
    max_occurrence = max(occurrences)
    max_pos = occurrences.index(max_occurrence)
    return common_delimiters[max_pos]

def decide_time_index_and_unit(lines, delimiter):
    """
    Time and frame number are at index 0 and 1
    Frame number may not exist
    At least two lines are required to decide if frame number exists
    Following the time or frame number is the tx ty tz and quaternions
    Unit is decided as either nanosec or sec
        depending on if decimal dot is found.
    So the unit can be wrong if timestamps in units ns or ms are provided.
    """
    if len(lines) < 2:
        raise ValueError("Not enough lines to determine time index")
    value_rows = []
    for line in lines:
        rags = line.rstrip(delimiter).split(delimiter)
        value = [float(rag) for rag in rags]
        value_rows.append(value)
    value_array = np.array(value_rows)
    delta_row = value_array[-1, :] - value_array[-2, :]
    whole_number = [value.is_integer() for value in delta_row]
    if whole_number[0]:
        if whole_number[1]:
            if delta_row[0] < delta_row[1]:  # frame id time[ns] tx[m] ty tz
                time_index = 1
                time_unit = 'ns'
                t_index = 2
            else:  # time[ns] frame id tx ty tz
                time_index = 0
                time_unit = 'ns'
                t_index = 2
        else:
            if delta_row[0] > 100:  # time[ns] tx ty tz
                time_index = 0
                time_unit = 'ns'
                t_index = 1
            else:  # frame id time[s] tx ty tz
                time_index = 1
                time_unit = 's'
                t_index = 2
    else:
        if whole_number[1]:
            # time[s] frame id tx ty tz
            time_index = 0
            time_unit = 's'
            t_index = 2
        else:
            # time[s] tx ty tz
            time_index = 0
            time_unit = 's'
            t_index = 1

    return time_index, time_unit, t_index

def normalize_quat_str(val_str_list):
    max_len = max([len(x) - x.find('.') - 1 for x in val_str_list])
    if max_len > 8:
        return val_str_list
    q4 = np.array([float(x) for x in val_str_list])
    q4_normalized = q4 / np.linalg.norm(q4)
    strlist = []
    for j in range(4):
        strlist.append("{}".format(q4_normalized[j]))
    return strlist


def read_pose_from_json(pose_json):
    """

    :param pose_json:
    :return:
    """
    with open(pose_json, 'r') as load_f:
        load_dict = json.load(load_f)
        x = float(load_dict['translation']['x'])
        y = float(load_dict['translation']['y'])
        z = float(load_dict['translation']['z'])
        q_x = float(load_dict['rotation']['i'])
        q_y = float(load_dict['rotation']['j'])
        q_z = float(load_dict['rotation']['k'])
        q_w = float(load_dict['rotation']['w'])
        pose = [x, y, z, q_x, q_y, q_z, q_w]
        return pose


def interpolate_imu_data(time_gyro_array, time_accel_array):
    """
    interpolate accelerometer data at gyro epochs
    :param time_gyro_array: each row [time in sec, gx, gy, gz]
    :param time_accel_array: each row [time in sec, ax, ay, az]
    :return: time_gyro_accel_array: each row [time in sec, gx, gy, gz, ax, ay, az]
    """
    a = []
    for c in range(1, 1+3):
        a.append(np.interp(time_gyro_array[:, 0], time_accel_array[:, 0], time_accel_array[:, c]))
    return np.column_stack((time_gyro_array, a[0], a[1], a[2]))


def load_advio_imu_data(file_csv):
    """

    :param file_csv: each row [time in sec, x, y, z]
    :return: np array nx4
    """
    return genfromtxt(file_csv, delimiter=',', skip_header=0)


def check_file_exists(filename):
    """sanity check"""
    if not os.path.exists(filename):
        raise OSError("{} does not exist".format(filename))
