
import os
import subprocess
import sys
import numpy as np
from rosbag import Bag
import rospy

from typing import Dict, List, Tuple

# Assuming the working directory is vio_common/python
mypath = os.path.abspath('../timestamp_corrector/build')  # Make it an absolute path
if mypath not in sys.path:
    sys.path.append(mypath)
import TimestampCorrector as TC


def create_video_bag(seq_dir):
    # Step 1: Call the kalibr_bagcreater.py script with the specified arguments
    video_path = os.path.join(seq_dir, 'movie.mp4')
    imu_path = os.path.join(seq_dir, 'gyro_accel.csv')
    video_time_file_path = os.path.join(seq_dir, 'frame_timestamps.txt')
    output_bag_path = os.path.join(seq_dir, 'movie.bag')

    kalibr_command = [
        'python3', 'kalibr_bagcreater.py',
        '--video', video_path,
        '--imu', imu_path,
        '--video_time_file', video_time_file_path,
        '--output_bag', output_bag_path,
        '--sync_to_unix'
    ]

    try:
        subprocess.run(kalibr_command, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error during bag creation: {e}")
        sys.exit(1)


def check_largest_gap(bag_path, topic, gap_tol_ms):
    gap_tol = rospy.Duration.from_sec(gap_tol_ms / 1000)
    host_times = []
    with Bag(bag_path, 'r') as mid_bag:
        for topic, msg, t in mid_bag.read_messages(topics=[topic]):
            host_times.append(t)

    gaps = [host_times[i+1] - host_times[i] for i in range(len(host_times) - 1)]
    largest_gap_index, largest_gap = max(enumerate(gaps), key=lambda x: x[1])
    if largest_gap > gap_tol:
        start_time = host_times[0]  # First timestamp in the original list
        end_time = host_times[largest_gap_index]  # Last timestamp before the gap
        gap_duration = end_time - start_time
        print("Warn: Skipping the data segment from {}.{:09d} to {}.{:09d} of {:.3f} secs".format(
            start_time.secs, start_time.nsecs,
            end_time.secs, end_time.nsecs,
            gap_duration.to_sec()))
        return host_times[largest_gap_index + 1]
    else:
        return host_times[0]


def correct_livox_times(bag_path, topic, known_interval_ms, correct_time, start_time):
    host_times = []
    sensor_times = []
    with Bag(bag_path, 'r') as mid_bag:
        num_msg = 0
        for topic, msg, t in mid_bag.read_messages(topics=[topic]):
            if t < start_time:
                continue
            host_times.append(t)
            sensor_times.append(msg.header.stamp)
            num_msg += 1
    if not correct_time:
        return host_times

    # sanity checks
    true_duration = host_times[-1] - host_times[0]
    sensor_duration = sensor_times[-1] - sensor_times[0]
    gap = sensor_duration - true_duration
    gap_ms = gap.to_sec() * 1000
    if abs(gap_ms) > known_interval_ms:
        print('Warn: There may be missing sensor data, the host duration {}.{:09d}, '
              'the sensor duration {}.{:09d}, the gap {:.3f} ms, nominal interval {} ms'.format(
            true_duration.secs, true_duration.nsecs, sensor_duration.secs, sensor_duration.nsecs,
            gap_ms, known_interval_ms
        ))

    host_time_ref = host_times[0] - rospy.Time(100)
    host_times_float = [(ti - host_time_ref).to_sec() for ti in host_times]
    sensor_times_float = [ti.to_sec() for ti in sensor_times]
    TCor = TC.TimestampCorrector()
    host_times_corrected = []
    n = len(host_times_float)
    for i in range(n):
        TCor.correctTimestamp(sensor_times_float[i], host_times_float[i])
    for i in range(n):
        d = rospy.Duration.from_sec(TCor.getLocalTime(sensor_times_float[i]))
        host_times_corrected.append(host_time_ref + d)
    print('{} back time before correction {}.{:09d} after {}.{:09d}'.format(
        topic, host_times[-1].secs, host_times[-1].nsecs,
        host_times_corrected[-1].secs, host_times_corrected[-1].nsecs))
    return host_times_corrected


def merge_and_sort(host_times: Dict[str, List[float]]) -> Tuple[List[float], List[Tuple[str, int]]]:
    """
    Merges and sorts timestamps from multiple topics.

    Parameters:
        host_times (Dict[str, List[float]]): Dictionary with topic names as keys and lists of timestamps as values.

    Returns:
        Tuple[List[float], List[Tuple[str, int]]]:
            - all_host_times: Sorted list of all timestamps.
            - orig_topic_indices: List of tuples indicating the original topic and index of each timestamp.
    """
    all_host_times = []
    orig_topic_indices = []
    for topic, timestamps in host_times.items():
        for idx, time in enumerate(timestamps):
            all_host_times.append(time)
            orig_topic_indices.append((topic, idx))

    combined = list(zip(all_host_times, orig_topic_indices))
    combined.sort(key=lambda x: x[0])
    all_host_times_sorted, orig_topic_indices_sorted = zip(*combined) if combined else ([], [])
    return list(all_host_times_sorted), list(orig_topic_indices_sorted)


def correct_livox_times2(bag_path, topics, known_intervals_ms, correct_time, start_time):
    """
    assume these topics messages have the same sensor clock.
    """
    host_times = {}
    sensor_times = {}
    num_msgs = {}
    for t in topics:
        host_times[t] = []
        sensor_times[t] = []
        num_msgs[t] = 0
    
    with Bag(bag_path, 'r') as mid_bag:
        for topic, msg, t in mid_bag.read_messages(topics=topics):
            if t < start_time:
                continue
            host_times[topic].append(t)
            sensor_times[topic].append(msg.header.stamp)
            num_msgs[topic] += 1
    if not correct_time:
        return host_times

    # sanity checks
    for ti, topic in enumerate(topics):
        true_duration = host_times[topic][-1] - host_times[topic][0]
        sensor_duration = sensor_times[topic][-1] - sensor_times[topic][0]
        gap = sensor_duration - true_duration
        gap_ms = gap.to_sec() * 1000
        if abs(gap_ms) > known_intervals_ms[ti]:
            print('Warn: There may be missing sensor data on topic {}, the host duration {}.{:09d}, '
                'the sensor duration {}.{:09d}, the gap {:.3f} ms, nominal interval {} ms'.format(
                topic, true_duration.secs, true_duration.nsecs, sensor_duration.secs, sensor_duration.nsecs,
                gap_ms, known_intervals_ms[ti]
            ))

    all_sensor_times, orig_topic_indices = merge_and_sort(sensor_times)
    all_host_times = []
    for tp in orig_topic_indices:
        all_host_times.append(host_times[tp[0]][tp[1]])

    host_time_ref = all_host_times[0] - rospy.Time(100)
    host_times_float = [(ti - host_time_ref).to_sec() for ti in all_host_times]
    sensor_times_float = [ti.to_sec() for ti in all_sensor_times]
    TCor = TC.TimestampCorrector()
    host_times_corrected = []
    n = len(host_times_float)
    for i in range(n):
        TCor.correctTimestamp(sensor_times_float[i], host_times_float[i])
    for i in range(n):
        d = rospy.Duration.from_sec(TCor.getLocalTime(sensor_times_float[i]))
        host_times_corrected.append(host_time_ref + d)
    
    print('{} front time before correction {}.{:09d} after {}.{:09d}'.format(
        topics, all_host_times[0].secs, all_host_times[0].nsecs,
        host_times_corrected[0].secs, host_times_corrected[0].nsecs))

    print('{} back time before correction {}.{:09d} after {}.{:09d}'.format(
        topics, all_host_times[-1].secs, all_host_times[-1].nsecs,
        host_times_corrected[-1].secs, host_times_corrected[-1].nsecs))
    
    # recover host_times for each topic
    topic_host_times_corr = {}
    for topic in topics:
        topic_host_times_corr[topic] = [None] * len(host_times[topic])
    for ti, tp in enumerate(orig_topic_indices):
        topic_host_times_corr[tp[0]][tp[1]] = host_times_corrected[ti]
    for topic in topics:
        assert all(time_corr is not None for time_corr in topic_host_times_corr[topic]), \
            f"One or more None values found in topic '{topic}'"
    return topic_host_times_corr, TCor, host_time_ref


def load_timed_file(timed_file):
    """
    Load a timed file where each line contains a timestamp followed by other variables.
    
    :param timed_file: Path to the file containing time values.
    :return: NumPy array with parsed time values and associated data.
    """
    data = []
    with open(timed_file, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if parts:  # Ensure the line is not empty
                data.append([float(parts[0])] + parts[1:]) 

    return np.array(data, dtype=object)  # Use dtype=object to handle mixed types


def save_timed_file(data, filename):
    """
    Saves the corrected time data to a file with rospy.Time formatted as sec.nsec.
    
    :param data: List or NumPy array containing time-corrected data.
    :param filename: Path to the output file.
    """
    with open(filename, 'w') as f:
        for row in data:
            formatted_time = f"{row[0].secs}.{row[0].nsecs:09d}"
            other_values = " ".join(map(str, row[1:]))
            f.write(f"{formatted_time} {other_values}\n")


def correct_lidar_traj_times(TCor, host_time_ref, sensor_timed_file, host_timed_file):
    """
    Adjusts sensor timestamps using a time correction function and saves corrected times.

    :param TCor: A time correction object that provides getLocalTime().
    :param host_time_ref: A rospy.Time object representing the reference host time.
    :param sensor_timed_file: Path to the input sensor-timed file.
    :param host_timed_file: Path to the output corrected-timed file.
    """
    # Load sensor time data
    sensor_timed_data = load_timed_file(sensor_timed_file)
    sensor_times = sensor_timed_data[:, 0].astype(float)  # Convert first column to float
    n = len(sensor_times)

    host_times = [
        host_time_ref + rospy.Duration.from_sec(TCor.getLocalTime(sensor_times[i]))
        for i in range(n)
    ]

    # Combine corrected times with original data (excluding old timestamps)
    host_timed_data = np.column_stack((host_times, sensor_timed_data[:, 1:]))
    save_timed_file(host_timed_data, host_timed_file)


def merge_mid360_bag(seq_dir, correct_time):
    # Step 2: Read messages from mid360.bag and write to movie.bag
    mid_bag_path = os.path.join(seq_dir, 'mid360.bag')
    if not os.path.isfile(mid_bag_path):
        mid_bag_path = os.path.join(seq_dir, 'lidar.bag')
    lidar_interval_ms = 100 # ms
    imu_interval_ms = 5 # ms

    start_time = check_largest_gap(mid_bag_path, "/livox/lidar", lidar_interval_ms * 2)
    # corrected_lidar_times = correct_livox_times(mid_bag_path, "/livox/lidar", lidar_interval_ms, correct_time, start_time)
    # corrected_imu_times = correct_livox_times(mid_bag_path, "/livox/imu", imu_interval_ms, correct_time, start_time)

    corrected_times, TCor, host_time_ref = correct_livox_times2(mid_bag_path, ["/livox/lidar", "/livox/imu"], 
                                           [lidar_interval_ms, imu_interval_ms], correct_time, start_time)
    correct_lidar_traj_times(TCor, host_time_ref, os.path.join(seq_dir, 'faster_lio_traj.txt'), os.path.join(seq_dir, 'faster_lio_traj_synced.txt'))

    corrected_lidar_times = corrected_times["/livox/lidar"]
    corrected_imu_times = corrected_times["/livox/imu"]

    movie_bag_path = os.path.join(seq_dir, 'movie.bag')
    gnorm = 9.805 # https://github.com/Livox-SDK/LIO-Livox/blob/master/include/IMUIntegrator/IMUIntegrator.h#L84

    try:
        outputmode = 'a'
        if os.path.isfile(movie_bag_path) and os.path.getsize(movie_bag_path) > 100:
            outputmode = 'a'
        else:
            outputmode = 'w'

        with Bag(mid_bag_path, 'r') as mid_bag, Bag(movie_bag_path, outputmode) as movie_bag:
            num_msg = 0
            for topic, msg, t in mid_bag.read_messages(topics=["/livox/lidar"]):
                if t < start_time:
                    continue
                msg.header.stamp = corrected_lidar_times[num_msg]
                movie_bag.write(topic, msg, msg.header.stamp)
                num_msg += 1
            assert num_msg == len(corrected_lidar_times), "Inconsistent lidar messages and timestamps!"
            print("Finished writing {} lidar messages to movie.bag".format(num_msg))
            num_msg = 0
            for topic, msg, t in mid_bag.read_messages(topics=["/livox/imu"]):
                if t < start_time:
                    continue
                msg.header.stamp = corrected_imu_times[num_msg]
                msg.linear_acceleration.x *= gnorm
                msg.linear_acceleration.y *= gnorm
                msg.linear_acceleration.z *= gnorm
                movie_bag.write(topic, msg, msg.header.stamp)
                num_msg += 1
            assert num_msg == len(corrected_imu_times), "Inconsistent IMU messages and timestamps!"
            print("Finished writing {} imu messages to movie.bag".format(num_msg))
    except Exception as e:
        print(f"Error reading or writing bag files: {e}")
        sys.exit(1)


def correct_pandar_bag_times(bagname, corrbagname, topic):
    import sensor_msgs.point_cloud2 as pc2
    # from sensor_msgs.msg import PointCloud2, PointField
    # import std_msgs.msg
    all_host_times = []
    all_sensor_times = []

    # 1. Load all host times and sensor times
    with Bag(bagname, 'r') as bag:
        for topic_, msg, t in bag.read_messages(topics=[topic]):
            all_host_times.append(t)
            all_sensor_times.append(msg.header.stamp)

    if not all_host_times:
        print(f"No messages found in {bagname} on topic {topic}")
        return

    host_time_ref = all_host_times[0] - rospy.Duration(100)
    if all_sensor_times[0].secs > 100:
        sensor_time_ref = all_sensor_times[0] - rospy.Duration(100)
    else:
        sensor_time_ref = rospy.Time(0)

    host_times_float = [(t - host_time_ref).to_sec() for t in all_host_times]
    sensor_times_float = [(t - sensor_time_ref).to_sec() for t in all_sensor_times]

    TCor = TC.TimestampCorrector()
    for sensor_t, host_t in zip(sensor_times_float, host_times_float):
        TCor.correctTimestamp(sensor_t, host_t)

    host_times_corrected = []
    for sensor_t in sensor_times_float:
        d = rospy.Duration.from_sec(TCor.getLocalTime(sensor_t))
        host_times_corrected.append(host_time_ref + d)

    print(f'{topic} front time before correction {all_host_times[0].secs}.{all_host_times[0].nsecs:09d} after {host_times_corrected[0].secs}.{host_times_corrected[0].nsecs:09d}')
    print(f'{topic} back time before correction {all_host_times[-1].secs}.{all_host_times[-1].nsecs:09d} after {host_times_corrected[-1].secs}.{host_times_corrected[-1].nsecs:09d}')

    # 2. Correct point timestamps and save to new bag
    with Bag(corrbagname, 'w') as outbag, Bag(bagname, 'r') as bag:
        idx = 0
        for topic_, msg, t in bag.read_messages(topics=[topic]):
            corrected_time = host_times_corrected[idx]
            fields = msg.fields
            points = list(pc2.read_points(msg, field_names=["x", "y", "z", "intensity", "timestamp", "ring"], skip_nans=True))
            
            updated_points = []
            for p in points:
                p = list(p)
                sensor_time = p[4]  # timestamp field
                corrected_sensor_time = host_time_ref.to_sec() + TCor.getLocalTime(sensor_time - sensor_time_ref.to_sec())
                p[4] = corrected_sensor_time
                updated_points.append(tuple(p))

            corrected_msg = pc2.create_cloud(msg.header, fields, updated_points)
            corrected_msg.header.stamp = corrected_time

            outbag.write(topic, corrected_msg, t=corrected_time)
            idx += 1


def need_time_correction(lidar_bagpath, topic="/hesai/pandar", threshold_ns=1e9):
    with Bag(lidar_bagpath, 'r') as bag:
        for topic_, msg, t in bag.read_messages(topics=[topic]):
            ros_time = t
            header_stamp = msg.header.stamp
            diff = abs((ros_time - header_stamp).to_nsec())
            if diff > threshold_ns:
                return True
            else:
                return False
    return False


def merge_pandar_bag(seq_dir):
    lidar_bagpath = os.path.join(seq_dir, 'lidar.bag')
    movie_bagpath = os.path.join(seq_dir, 'movie.bag')

    lidar_corr_bagpath = os.path.join(seq_dir, 'lidar_corr.bag')
    correct_time = need_time_correction(lidar_bagpath, topic="/hesai/pandar")

    if correct_time:
        correct_pandar_bag_times(lidar_bagpath, lidar_corr_bagpath, "/hesai/pandar")
        lidar_to_merge = lidar_corr_bagpath
    else:
        lidar_to_merge = lidar_bagpath

    with Bag(movie_bagpath, 'a') as movie_bag:
        with Bag(lidar_to_merge, 'r') as lidar_bag:
            for topic, msg, t in lidar_bag.read_messages():
                movie_bag.write(topic, msg, t=t)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Process livox phone sequence with optional time correction.")
    parser.add_argument("seq_dir", type=str, help="Path to the sequence directory.")
    parser.add_argument("--no-correct-time", action="store_false", dest="correct_time", help="Disable time correction.")
    args = parser.parse_args()

    print(f"Sequence Directory: {args.seq_dir}")
    print(f"Time correction: {args.correct_time}")

    seq_list = []
    for root, dirs, files in os.walk(args.seq_dir):
        for fn in files:
            if fn == 'mid360.bag' or fn == 'lidar.bag':
                seq_list.append(root)
    print(f'Found {len(seq_list)} sequences in {args.seq_dir}')
    for index, folder in enumerate(seq_list, start=1):
        print(f'{index}: {folder}')

    numseqs = len(seq_list)
    for s, folder in enumerate(seq_list):
        print(f'Processing {s}/{numseqs} {folder}')
        create_video_bag(folder)
        lidar_bagpath = os.path.join(folder, 'lidar.bag')
        with Bag(lidar_bagpath, 'r') as bag:
            topic_info = bag.get_type_and_topic_info()
            topics = set(topic_info[1].keys())
            print(f"Topics in {lidar_bagpath}: {topics}")

        if '/hesai/pandar' in topics:
            merge_pandar_bag(folder)
        elif '/livox/lidar' in topics:
            merge_mid360_bag(folder, args.correct_time)
        else:
            print(f"No known lidar topics found in {lidar_bagpath}")

