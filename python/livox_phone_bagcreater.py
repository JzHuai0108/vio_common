
import os
import subprocess
import sys
from rosbag import Bag
import rospy

sys.path.append('../timestamp_corrector/build')
import TimestampCorrector as TC


def create_video_bag(seq_dir):
    # Step 1: Call the kalibr_bagcreater.py script with the specified arguments
    video_path = os.path.join(seq_dir, 'movie.mp4')
    imu_path = os.path.join(seq_dir, 'gyro_accel.csv')
    video_time_file_path = os.path.join(seq_dir, 'frame_timestamps.txt')
    output_bag_path = os.path.join(seq_dir, 'movie.bag')

    kalibr_command = [
        'python', 'kalibr_bagcreater.py',
        '--video', video_path,
        '--imu', imu_path,
        '--video_time_file', video_time_file_path,
        '--output_bag', output_bag_path
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
            host_times.append(msg.header.stamp)

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
    sensor_times = [rospy.Time(100, 0)]
    with Bag(bag_path, 'r') as mid_bag:
        num_msg = 0
        for topic, msg, t in mid_bag.read_messages(topics=[topic]):
            if msg.header.stamp < start_time:
                continue
            host_times.append(msg.header.stamp)
            sensor_times.append(sensor_times[-1] + rospy.Duration(0, known_interval_ms * 1000000))
            num_msg += 1
    if not correct_time:
        return host_times

    sensor_times.pop()

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


def merge_mid360_bag(seq_dir, correct_time):
    # Step 2: Read messages from mid360.bag and write to movie.bag
    mid_bag_path = os.path.join(seq_dir, 'mid360.bag')
    lidar_interval_ms = 100 # ms
    imu_interval_ms = 5 # ms

    start_time = check_largest_gap(mid_bag_path, "/livox/lidar", lidar_interval_ms * 2)
    corrected_lidar_times = correct_livox_times(mid_bag_path, "/livox/lidar", lidar_interval_ms, correct_time, start_time)
    corrected_imu_times = correct_livox_times(mid_bag_path, "/livox/imu", imu_interval_ms, correct_time, start_time)

    movie_bag_path = os.path.join(seq_dir, 'movie.bag')
    gnorm = 9.805 # https://github.com/Livox-SDK/LIO-Livox/blob/master/include/IMUIntegrator/IMUIntegrator.h#L84

    try:
        with Bag(mid_bag_path, 'r') as mid_bag, Bag(movie_bag_path, 'a') as movie_bag:
            num_msg = 0
            for topic, msg, t in mid_bag.read_messages(topics=["/livox/lidar"]):
                if msg.header.stamp < start_time:
                    continue
                msg.header.stamp = corrected_lidar_times[num_msg]
                movie_bag.write(topic, msg, corrected_lidar_times[num_msg])
                num_msg += 1
            assert num_msg == len(corrected_lidar_times), "Inconsistent lidar messages and timestamps!"
            print("Finished writing {} lidar messages to movie.bag".format(num_msg))
            num_msg = 0
            for topic, msg, t in mid_bag.read_messages(topics=["/livox/imu"]):
                if msg.header.stamp < start_time:
                    continue
                msg.header.stamp = corrected_imu_times[num_msg]
                msg.linear_acceleration.x *= gnorm
                msg.linear_acceleration.y *= gnorm
                msg.linear_acceleration.z *= gnorm
                movie_bag.write(topic, msg, corrected_imu_times[num_msg])
                num_msg += 1
            assert num_msg == len(corrected_imu_times), "Inconsistent IMU messages and timestamps!"
            print("Finished writing {} imu messages to movie.bag".format(num_msg))
    except Exception as e:
        print(f"Error reading or writing bag files: {e}")
        sys.exit(1)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Process livox phone sequence with optional time correction.")
    parser.add_argument("seq_dir", type=str, help="Path to the sequence directory.")
    parser.add_argument("--no-correct-time", action="store_false", dest="correct_time", help="Disable time correction.")
    args = parser.parse_args()

    print(f"Sequence Directory: {args.seq_dir}")
    print(f"Time correction: {args.correct_time}")

    create_video_bag(args.seq_dir)
    merge_mid360_bag(args.seq_dir, args.correct_time)

