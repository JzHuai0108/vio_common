#!/usr/bin/env python3
import argparse
import csv
import shutil
from bisect import bisect_left
from pathlib import Path

import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu


def to_time_ns(value):
    """Accept seconds or nanoseconds and return integer nanoseconds."""
    try:
        value_f = float(value)
    except ValueError as exc:
        raise ValueError(f'Bad timestamp value: {value!r}') from exc

    if value_f > 1e12:
        return int(value_f)
    return int(round(value_f * 1e9))


def read_csv_sensor_vec3(csv_path):
    """
    Read CSV columns: host_time, x, y, z[, sensor_time].

    If sensor_time exists, it is used for the ROS timestamp. Otherwise the first
    column is used.
    """
    times_ns = []
    vecs = []
    csv_path = Path(csv_path)

    with csv_path.open('r', newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].strip().startswith('#'):
                continue

            x, y, z = float(row[1]), float(row[2]), float(row[3])
            time_col = 4 if len(row) >= 5 else 0
            times_ns.append(to_time_ns(row[time_col]))
            vecs.append((x, y, z))

    if any(t2 < t1 for t1, t2 in zip(times_ns, times_ns[1:])):
        paired = sorted(zip(times_ns, vecs))
        times_ns = [t for t, _ in paired]
        vecs = [v for _, v in paired]

    return times_ns, vecs


def interpolate_vec3(t_ns, t0_ns, v0, t1_ns, v1):
    if t1_ns == t0_ns:
        return v0

    alpha = (t_ns - t0_ns) / float(t1_ns - t0_ns)
    return (
        v0[0] + (v1[0] - v0[0]) * alpha,
        v0[1] + (v1[1] - v0[1]) * alpha,
        v0[2] + (v1[2] - v0[2]) * alpha,
    )


def interpolate_to_targets(src_times, src_vecs, target_times, clamp=True):
    out = []
    for t_ns in target_times:
        index = bisect_left(src_times, t_ns)
        if index == 0:
            out.append(src_vecs[0] if clamp else None)
        elif index >= len(src_times):
            out.append(src_vecs[-1] if clamp else None)
        else:
            out.append(interpolate_vec3(
                t_ns,
                src_times[index - 1],
                src_vecs[index - 1],
                src_times[index],
                src_vecs[index],
            ))
    return out


def create_topic_metadata(topic):
    kwargs = {
        'name': topic,
        'type': 'sensor_msgs/msg/Imu',
        'serialization_format': 'cdr',
    }

    try:
        return rosbag2_py.TopicMetadata(**kwargs, offered_qos_profiles='')
    except TypeError:
        return rosbag2_py.TopicMetadata(**kwargs)


def make_imu_msg(t_ns, gyro, accel, frame_id):
    msg = Imu()
    msg.header.stamp.sec = t_ns // 1_000_000_000
    msg.header.stamp.nanosec = t_ns % 1_000_000_000
    msg.header.frame_id = frame_id

    msg.angular_velocity.x = gyro[0]
    msg.angular_velocity.y = gyro[1]
    msg.angular_velocity.z = gyro[2]

    msg.linear_acceleration.x = accel[0]
    msg.linear_acceleration.y = accel[1]
    msg.linear_acceleration.z = accel[2]

    msg.orientation_covariance[0] = -1.0
    msg.angular_velocity_covariance[0] = -1.0
    msg.linear_acceleration_covariance[0] = -1.0

    return msg


def write_imu_bag(
        accel_csv,
        gyro_csv,
        out_bag_path,
        imu_topic='/imu/data',
        frame_id='imu_link',
        clamp_out_of_range=True,
        storage_id='sqlite3',
        overwrite=False):
    acc_t_ns, acc_xyz = read_csv_sensor_vec3(accel_csv)
    gyr_t_ns, gyr_xyz = read_csv_sensor_vec3(gyro_csv)

    if not acc_t_ns:
        raise RuntimeError(f'No accel samples found in {accel_csv}')
    if not gyr_t_ns:
        raise RuntimeError(f'No gyro samples found in {gyro_csv}')

    out_bag_path = Path(out_bag_path)
    if out_bag_path.exists():
        if not overwrite:
            raise FileExistsError(
                f'Output bag already exists: {out_bag_path}. '
                'Pass --overwrite to replace it.'
            )
        if out_bag_path.is_dir():
            shutil.rmtree(out_bag_path)
        else:
            out_bag_path.unlink()

    out_bag_path.parent.mkdir(parents=True, exist_ok=True)

    acc_interp = interpolate_to_targets(
        acc_t_ns,
        acc_xyz,
        gyr_t_ns,
        clamp=clamp_out_of_range,
    )

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(out_bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    writer.create_topic(create_topic_metadata(imu_topic))

    written = 0
    skipped = 0
    for t_ns, gyro, accel in zip(gyr_t_ns, gyr_xyz, acc_interp):
        if accel is None:
            skipped += 1
            continue

        msg = make_imu_msg(t_ns, gyro, accel, frame_id)
        writer.write(imu_topic, serialize_message(msg), t_ns)
        written += 1

    print(f'[IMU] Wrote {written} messages to {out_bag_path} on {imu_topic} '
          f'(skipped {skipped}).')


def main():
    parser = argparse.ArgumentParser(
        description='Create a ROS2 bag containing sensor_msgs/msg/Imu messages '
                    'from accel.csv and gyro.csv.'
    )
    parser.add_argument(
        'imu_dir',
        type=str,
        help='Directory containing accel.csv and gyro.csv.',
    )
    parser.add_argument(
        '--out_bag',
        type=str,
        help='Output ROS2 bag directory (default: [imu_dir]/d455imu_ros2bag).',
    )
    parser.add_argument('--imu-topic', default='/imu/data', help='IMU topic name')
    parser.add_argument('--frame-id', default='imu_link', help='IMU frame_id')
    parser.add_argument(
        '--storage-id',
        default='sqlite3',
        help='ROS2 bag storage plugin id (default: sqlite3)',
    )
    parser.add_argument(
        '--no-clamp',
        action='store_true',
        help='Skip gyro samples outside accel time span instead of clamping.',
    )
    parser.add_argument(
        '--overwrite',
        action='store_true',
        help='Replace the output bag directory if it already exists.',
    )

    args = parser.parse_args()
    imu_dir = Path(args.imu_dir).expanduser().resolve()
    out_bag = (
        Path(args.out_bag).expanduser().resolve()
        if args.out_bag
        else imu_dir / 'd455imu_ros2bag'
    )

    write_imu_bag(
        accel_csv=imu_dir / 'accel.csv',
        gyro_csv=imu_dir / 'gyro.csv',
        out_bag_path=out_bag,
        imu_topic=args.imu_topic,
        frame_id=args.frame_id,
        clamp_out_of_range=not args.no_clamp,
        storage_id=args.storage_id,
        overwrite=args.overwrite,
    )


if __name__ == '__main__':
    main()
