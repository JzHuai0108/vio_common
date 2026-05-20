#!/usr/bin/env python3
import argparse
import csv
import math
import shutil
from pathlib import Path


STANDARD_GRAVITY = 9.80665
DEG_TO_RAD = math.pi / 180.0


def millisec_to_time_ns(value):
    """Convert a millisecond timestamp to integer nanoseconds."""
    try:
        return int(round(float(value) * 1_000_000))
    except ValueError as exc:
        raise ValueError(f'Bad millisecond timestamp value: {value!r}') from exc


def read_csv_imu(csv_path):
    """
    Read CSV columns:
      time_ms, ax_g, ay_g, az_g, gx_deg_s, gy_deg_s, gz_deg_s, temperature_c
    """
    samples = []
    csv_path = Path(csv_path)

    with csv_path.open('r', newline='') as f:
        reader = csv.reader(f)
        for line_num, row in enumerate(reader, start=1):
            if not row or row[0].strip().startswith('#'):
                continue

            row = [cell.strip() for cell in row]
            if len(row) < 8:
                raise ValueError(
                    f'{csv_path}:{line_num}: expected at least 8 columns, '
                    f'got {len(row)}'
                )

            try:
                t_ns = millisec_to_time_ns(row[0])
                accel = (
                    float(row[1]) * STANDARD_GRAVITY,
                    float(row[2]) * STANDARD_GRAVITY,
                    float(row[3]) * STANDARD_GRAVITY,
                )
                gyro = (
                    float(row[4]) * DEG_TO_RAD,
                    float(row[5]) * DEG_TO_RAD,
                    float(row[6]) * DEG_TO_RAD,
                )
                temperature_c = float(row[7])
            except ValueError:
                if line_num == 1:
                    continue
                raise ValueError(
                    f'{csv_path}:{line_num}: failed to parse IMU row: {row!r}'
                ) from None

            samples.append((t_ns, gyro, accel, temperature_c))

    if any(t2 < t1 for (t1, _, _, _), (t2, _, _, _) in zip(samples, samples[1:])):
        samples.sort(key=lambda sample: sample[0])

    return samples


def create_topic_metadata(rosbag2_py, topic):
    kwargs = {
        'name': topic,
        'type': 'sensor_msgs/msg/Imu',
        'serialization_format': 'cdr',
    }

    try:
        return rosbag2_py.TopicMetadata(**kwargs, offered_qos_profiles='')
    except TypeError:
        return rosbag2_py.TopicMetadata(**kwargs)


def make_imu_msg(imu_msg_type, t_ns, gyro, accel, frame_id):
    msg = imu_msg_type()
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
        imu_csv,
        out_bag_path,
        imu_topic='/imu/data',
        frame_id='imu_link',
        storage_id='sqlite3',
        overwrite=False):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu

    samples = read_csv_imu(imu_csv)

    if not samples:
        raise RuntimeError(f'No IMU samples found in {imu_csv}')

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

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(out_bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    writer.create_topic(create_topic_metadata(rosbag2_py, imu_topic))

    written = 0
    for t_ns, gyro, accel, _temperature_c in samples:
        msg = make_imu_msg(Imu, t_ns, gyro, accel, frame_id)
        writer.write(imu_topic, serialize_message(msg), t_ns)
        written += 1

    print(f'[IMU] Wrote {written} messages to {out_bag_path} on {imu_topic}.')


def main():
    parser = argparse.ArgumentParser(
        description='Create a ROS2 bag containing sensor_msgs/msg/Imu messages '
                    'from a single CSV file.'
    )
    parser.add_argument(
        'imu_csv',
        type=str,
        help=('CSV containing columns: time_ms, ax_g, ay_g, az_g, '
              'gx_deg_s, gy_deg_s, gz_deg_s, temperature_c.'),
    )
    parser.add_argument(
        '--out_bag',
        type=str,
        help='Output ROS2 bag directory (default: [imu_csv_dir]/imu).',
    )
    parser.add_argument('--imu-topic', default='/imu/data', help='IMU topic name')
    parser.add_argument('--frame-id', default='imu_link', help='IMU frame_id')
    parser.add_argument(
        '--storage-id',
        default='sqlite3',
        help='ROS2 bag storage plugin id (default: sqlite3)',
    )
    parser.add_argument(
        '--overwrite',
        action='store_true',
        help='Replace the output bag directory if it already exists.',
    )

    args = parser.parse_args()
    imu_csv = Path(args.imu_csv).expanduser().resolve()
    out_bag = (
        Path(args.out_bag).expanduser().resolve()
        if args.out_bag
        else imu_csv.parent / 'imu'
    )

    write_imu_bag(
        imu_csv=imu_csv,
        out_bag_path=out_bag,
        imu_topic=args.imu_topic,
        frame_id=args.frame_id,
        storage_id=args.storage_id,
        overwrite=args.overwrite,
    )


if __name__ == '__main__':
    main()
