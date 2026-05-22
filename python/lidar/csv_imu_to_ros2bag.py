#!/usr/bin/env python3
import argparse
import csv
import math
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path


STANDARD_GRAVITY = 9.80665
DEG_TO_RAD = math.pi / 180.0
GPS_EPOCH_UNIX_S = 315964800
NS_PER_S = 1_000_000_000
MS_TO_NS = 1_000_000
GPS_WEEK_S = 7 * 24 * 60 * 60
GPS_WEEK_NS = GPS_WEEK_S * NS_PER_S
DEFAULT_FRONT_GAP_S = 10.0

# UTC dates where GPS-UTC increased by one second. GPS time started aligned
# with UTC on 1980-01-06 and is 18 seconds ahead of UTC from 2017 onward.
GPS_LEAP_SECOND_DATES = tuple(
    datetime(year, month, day, tzinfo=timezone.utc)
    for year, month, day in (
        (1981, 7, 1),
        (1982, 7, 1),
        (1983, 7, 1),
        (1985, 7, 1),
        (1988, 1, 1),
        (1990, 1, 1),
        (1991, 1, 1),
        (1992, 7, 1),
        (1993, 7, 1),
        (1994, 7, 1),
        (1996, 1, 1),
        (1997, 7, 1),
        (1999, 1, 1),
        (2006, 1, 1),
        (2009, 1, 1),
        (2012, 7, 1),
        (2015, 7, 1),
        (2017, 1, 1),
    )
)


def millisec_to_time_ns(value):
    """Convert a millisecond timestamp to integer nanoseconds."""
    try:
        return int(round(float(value) * MS_TO_NS))
    except ValueError as exc:
        raise ValueError(f'Bad millisecond timestamp value: {value!r}') from exc


def gps_utc_offset_s(utc_time):
    """Return GPS-UTC offset in seconds for a timezone-aware UTC datetime."""
    return sum(1 for leap_date in GPS_LEAP_SECOND_DATES if utc_time >= leap_date)


def parse_gps_date(gps_date):
    try:
        return datetime.strptime(gps_date, '%Y%m%d').replace(tzinfo=timezone.utc)
    except ValueError as exc:
        raise ValueError(
            f'Bad GPS date {gps_date!r}; expected YYYYMMDD, e.g. 20260513'
        ) from exc


def gps_week_from_date(gps_date):
    """Return the GPS week containing a YYYYMMDD UTC date.

    Dates before the GPS epoch are accepted as a convenience marker and select
    GPS week 0.
    """
    utc_date = parse_gps_date(gps_date)
    unix_s = int(utc_date.timestamp())
    gps_s = unix_s - GPS_EPOCH_UNIX_S + gps_utc_offset_s(utc_date)
    if gps_s < 0:
        return 0
    return gps_s // GPS_WEEK_S


def gps_week_tow_ns_to_unix_ns(gps_week, tow_ns):
    gps_elapsed_ns = gps_week * GPS_WEEK_NS + tow_ns

    # GPS time is ahead of UTC by a leap-second offset. Resolve the offset
    # iteratively so timestamps around leap-second boundaries are still sane.
    offset_s = 18
    unix_ns = GPS_EPOCH_UNIX_S * NS_PER_S + gps_elapsed_ns
    for _ in range(4):
        unix_ns = (
            GPS_EPOCH_UNIX_S * NS_PER_S
            + gps_elapsed_ns
            - offset_s * NS_PER_S
        )
        utc_time = datetime.fromtimestamp(unix_ns / NS_PER_S, tz=timezone.utc)
        next_offset_s = gps_utc_offset_s(utc_time)
        if next_offset_s == offset_s:
            break
        offset_s = next_offset_s

    return unix_ns


def make_gps_tow_ms_converter(gps_date):
    gps_week = gps_week_from_date(gps_date)
    previous_tow_ns = None

    def convert(value):
        nonlocal gps_week, previous_tow_ns
        tow_ns = millisec_to_time_ns(value)
        if tow_ns < 0 or tow_ns >= GPS_WEEK_NS:
            raise ValueError(
                f'Bad GPS time-of-week millisecond value: {value!r}; '
                f'expected [0, {GPS_WEEK_S * 1000})'
            )

        if (
                previous_tow_ns is not None
                and previous_tow_ns - tow_ns > GPS_WEEK_NS // 2):
            gps_week += 1
        previous_tow_ns = tow_ns

        return gps_week_tow_ns_to_unix_ns(gps_week, tow_ns)

    return convert


def discard_before_front_gap(samples, min_gap_s=DEFAULT_FRONT_GAP_S):
    if len(samples) < 2 or min_gap_s <= 0:
        return samples

    min_gap_ns = int(round(min_gap_s * NS_PER_S))
    for idx, ((t1, _, _, _), (t2, _, _, _)) in enumerate(
            zip(samples, samples[1:]),
            start=1):
        gap_ns = t2 - t1
        if gap_ns >= min_gap_ns:
            discarded = samples[:idx]
            kept = samples[idx:]
            print(
                '[IMU] Warning: discarded '
                f'{len(discarded)} leading sample(s) before a '
                f'{gap_ns / NS_PER_S:.3f}s time gap. '
                f'First kept timestamp: {t2 / NS_PER_S:.9f}.',
                file=sys.stderr,
            )
            return kept

    return samples


def read_csv_imu(csv_path, gps_date=None, front_gap_s=DEFAULT_FRONT_GAP_S):
    """
    Read CSV columns:
      time_ms, ax_g, ay_g, az_g, gx_deg_s, gy_deg_s, gz_deg_s, temperature_c

    If gps_date is provided, time_ms is interpreted as GPS time-of-week in
    milliseconds for the GPS week containing that YYYYMMDD UTC date.
    """
    samples = []
    csv_path = Path(csv_path)
    time_converter = (
        make_gps_tow_ms_converter(gps_date)
        if gps_date
        else millisec_to_time_ns
    )

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
                t_ns = time_converter(row[0])
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

    samples = discard_before_front_gap(samples, min_gap_s=front_gap_s)

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
        gps_date=None,
        front_gap_s=DEFAULT_FRONT_GAP_S,
        overwrite=False):
    samples = read_csv_imu(
        imu_csv,
        gps_date=gps_date,
        front_gap_s=front_gap_s,
    )

    if not samples:
        raise RuntimeError(f'No IMU samples found in {imu_csv}')

    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu

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
        '--gps-date',
        type=str,
        help=('Interpret CSV time_ms as GPS time-of-week milliseconds in the '
              'GPS week containing this UTC date (YYYYMMDD, e.g. 20260513), '
              'then convert to Unix time. Dates before 19800106 select GPS '
              'week 0.'),
    )
    parser.add_argument(
        '--storage-id',
        default='sqlite3',
        help='ROS2 bag storage plugin id (default: sqlite3)',
    )
    parser.add_argument(
        '--front-gap-s',
        type=float,
        default=DEFAULT_FRONT_GAP_S,
        help=('Discard leading IMU samples before the first time gap of at '
              'least this many seconds (default: 10.0; set <= 0 to disable).'),
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

    try:
        write_imu_bag(
            imu_csv=imu_csv,
            out_bag_path=out_bag,
            imu_topic=args.imu_topic,
            frame_id=args.frame_id,
            storage_id=args.storage_id,
            gps_date=args.gps_date,
            front_gap_s=args.front_gap_s,
            overwrite=args.overwrite,
        )
    except (FileExistsError, RuntimeError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    main()
