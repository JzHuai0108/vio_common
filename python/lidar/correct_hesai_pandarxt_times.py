#!/usr/bin/env python3
import os, sys, csv
from pathlib import Path
from collections import deque

from bisect import bisect_left
import argparse

import rosbag
import rospy
from sensor_msgs.msg import Imu

# ---- import your timestamp corrector (pybind11 module) ----
MYMOD_PATH = os.path.abspath('../timestamp_corrector/build')
if MYMOD_PATH not in sys.path:
    sys.path.append(MYMOD_PATH)
import TimestampCorrector as TC  # provides TimestampCorrector()

def header_time_ns(msg):
    """Return msg.header.stamp as int nanoseconds, or None if missing."""
    h = getattr(msg, 'header', None)
    return int(h.stamp.secs) * 1_000_000_000 + int(h.stamp.nsecs)

def to_time(ns: int) -> rospy.Time:
    return rospy.Time(secs=ns // 1_000_000_000, nsecs=ns % 1_000_000_000)

def to_time_ns(x):
    """Accept float seconds or int nanoseconds and return int nanoseconds."""
    # If it's obviously ns-scale, keep it; else treat as seconds.
    try:
        xf = float(x)
    except Exception:
        raise ValueError(f"Bad timestamp value: {x!r}")
    if xf > 1e12:   # already in ns
        return int(xf)
    return int(round(xf * 1e9))

def read_csv_sensor_vec3(csv_path):
    """
    Read CSV with a header line starting with '#'.
    Columns: host_time, x, y, z, sensor_time
    Returns two lists (sorted by sensor_time_ns):
      times_ns: [t_ns, ...]
      vecs:     [(x,y,z), ...]
    """
    times_ns, vecs = [], []
    with open(csv_path, 'r', newline='') as f:
        reader = csv.reader(f)
        # skip header lines starting with '#'
        for row in reader:
            if not row:
                continue
            if row[0].strip().startswith('#'):
                continue
            # host_time = row[0] (ignored)
            x = float(row[1]); y = float(row[2]); z = float(row[3])
            t_ns = to_time_ns(row[4])
            times_ns.append(t_ns)
            vecs.append((x, y, z))
    # ensure sorted by time (just in case)
    if any(t2 < t1 for t1, t2 in zip(times_ns, times_ns[1:])):
        paired = sorted(zip(times_ns, vecs))
        times_ns = [t for t, _ in paired]
        vecs     = [v for _, v in paired]
    return times_ns, vecs

def lerp(a, b, alpha):
    return a + (b - a) * alpha

def interp_vec3(t_ns, t0_ns, v0, t1_ns, v1):
    if t1_ns == t0_ns:
        return v0
    alpha = (t_ns - t0_ns) / float(t1_ns - t0_ns)
    return (lerp(v0[0], v1[0], alpha),
            lerp(v0[1], v1[1], alpha),
            lerp(v0[2], v1[2], alpha))

def interpolate_to_targets(src_times, src_vecs, tgt_times, mode='clamp'):
    """
    For each tgt_times[i], return a vec3 interpolated from src series.
    mode:
      - 'clamp': use nearest endpoint value when outside src range
      - 'skip' : return None when outside range
    """
    out = []
    for t in tgt_times:
        j = bisect_left(src_times, t)
        if j == 0:
            if mode == 'skip':
                out.append(None)
                continue
            out.append(src_vecs[0])  # clamp to first
        elif j >= len(src_times):
            if mode == 'skip':
                out.append(None)
                continue
            out.append(src_vecs[-1]) # clamp to last
        else:
            t0 = src_times[j-1]; v0 = src_vecs[j-1]
            t1 = src_times[j];   v1 = src_vecs[j]
            out.append(interp_vec3(t, t0, v0, t1, v1))
    return out

def ns_to_rospy_time(ns):
    return rospy.Time(secs=ns // 1_000_000_000, nsecs=ns % 1_000_000_000)

def append_imu_data(accel_csv, gyro_csv, out_bag_path, imu_topic='/imu/data',
                    frame_id='imu_link', clamp_out_of_range=True):
    # 1) Load accel and gyro (using sensor timestamps)
    acc_t_ns, acc_xyz = read_csv_sensor_vec3(accel_csv)
    gyr_t_ns, gyr_xyz = read_csv_sensor_vec3(gyro_csv)

    if not acc_t_ns:
        raise RuntimeError(f"No accel samples found in {accel_csv}")
    if not gyr_t_ns:
        raise RuntimeError(f"No gyro samples found in {gyro_csv}")

    # 2) Interpolate accel to gyro timestamps
    mode = 'clamp' if clamp_out_of_range else 'skip'
    acc_interp = interpolate_to_targets(acc_t_ns, acc_xyz, gyr_t_ns, mode=mode)

    # 3) Open bag in append mode and write IMU messages at gyro cadence
    out_bag = Path(out_bag_path)
    if not out_bag.exists():
        raise FileNotFoundError(f"Output bag does not exist (open in 'a'): {out_bag}")
    written = 0
    skipped = 0

    with rosbag.Bag(str(out_bag), 'a') as bag:
        for t_ns, g, a in zip(gyr_t_ns, gyr_xyz, acc_interp):
            if a is None:  # outside accel range and clamp_out_of_range=False
                skipped += 1
                continue

            imu = Imu()
            imu.header.stamp = ns_to_rospy_time(t_ns)
            imu.header.frame_id = frame_id

            # Angular velocity from gyro (rad/s)
            imu.angular_velocity.x = g[0]
            imu.angular_velocity.y = g[1]
            imu.angular_velocity.z = g[2]
            # Linear acceleration from interpolated accel (m/s^2)
            imu.linear_acceleration.x = a[0]
            imu.linear_acceleration.y = a[1]
            imu.linear_acceleration.z = a[2]

            # Covariances: unknown → set to -1 (per ROS convention)
            imu.orientation_covariance[0] = -1.0
            imu.angular_velocity_covariance[0] = -1.0
            imu.linear_acceleration_covariance[0] = -1.0

            # Write with bag timestamp == header.stamp (typical)
            t_ros = imu.header.stamp
            bag.write(imu_topic, imu, t_ros)
            written += 1

    print(f"[IMU] Wrote {written} messages to {imu_topic} (skipped {skipped}).")


def main():
    ap = argparse.ArgumentParser(description='Rewrite LiDAR times in a ROS1 bag using TimestampCorrector.')
    ap.add_argument('in_bag', help='Input ROS1 .bag file. It can be converted from ros2 bag by using "rosbags-convert <ros2_bag_dir> --dst <ros1.bag>"')
    ap.add_argument('--imu_dir', type=str, help='Directory containing accel.csv and gyro.csv. If not provided, will set to [in_bag.base_dir]/../realsense/imu')
    ap.add_argument('--out_bag', type=str, help='Output ROS1 .bag file. If not provided, will set to [in_bag.base_dir]/xt32_d455imu.bag')
    ap.add_argument('--topic',  default='/lidar_points', help='Topic to correct (default: /lidar_points)')
    ap.add_argument('--ref-offset-sec', type=float, default=100.0,
                    help='Seconds to subtract from first host time to build a reference (default: 100)')
    ap.add_argument('--imu-topic', default='/imu/data', help='IMU topic name')
    ap.add_argument('--frame-id',  default='imu_link', help='IMU frame_id')
    ap.add_argument('--no-clamp', action='store_true',
                        help='If set, skip gyro samples outside accel time span (default: clamp to edges)')

    args = ap.parse_args()

    in_bag  = Path(args.in_bag).expanduser().resolve()
    if args.imu_dir:
        imu_dir = Path(args.imu_dir).expanduser().resolve()
    else:
        imu_dir = in_bag.parent.parent / 'realsense' / 'imu'
        imu_dir = imu_dir.resolve()
    if args.out_bag:
        out_bag = Path(args.out_bag).expanduser().resolve()
    else:
        out_bag = in_bag.parent / 'xt32_d455imu.bag'
        out_bag = out_bag.resolve()
    if out_bag == in_bag:
        out_bag = in_bag.parent / 'xt32_d455imu_corrected.bag'

    topic   = args.topic

    if not in_bag.exists():
        print(f"[ERR] Input bag not found: {in_bag}")
        sys.exit(2)

    # -------- PASS 1: collect times --------
    sensor_ns = []
    host_ns   = []

    print(f"[INFO] Scanning {in_bag} for topic: {topic}")
    with rosbag.Bag(str(in_bag), 'r') as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            hns = header_time_ns(msg)
            if hns is None:
                continue
            sensor_ns.append(hns)
            host_ns.append(int(t.to_nsec()))

    if not sensor_ns:
        print(f"[ERR] No messages with Header on topic {topic}")
        sys.exit(3)
    first_diff = host_ns[0] - sensor_ns[0]
    run_time_correction = False
    if abs(first_diff) > 100e9:
        run_time_correction = True

    print(f"[INFO] Collected {len(sensor_ns)} messages. Run time correction: {run_time_correction}")

    # -------- PASS 2: run TimestampCorrector --------
    if not run_time_correction:
        corrected_ns = sensor_ns
    else:
        host_ref_ns = host_ns[0] - int(args.ref_offset_sec * 1e9)
        sens_f = [s / 1e9 for s in sensor_ns]                 # seconds
        host_f = [(h - host_ref_ns) / 1e9 for h in host_ns]   # seconds relative to host_ref

        TCor = TC.TimestampCorrector()
        for s, h in zip(sens_f, host_f):
            TCor.correctTimestamp(s, h)

        corrected_ns = []
        for s in sens_f:
            corr_s = float(TCor.getLocalTime(s))              # seconds relative to host_ref
            corrected_ns.append(int(round(host_ref_ns + corr_s * 1e9)))

        # Quick stats
        diffs_ms = [(c - h) / 1e6 for c, h in zip(corrected_ns, host_ns)]
        diffs_sorted = sorted(diffs_ms)
        mid = len(diffs_sorted) // 2
        median = (diffs_sorted[mid] if len(diffs_sorted) % 2 else
                0.5 * (diffs_sorted[mid - 1] + diffs_sorted[mid]))
        mean = sum(diffs_ms) / len(diffs_ms)
        print(f"[STATS] Δ = corrected_host - bag_host (ms): "
            f"mean={mean:.3f}  min={min(diffs_ms):.3f}  max={max(diffs_ms):.3f}  median={median:.3f}")

    # -------- PASS 3: write new bag with corrected times --------
    print(f"[INFO] Writing corrected bag → {out_bag}")
    with rosbag.Bag(str(in_bag), 'r') as src, rosbag.Bag(str(out_bag), 'w') as dst:
        idx = 0
        last_ts_per_topic = {}

        for top, msg, t in src.read_messages():  # copy everything; fix only target topic
            write_t = t

            if top == topic:
                # use corrected host time in order
                if idx >= len(corrected_ns):
                    raise RuntimeError(f"Corrected time list exhausted at index {idx}")
                new_ns = corrected_ns[idx]
                idx += 1

                msg.header.stamp = to_time(new_ns)
                write_t = to_time(new_ns)

            # enforce non-decreasing per topic (rosbag requirement)
            prev = last_ts_per_topic.get(top, None)
            if prev is not None and write_t <= prev:
                # bump by 1 ns to keep monotonicity
                write_t = to_time(int(prev.to_nsec()) + 1)
            last_ts_per_topic[top] = write_t

            dst.write(top, msg, t=write_t)

    accel_csv = str(imu_dir / 'accel.csv')
    gyro_csv  = str(imu_dir / 'gyro.csv')

    append_imu_data(
        accel_csv=accel_csv,
        gyro_csv=gyro_csv,
        out_bag_path=out_bag,
        imu_topic=args.imu_topic,
        frame_id=args.frame_id,
        clamp_out_of_range=not args.no_clamp,
    )

if __name__ == '__main__':
    main()
