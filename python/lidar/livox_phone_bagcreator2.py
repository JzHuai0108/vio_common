#!/usr/bin/env python3
"""Merge LidarDroid phone video/IMU and Mid360 on the shared phone clock.

Requires ROS 1 Python (rosbag, rospy), OpenCV for kalibr_bagcreator.py, and
NumPy. Camera and phone IMU stamps must use the same sensor clock. LiDAR bag
receipt times and phone CSV receipt times must share a Unix clock; fixed
receipt latency remains.
"""

import argparse
import csv
import math
import os
from pathlib import Path
import subprocess
import sys

import numpy as np


def timestamp_corrector():
    """Import the project's built pybind11 implementation."""
    build = os.environ.get(
        "TIMESTAMP_CORRECTOR_BUILD",
        str(Path(__file__).resolve().parents[2] / "timestamp_corrector" / "build"),
    )
    if build not in sys.path:
        sys.path.insert(0, build)
    try:
        import TimestampCorrector
    except ImportError as exc:
        raise RuntimeError(
            "Build vio_common/timestamp_corrector for this Python interpreter "
            "or set TIMESTAMP_CORRECTOR_BUILD to its build directory"
        ) from exc
    return TimestampCorrector


class ClockFit:
    """Affine midpoint fit from the project's TimestampCorrector."""

    def __init__(self, pairs):
        pairs = sorted(pairs)
        if len(pairs) < 2:
            raise ValueError("Need at least two timestamp pairs")
        self.remote_zero, self.local_zero = pairs[0]
        self.corrector = timestamp_corrector().TimestampCorrector()
        last_remote = None
        for remote, local in pairs:
            if remote == last_remote:
                continue
            self.corrector.correctTimestamp(
                remote - self.remote_zero, local - self.local_zero
            )
            last_remote = remote
        self.rate = self.corrector.getSlope()
        if not math.isfinite(self.rate) or not 0.99 < self.rate < 1.01:
            raise ValueError(f"Implausible clock rate: {self.rate}")

    def get_local_time(self, remote):
        return self.local_zero + self.corrector.getLocalTime(
            remote - self.remote_zero
        )

    def get_remote_time(self, local):
        # TimestampCorrector has no inverse API; invert its midpoint line.
        return self.remote_zero + (
            local - self.local_zero - self.corrector.getOffset()
        ) / self.rate

def phone_pairs(imu_csv):
    with open(imu_csv, newline="") as stream:
        for row in csv.reader(stream):
            if row and row[0] and row[0][0].isdigit():
                yield float(row[0]), float(row[-1])


def check_camera_clock(frame_csv, phone_fit):
    """Reject camera stamps that do not share the phone IMU sensor clock."""
    first = last = None
    with frame_csv.open(newline="") as stream:
        for row in csv.reader(stream):
            if row and row[0] and row[0][0].isdigit():
                last = float(row[0]), float(row[-1])
                if first is None:
                    first = last
    if first is None:
        raise ValueError(f"No frame timestamp pairs in {frame_csv}")
    residuals = [host - phone_fit.get_local_time(sensor) for sensor, host in (first, last)]
    if any(abs(residual) > 1.0 for residual in residuals):
        raise ValueError(
            "Camera and phone IMU stamps do not share a clock; "
            f"frame receipt residuals: {residuals}"
        )
    return residuals

def seconds(stamp):
    return stamp.secs + stamp.nsecs * 1e-9


def fit_lidar_clock(bag_path):
    import rosbag

    pairs = []
    with rosbag.Bag(str(bag_path)) as bag:
        for _, msg, received in bag.read_messages(topics=["/livox/lidar", "/livox/imu"]):
            pairs.append((seconds(msg.header.stamp), seconds(received)))
    return ClockFit(pairs), len(pairs)


def scale_point_offsets(cloud, rate):
    """Livox PointCloud2 timestamp is a FLOAT64 nanosecond offset from header."""
    field = next((f for f in cloud.fields if f.name == "timestamp"), None)
    if field is None or field.datatype != 8:  # sensor_msgs/PointField.FLOAT64
        raise ValueError("Expected Livox FLOAT64 timestamp point field")
    data = bytearray(cloud.data)
    offsets = np.ndarray((cloud.height, cloud.width),
                         dtype=">f8" if cloud.is_bigendian else "<f8",
                         buffer=data, offset=field.offset,
                         strides=(cloud.row_step, cloud.point_step))
    if offsets.size and (np.nanmin(offsets) < 0 or np.nanmax(offsets) > 1e9):
        raise ValueError("Expected point timestamps as offsets in nanoseconds")
    offsets *= rate
    cloud.data = bytes(data)


def convert_trajectory(source, destination, to_phone):
    if not source.is_file():
        return
    with source.open() as inp, destination.open("x") as out:
        for line in inp:
            if not line.strip() or line.lstrip().startswith("#"):
                out.write(line)
                continue
            stamp, rest = line.split(maxsplit=1)
            out.write(f"{to_phone(float(stamp)):.9f} {rest}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sequence_dir", type=Path)
    parser.add_argument("--output", type=Path,
                        help="Default: sequence_dir/movie_phoneclock.bag")
    parser.add_argument("--fit-only", action="store_true",
                        help="Print clock fits without writing outputs")
    args = parser.parse_args()
    seq = args.sequence_dir.resolve()
    phone_times = list(phone_pairs(seq / "gyro_accel.csv"))
    if not phone_times:
        parser.error("No phone IMU timestamp pairs")
    phone = ClockFit(phone_times)  # phone sensor -> Unix
    frame_residuals = check_camera_clock(seq / "frame_timestamps.txt", phone)

    lidar_bag = seq / "lidar.bag"
    if not lidar_bag.is_file():
        lidar_bag = seq / "mid360.bag"
    if not lidar_bag.is_file():
        parser.error("Expected lidar.bag or mid360.bag")

    lidar, count = fit_lidar_clock(lidar_bag)  # Livox sensor -> Unix

    def to_phone(sensor_time):
        return phone.get_remote_time(lidar.get_local_time(sensor_time))

    rate = lidar.rate / phone.rate
    print(f"Camera/IMU receipt residuals: {frame_residuals[0]:.6f}, {frame_residuals[1]:.6f} s")
    print(f"Phone sensor -> Unix rate: {phone.rate:.9f}")
    print(f"Livox sensor -> Unix rate: {lidar.rate:.9f} ({count} pairs)")
    print(f"Livox -> phone clock rate: {rate:.9f}")
    if args.fit_only:
        return

    output = args.output or seq / "movie_phoneclock.bag"
    trajectory = seq / "faster_lio_traj_phoneclock.txt"
    if output.exists() or trajectory.exists():
        parser.error(f"Output exists: {output if output.exists() else trajectory}")

    # No --sync_to_unix: retain the shared camera/phone-IMU clock.
    base = Path(__file__).resolve().parents[1]
    subprocess.run([sys.executable, str(base / "kalibr_bagcreator.py"),
                    "--video", str(seq / "movie.mp4"),
                    "--imu", str(seq / "gyro_accel.csv"),
                    "--video_time_file", str(seq / "frame_timestamps.txt"),
                    "--no_preview", "--output_bag", str(output)], check=True)

    import rosbag
    import rospy

    counts = {"/livox/lidar": 0, "/livox/imu": 0}
    with rosbag.Bag(str(lidar_bag)) as source, rosbag.Bag(str(output), "a") as target:
        for topic, msg, _ in source.read_messages(topics=list(counts)):
            phone_time = rospy.Time.from_sec(to_phone(seconds(msg.header.stamp)))
            if topic == "/livox/lidar":
                scale_point_offsets(msg, rate)
            else:
                # Livox IMU reports acceleration in g; ROS Imu expects m/s^2.
                for axis in ("x", "y", "z"):
                    setattr(msg.linear_acceleration, axis,
                            getattr(msg.linear_acceleration, axis) * 9.805)
            msg.header.stamp = phone_time
            target.write(topic, msg, phone_time)
            counts[topic] += 1
    if min(counts.values()) == 0:
        raise ValueError(f"Missing Livox topic: {counts}")
    convert_trajectory(seq / "faster_lio_traj.txt", trajectory, to_phone)
    print(f"Wrote {counts} to {output}")


if __name__ == "__main__":
    main()
