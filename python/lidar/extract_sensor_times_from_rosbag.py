#!/usr/bin/env python3
"""Extract ROS1 bag-record times and message header stamps.

The bag is traversed once for all requested streams.  Each output row keeps
the ROS sec/nsec components separately, avoiding the precision loss caused by
representing epoch timestamps only as floating-point seconds.
"""

import argparse
import math
import os
import struct
from pathlib import Path
from typing import Dict, Mapping, TextIO, Tuple

import rosbag
import numpy as np


DEFAULT_STREAMS = {
    "lidar": "/rslidar_front_points",
    "imu": "/rslidar_front_imu_data",
    "camera_left": "/front_seeker/fisheye/left/image_raw/compressed",
    "camera_right": "/front_seeker/fisheye/right/image_raw/compressed",
}


def time_to_ns(secs: int, nsecs: int) -> int:
    """Convert separate ROS time fields to integer nanoseconds."""
    return int(secs) * 1_000_000_000 + int(nsecs)


def format_stamp(secs: int, nsecs: int) -> str:
    """Format a ROS time exactly, including nine fractional digits."""
    return f"{int(secs)}.{int(nsecs):09d}"


def raw_header_stamp(raw_message) -> Tuple[int, int, int]:
    """Read ``seq, secs, nsecs`` from a serialized leading ROS1 Header.

    ``raw=True`` avoids deserializing multi-megabyte PointCloud2 and image
    payloads. ROS1 serializes message fields in declaration order, and all
    supported sensor_msgs types declare ``Header header`` first.
    """
    if len(raw_message) < 5:
        raise TypeError("Unexpected rosbag raw-message tuple")
    serialized = raw_message[1]
    message_class = raw_message[4]
    slots = getattr(message_class, "__slots__", ())
    if not slots or slots[0] != "header":
        raise TypeError(
            f"{getattr(message_class, '_type', message_class)} does not have "
            "a leading ROS Header"
        )
    if len(serialized) < 12:
        raise ValueError("Serialized message is too short to contain a Header")
    return struct.unpack_from("<III", serialized, 0)


def _read_u32(serialized: bytes, offset: int) -> Tuple[int, int]:
    if offset + 4 > len(serialized):
        raise ValueError("Truncated serialized PointCloud2 metadata")
    return struct.unpack_from("<I", serialized, offset)[0], offset + 4


def _read_string(serialized: bytes, offset: int) -> Tuple[str, int]:
    length, offset = _read_u32(serialized, offset)
    end = offset + length
    if end > len(serialized):
        raise ValueError("Truncated serialized PointCloud2 string")
    return serialized[offset:end].decode("utf-8"), end


def pointcloud2_time_bounds(raw_message) -> Tuple[float, float]:
    """Return min/max of the per-point ``timestamp`` field.

    The Airy96 layout is an organized 860x96 PointCloud2 with 27-byte points;
    its timestamp is a FLOAT64 absolute time at byte offset 18.  Metadata is
    still parsed on every scan so an unexpected layout fails explicitly
    instead of silently reading the wrong bytes.
    """
    serialized = raw_message[1]
    offset = 12  # uint32 seq + time stamp (sec,nsec)
    _, offset = _read_string(serialized, offset)  # header.frame_id
    if offset + 8 > len(serialized):
        raise ValueError("Truncated serialized PointCloud2 dimensions")
    height, width = struct.unpack_from("<II", serialized, offset)
    offset += 8
    field_count, offset = _read_u32(serialized, offset)
    timestamp_field = None
    for _ in range(field_count):
        name, offset = _read_string(serialized, offset)
        if offset + 9 > len(serialized):
            raise ValueError("Truncated serialized PointField")
        field_offset = struct.unpack_from("<I", serialized, offset)[0]
        datatype = serialized[offset + 4]
        count = struct.unpack_from("<I", serialized, offset + 5)[0]
        offset += 9
        if name == "timestamp":
            timestamp_field = (field_offset, datatype, count)

    if timestamp_field is None:
        raise ValueError("PointCloud2 has no 'timestamp' point field")
    field_offset, datatype, count = timestamp_field
    # sensor_msgs/PointField.FLOAT64 == 8.
    if datatype != 8 or count != 1:
        raise ValueError(
            "Expected scalar FLOAT64 PointCloud2 timestamp, got "
            f"datatype={datatype}, count={count}"
        )
    if offset + 13 > len(serialized):
        raise ValueError("Truncated serialized PointCloud2 layout")
    is_bigendian = bool(serialized[offset])
    point_step, row_step, data_length = struct.unpack_from(
        "<III", serialized, offset + 1
    )
    data_offset = offset + 13
    if data_offset + data_length > len(serialized):
        raise ValueError("Truncated serialized PointCloud2 data")
    if width == 0 or height == 0:
        raise ValueError("PointCloud2 scan contains no points")
    if field_offset + 8 > point_step or row_step < width * point_step:
        raise ValueError("Invalid PointCloud2 timestamp offset or row stride")
    if data_length < row_step * height:
        raise ValueError("PointCloud2 data is shorter than height*row_step")

    byte_order = ">" if is_bigendian else "<"
    timestamps = np.ndarray(
        shape=(height, width),
        dtype=np.dtype(byte_order + "f8"),
        buffer=serialized,
        offset=data_offset + field_offset,
        strides=(row_step, point_step),
    )
    finite = timestamps[np.isfinite(timestamps)]
    if finite.size == 0:
        raise ValueError("PointCloud2 timestamp field has no finite values")
    return float(finite.min()), float(finite.max())


def seconds_float_to_parts(timestamp: float) -> Tuple[int, int]:
    """Round a finite FLOAT64 seconds timestamp to ROS sec/nsec fields."""
    if not math.isfinite(timestamp):
        raise ValueError("Non-finite point timestamp")
    secs = math.floor(timestamp)
    nsecs = int(round((timestamp - secs) * 1_000_000_000))
    if nsecs >= 1_000_000_000:
        secs += 1
        nsecs -= 1_000_000_000
    return int(secs), nsecs


def extract_sensor_times(
    bag_path: Path, output_dir: Path, streams: Mapping[str, str]
) -> Dict[str, int]:
    """Extract timestamps for every named topic and return message counts."""
    bag_path = bag_path.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()
    if not bag_path.is_file():
        raise FileNotFoundError(f"ROS bag does not exist: {bag_path}")
    if not streams:
        raise ValueError("At least one stream must be requested")
    if len(set(streams.values())) != len(streams):
        raise ValueError("Each output stream must refer to a distinct topic")

    output_dir.mkdir(parents=True, exist_ok=True)
    counts = {name: 0 for name in streams}
    topic_to_name = {topic: name for name, topic in streams.items()}
    final_paths = {
        name: output_dir / f"{name}_times.txt" for name in streams
    }
    temporary_paths = {
        name: output_dir / f".{name}_times.txt.tmp" for name in streams
    }
    handles: Dict[str, TextIO] = {}

    try:
        with rosbag.Bag(str(bag_path), "r") as bag:
            topic_info = bag.get_type_and_topic_info().topics
            missing = [topic for topic in streams.values() if topic not in topic_info]
            if missing:
                raise ValueError(
                    "Requested topic(s) missing from bag: " + ", ".join(missing)
                )

            for name, topic in streams.items():
                handle = temporary_paths[name].open("w", encoding="utf-8")
                handles[name] = handle
                handle.write(f"# bag: {bag_path}\n")
                handle.write(f"# topic: {topic}\n")
                handle.write(f"# type: {topic_info[topic].msg_type}\n")
                if topic_info[topic].msg_type == "sensor_msgs/PointCloud2":
                    handle.write(
                        "# columns: index header_seq header_stamp bag_time "
                        "min_point_time max_point_time "
                        "header_stamp_sec header_stamp_nsec "
                        "bag_time_sec bag_time_nsec "
                        "min_point_time_sec min_point_time_nsec "
                        "max_point_time_sec max_point_time_nsec "
                        "point_time_span_ns bag_minus_header_ns\n"
                    )
                else:
                    handle.write(
                        "# columns: index header_seq header_stamp bag_time "
                        "header_stamp_sec header_stamp_nsec "
                        "bag_time_sec bag_time_nsec bag_minus_header_ns\n"
                    )

            for topic, raw_message, bag_time in bag.read_messages(
                topics=list(topic_to_name), raw=True
            ):
                name = topic_to_name[topic]
                header_seq, header_secs, header_nsecs = raw_header_stamp(
                    raw_message
                )
                bag_secs = int(bag_time.secs)
                bag_nsecs = int(bag_time.nsecs)
                delta_ns = time_to_ns(bag_secs, bag_nsecs) - time_to_ns(
                    header_secs, header_nsecs
                )
                if topic_info[topic].msg_type == "sensor_msgs/PointCloud2":
                    min_point_time, max_point_time = pointcloud2_time_bounds(
                        raw_message
                    )
                    min_secs, min_nsecs = seconds_float_to_parts(min_point_time)
                    max_secs, max_nsecs = seconds_float_to_parts(max_point_time)
                    point_span_ns = time_to_ns(max_secs, max_nsecs) - time_to_ns(
                        min_secs, min_nsecs
                    )
                    handles[name].write(
                        f"{counts[name]} {header_seq} "
                        f"{format_stamp(header_secs, header_nsecs)} "
                        f"{format_stamp(bag_secs, bag_nsecs)} "
                        f"{format_stamp(min_secs, min_nsecs)} "
                        f"{format_stamp(max_secs, max_nsecs)} "
                        f"{header_secs} {header_nsecs} "
                        f"{bag_secs} {bag_nsecs} "
                        f"{min_secs} {min_nsecs} {max_secs} {max_nsecs} "
                        f"{point_span_ns} {delta_ns}\n"
                    )
                else:
                    handles[name].write(
                        f"{counts[name]} {header_seq} "
                        f"{format_stamp(header_secs, header_nsecs)} "
                        f"{format_stamp(bag_secs, bag_nsecs)} "
                        f"{header_secs} {header_nsecs} "
                        f"{bag_secs} {bag_nsecs} "
                        f"{delta_ns}\n"
                    )
                counts[name] += 1

        for handle in handles.values():
            handle.close()
        handles.clear()
        for name in streams:
            os.replace(temporary_paths[name], final_paths[name])
    except BaseException:
        for handle in handles.values():
            handle.close()
        for path in temporary_paths.values():
            try:
                path.unlink()
            except FileNotFoundError:
                pass
        raise

    for name, topic in streams.items():
        print(
            f"Wrote {counts[name]} timestamps from {topic} to "
            f"{final_paths[name]}"
        )
    return counts


def parse_stream(value: str):
    """Parse NAME=TOPIC for repeatable custom stream arguments."""
    if "=" not in value:
        raise argparse.ArgumentTypeError("stream must have the form NAME=TOPIC")
    name, topic = value.split("=", 1)
    if not name or not topic or not topic.startswith("/"):
        raise argparse.ArgumentTypeError(
            "stream must have a non-empty name and an absolute ROS topic"
        )
    return name, topic


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Extract exact header and ROS1 bag-record timestamps."
    )
    parser.add_argument("bagfile", type=Path, help="input ROS1 bag")
    parser.add_argument("output_dir", type=Path, help="output directory")
    parser.add_argument(
        "--stream",
        action="append",
        type=parse_stream,
        metavar="NAME=TOPIC",
        help=(
            "custom named topic; repeat for multiple streams. If omitted, "
            "the WHU Airy96 LiDAR, IMU, and two fisheye topics are used"
        ),
    )
    args = parser.parse_args()

    streams = dict(args.stream) if args.stream else DEFAULT_STREAMS
    if len(streams) != len(args.stream or streams):
        parser.error("stream names must be unique")
    extract_sensor_times(args.bagfile, args.output_dir, streams)


if __name__ == "__main__":
    main()
