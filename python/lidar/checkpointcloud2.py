#!/usr/bin/env python3
import sys
import rosbag

# PointCloud2 reader (only used when msg is actually PointCloud2)
import sensor_msgs.point_cloud2 as pc2

USAGE = (
    f"Usage:\n"
    f"  {sys.argv[0]} <bagname> <topic> [sensor(velodyne|hesai|hesai_sync|ouster|points_raw|livox)] [time_unit(ns|us|ms|s)]\n"
    f"\n"
    f"Notes:\n"
    f"  - For PointCloud2: sensor selects field schema (like your original script).\n"
    f"  - For Livox CustomMsg: time_unit controls how timebase/offset_time are interpreted (default: ns).\n"
)

# Map sensor -> PointCloud2 field tuple
SCHEMAS = {
    "velodyne":   dict(fields=("x", "y", "z", "intensity", "ring", "time"), time_mode="relative"),
    "hesai":      dict(fields=("x", "y", "z", "intensity", "timestamp", "ring"), time_mode="absolute"),
    "hesai_sync": dict(fields=("x", "y", "z", "intensity", "timestamp", "timestamp_sync", "ring"), time_mode="absolute"),
    "ouster":     dict(fields=("x", "y", "z", "intensity", "t", "reflectivity", "ring", "noise", "range")),
    "points_raw": dict(fields=("x", "y", "z", "time", "ring")),
    # "livox" is handled by a dedicated parser (not PointCloud2)
    "livox":      dict(fields=()),  # placeholder
}

def infer_sensor_from_topic(topic: str) -> str:
    t = topic.lower()
    if "livox" in t: return "livox"
    if "velodyne" in t: return "velodyne"
    if "hesai" in t: return "hesai"
    if "os1" in t or "ouster" in t: return "ouster"
    return "points_raw"

def is_livox_custommsg(msg) -> bool:
    # Livox CustomMsg should have these
    return hasattr(msg, "timebase") and hasattr(msg, "point_num") and hasattr(msg, "points")

def is_pointcloud2(msg) -> bool:
    # sensor_msgs/PointCloud2 has .fields and typically .data
    return hasattr(msg, "fields") and hasattr(msg, "data")

def time_scale_from_unit(unit: str) -> float:
    u = (unit or "ns").lower()
    if u in ("ns", "nsec", "nanosecond", "nanoseconds"):
        return 1e-9
    if u in ("us", "usec", "microsecond", "microseconds"):
        return 1e-6
    if u in ("ms", "msec", "millisecond", "milliseconds"):
        return 1e-3
    if u in ("s", "sec", "second", "seconds"):
        return 1.0
    raise ValueError(f"Unknown time_unit '{unit}', use: ns|us|ms|s")

def format_field(name, val):
    if isinstance(val, float):
        return f"{name}: {val:.9f}" if ("time" in name or "timestamp" in name) else f"{name}: {val:.6f}"
    return f"{name}: {val}"

def read_and_print_points_pointcloud2(msg, fields, sample_every=100, head=5, tail=5):
    """Pretty-printer for PointCloud2."""
    # pass 1: count (generator is one-pass)
    count = 0
    for _ in pc2.read_points(msg, field_names=fields, skip_nans=True):
        count += 1

    # pass 2: print sampled rows
    gen = pc2.read_points(msg, field_names=fields, skip_nans=True)
    for j, p in enumerate(gen):
        if (j % sample_every == 0) or (j < head) or (j >= count - tail):
            kv = "  ".join(format_field(name, p[idx]) for idx, name in enumerate(fields))
            print(f"{j}, msg time: {msg.header.stamp.secs}.{msg.header.stamp.nsecs:09d}, {kv}")

def read_and_print_points_livox_custommsg(msg, time_unit="ns", sample_every=100, head=5, tail=5):
    """
    Pretty-printer for livox_ros_driver2/CustomMsg.

    Computes per-point time:
      t_point_sec = (timebase + offset_time) * scale
    where scale depends on time_unit (default ns).
    """
    scale = time_scale_from_unit(time_unit)

    pts = msg.points
    count = len(pts)
    # msg.point_num exists, but tolerate mismatch
    point_num = int(getattr(msg, "point_num", count))

    header_stamp = msg.header.stamp
    timebase = int(msg.timebase)
    lidar_id = int(msg.lidar_id) if hasattr(msg, "lidar_id") else -1

    print(f"[LIVOX] lidar_id={lidar_id}, point_num(field)={point_num}, points(len)={count}, timebase={timebase} ({time_unit})")
    print(f"[LIVOX] header.stamp={header_stamp.secs}.{header_stamp.nsecs:09d}")

    def should_print(j):
        return (j % sample_every == 0) or (j < head) or (j >= count - tail)

    for j, p in enumerate(pts):
        if not should_print(j):
            continue
        # p has: offset_time, x, y, z, reflectivity, tag, line
        ot_ns = int(p.offset_time)
        t_ns = timebase + ot_ns               # exact int ns
        t_sec = t_ns // 1_000_000_000
        t_nsec = t_ns % 1_000_000_000

        print(
            f"{j}, msg time: {header_stamp.secs}.{header_stamp.nsecs:09d}, "
            f"x: {float(p.x):.6f}  y: {float(p.y):.6f}  z: {float(p.z):.6f}  "
            f"reflectivity: {int(p.reflectivity)}  tag: {int(p.tag)}  line: {int(p.line)}  "
            f"offset_time: {ot_ns}  "
            f"t_point: {t_sec}.{t_nsec:09d})"
        )

def main():
    if len(sys.argv) < 3:
        print(USAGE)
        sys.exit(1)

    bagname = sys.argv[1]
    topic = sys.argv[2]
    sensor = sys.argv[3].lower() if len(sys.argv) >= 4 else infer_sensor_from_topic(topic)
    time_unit = sys.argv[4].lower() if len(sys.argv) >= 5 else "ns"

    if sensor not in SCHEMAS:
        print(f"[WARN] Unknown sensor '{sensor}'. Known: {list(SCHEMAS)}. Falling back to inferred.")
        sensor = infer_sensor_from_topic(topic)

    print(f"[INFO] bag='{bagname}', topic='{topic}', sensor_hint='{sensor}', time_unit='{time_unit}'")

    i = 0
    with rosbag.Bag(bagname) as bag:
        for topic_name, msg, t in bag.read_messages(topics=[topic]):
            print(f"\nFrame {i}, ros time: {t}")

            # Auto-detect actual message type
            if is_livox_custommsg(msg):
                # Ignore PointCloud2 schemas; parse Livox
                if i == 0:
                    # Print “shape” once
                    print("[INFO] Detected Livox CustomMsg on this topic.")
                    # show available attributes (small hint)
                    print(f"[INFO] Msg has: header,timebase,point_num,lidar_id,points (and maybe rsvd)")
                read_and_print_points_livox_custommsg(msg, time_unit=time_unit, sample_every=100, head=5, tail=5)

            elif is_pointcloud2(msg):
                # Use your original PointCloud2 logic
                if sensor == "livox":
                    # user forced livox but msg is PointCloud2; fallback
                    sensor2 = infer_sensor_from_topic(topic)
                    if sensor2 == "livox":
                        sensor2 = "points_raw"
                    print(f"[WARN] sensor_hint='livox' but message is PointCloud2. Using '{sensor2}'.")
                    sensor_use = sensor2
                else:
                    sensor_use = sensor

                fields = SCHEMAS[sensor_use]["fields"]
                if i == 0:
                    print(f"[INFO] Detected PointCloud2 on this topic.")
                    print(f"PointCloud2 fields in message: {msg.fields}")
                    print(f"[INFO] Using sensor='{sensor_use}', fields={fields}")
                print(f"header: {msg.header.stamp.secs}.{msg.header.stamp.nsecs:09d}  frame_id='{msg.header.frame_id}'")
                read_and_print_points_pointcloud2(msg, fields, sample_every=100, head=5, tail=5)

            else:
                # Unknown type
                if i == 0:
                    print("[ERROR] Unknown message type (neither PointCloud2 nor Livox CustomMsg).")
                    print(f"[HINT] msg.__class__={msg.__class__}")
                break

            i += 1
            if i > 5:  # limit for demo; remove to process all
                break

if __name__ == "__main__":
    main()
