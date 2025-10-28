#!/usr/bin/env python3
import sys
import rosbag
import sensor_msgs.point_cloud2 as pc2

USAGE = f"Usage: {sys.argv[0]} <bagname> <topic> [sensor(velodyne|hesai|hesai_sync|ouster|points_raw)]"

# Map sensor -> PointCloud2 field tuple
SCHEMAS = {
    "velodyne":   dict(fields=("x", "y", "z", "intensity", "ring", "time"),          time_mode="relative"),
    "hesai":      dict(fields=("x", "y", "z", "intensity", "timestamp", "ring"),     time_mode="absolute"),
    "hesai_sync":      dict(fields=("x", "y", "z", "intensity", "timestamp", "timestamp_sync", "ring"),     time_mode="absolute"),
    "ouster":     dict(fields=("x", "y", "z", "intensity", "t", "reflectivity", "ring", "noise", "range")),
    "points_raw": dict(fields=("x", "y", "z", "time", "ring")),  # some stacks publish like this
}

def infer_sensor(topic: str) -> str:
    t = topic.lower()
    if "velodyne" in t: return "velodyne"
    if "hesai" in t:    return "hesai"
    if "os1" in t or "ouster" in t: return "ouster"
    return "points_raw"  # fallback guess

def format_field(name, val):
    if isinstance(val, float):
        # show higher precision for timestamps
        return f"{name}: {val:.9f}" if "time" in name or "timestamp" in name else f"{name}: {val:.6f}"
    return f"{name}: {val}"

def read_and_print_points(msg, fields, sample_every=100, head=5, tail=5):
    """Single generic reader/pretty-printer for any schema."""
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

def main():
    if len(sys.argv) < 3:
        print(USAGE); sys.exit(1)

    bagname = sys.argv[1]
    topic = sys.argv[2]
    sensor = sys.argv[3].lower() if len(sys.argv) >= 4 else infer_sensor(topic)

    if sensor not in SCHEMAS:
        print(f"[WARN] Unknown sensor '{sensor}'. Known: {list(SCHEMAS)}. Falling back to 'points_raw'.")
        sensor = "points_raw"

    fields = SCHEMAS[sensor]["fields"]
    print(f"[INFO] Using sensor='{sensor}', fields={fields}")

    i = 0
    with rosbag.Bag(bagname) as bag:
        for topic_name, msg, t in bag.read_messages(topics=[topic]):
            if i == 0:
                print(f"PointCloud2 fields in message: {msg.fields}")
            print(f"\nFrame {i}, header: {msg.header.stamp}, ros time: {t}")

            read_and_print_points(msg, fields, sample_every=100, head=5, tail=5)

            i += 1
            if i > 5:  # limit for demo; remove to process all
                break

if __name__ == "__main__":
    main()
