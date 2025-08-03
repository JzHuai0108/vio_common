
import rosbag
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField, Imu
import sensor_msgs.point_cloud2 as pc2
import rospy

# Define VelodynePointXYZIRT fields
point_fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('intensity', 12, PointField.FLOAT32, 1),
    PointField('ring', 16, PointField.UINT16, 1),
    PointField('time', 18, PointField.FLOAT32, 1)
]

def read_pointcloud(msg):
    """Convert PointCloud2 to structured numpy array."""
    pc = []
    for p in pc2.read_points(msg, field_names=["x", "y", "z", "intensity", "ring", "time"], skip_nans=True):
        pc.append(p)
    return np.array(pc, dtype=np.dtype([
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
        ('ring', np.uint16),
        ('time', np.float32)
    ]))

def split_pointcloud(msg, k):
    """Split the point cloud into k segments."""
    cloud_np = read_pointcloud(msg)
    total_points = len(cloud_np)

    segments = []
    for i in range(k):
        start = i * total_points // k
        end = (i + 1) * total_points // k if i != k - 1 else total_points

        segment_np = cloud_np[start:end].copy() # strictly speaking, copy is not needed as our data changes are scoped.
        if len(segment_np) == 0:
            continue
        time_offset = float(segment_np['time'][0])
        segment_np['time'] -= time_offset  # normalize times within segment
        segment_points = [tuple(p) for p in segment_np]

        segment_time = msg.header.stamp.to_sec() + time_offset
        header = msg.header
        header.stamp = rospy.Time.from_sec(segment_time)

        segment_msg = pc2.create_cloud(header, point_fields, segment_points)
        segments.append(segment_msg)

    return segments

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Split Velodyne point clouds in a ROS bag.')
    parser.add_argument('input_bag', type=str, help='Input ROS bag file path')
    parser.add_argument('output_bag', type=str, help='Output ROS bag file path')
    parser.add_argument('--velodyne_topic', type=str, default='/velodyne_points', help='Velodyne point cloud topic')
    parser.add_argument('--imu_topic', type=str, default='/imu/data', help='IMU topic')
    parser.add_argument('--split_k', type=int, default=4, help='Number of segments to split each point cloud into')
    args = parser.parse_args()

    bag_in = rosbag.Bag(args.input_bag, 'r')
    bag_out = rosbag.Bag(args.output_bag, 'w')

    for topic, msg, t in bag_in.read_messages():
        if topic == args.velodyne_topic:
            segments = split_pointcloud(msg, args.split_k)
            for s, seg_msg in enumerate(segments):
                bag_out.write(args.velodyne_topic, seg_msg, seg_msg.header.stamp)
        else:
            bag_out.write(topic, msg, t)

    bag_in.close()
    bag_out.close()
    print(f"[Done] Saved split bag to {args.output_bag}")

if __name__ == '__main__':
    main()
