
"""
extract a pointcloud2 message from rosbag and save the xyz components as a pcd file
"""
import argparse
import os
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def write_xyzi_pcd(output_filename, xyzi):
    num_points = xyzi.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {num_points}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {num_points}\n"
        "DATA binary\n"
    )
    with open(output_filename, "wb") as stream:
        stream.write(header.encode("ascii"))
        np.ascontiguousarray(xyzi, dtype=np.float32).tofile(stream)

def extract_pcd(bagfile, topic, starttime, endtime, pcddir, split=1):
    """
    extract a pointcloud2 message from rosbag and save as a pcd file
    """
    bag = rosbag.Bag(bagfile)
    bagstarttime = rospy.Time(bag.get_start_time())
    msgstarttime = starttime + bagstarttime
    msgendtime = endtime + bagstarttime
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if t >= msgstarttime and t <= msgendtime:
            msg_fields = {f.name for f in msg.fields}
            if "intensity" in msg_fields:
                point_iter = pc2.read_points(
                    msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
                )
                xyzi = np.asarray(list(point_iter), dtype=np.float32)
            else:
                point_iter = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
                xyz = np.asarray(list(point_iter), dtype=np.float32)
                xyzi = np.zeros((xyz.shape[0], 4), dtype=np.float32)
                if xyz.size > 0:
                    xyzi[:, :3] = xyz

            if xyzi.size == 0:
                continue
            if xyzi.ndim == 1:
                xyzi = xyzi.reshape(1, -1)

            start = 0
            partsize = xyzi.shape[0] // split
            for i in range(1, split + 1):
                if split == 1:
                    output_filename = os.path.join(pcddir, "%d.%09d.pcd" % (t.secs, t.nsecs))
                else:
                    output_filename = os.path.join(pcddir, "%d.%09d_%d.pcd" % (t.secs, t.nsecs, i))
                if i == split:
                    end = xyzi.shape[0]
                else:
                    end = start + partsize
                write_xyzi_pcd(output_filename, xyzi[start:end, :])
                start = end
                print("saved pcd file: {}".format(output_filename))


def extract_pcd_times(bagfile, topic, starttime, endtime, outfile):
    """
    extract a pointcloud2 message from rosbag and save as a pcd file
    """
    bag = rosbag.Bag(bagfile)
    bagstarttime = rospy.Time(bag.get_start_time())
    msgstarttime = starttime + bagstarttime
    msgendtime = endtime + bagstarttime

    stream = open(outfile, 'w')
    stream.write('#Sensor time, host time\n')
    count = 0
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if t >= msgstarttime and t <= msgendtime:
            stream.write('%d.%09d %d.%09d\n' %
                    (msg.header.stamp.secs, msg.header.stamp.nsecs, t.secs, t.nsecs))
            count += 1
    stream.close()
    print('Wrote {} times to {}'.format(count, outfile))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bagfile', type=str, help='rosbag file')
    parser.add_argument('--topic', type=str, help='topic name', default='/velodyne_points')
    parser.add_argument('--starttime', type=float, help='start time, (default: %(default)s)', default=0.0)
    parser.add_argument('--endtime', type=float, help='end time, (default: %(default)s)', default=1e8)
    parser.add_argument('--pcddir', type=str, help='pcd file directory')
    parser.add_argument('--split', type=int, help='split the pcd files into k chunks', default=1)
    args = parser.parse_args()
    extract_pcd(args.bagfile, args.topic, rospy.Duration(args.starttime), rospy.Duration(args.endtime), args.pcddir, args.split)

    outfile = args.bagfile.replace('.bag', args.topic.replace('/', '_') + '.txt')
    extract_pcd_times(args.bagfile, args.topic, rospy.Duration(args.starttime), rospy.Duration(args.endtime), outfile)
