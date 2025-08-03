
"""
extract a pointcloud2 message from rosbag and save the xyz components as a pcd file
"""
import argparse
import os
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def extract_pcd(bagfile, topic, starttime, endtime, pcddir, split=1):
    """
    extract a pointcloud2 message from rosbag and save as a pcd file
    """
    bag = rosbag.Bag(bagfile)
    bagstarttime = rospy.Time(bag.get_start_time())
    msgstarttime = starttime + bagstarttime
    msgendtime = endtime + bagstarttime
    import open3d as o3d
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if t >= msgstarttime and t <= msgendtime:
            gen = pc2.read_points(msg, skip_nans=True)
            int_data = list(gen)
            xyz = np.array([[0, 0, 0]])
            for x in int_data:
                xyz = np.append(xyz, [[x[0], x[1], x[2]]], axis=0)
            start = 0
            partsize = xyz.shape[0] // split
            for i in range(1, split + 1):
                if split == 1:
                    output_filename = os.path.join(pcddir, str(t) + ".pcd")
                else:
                    output_filename = os.path.join(pcddir, str(t) + "_{}.pcd".format(i))
                if i == split:
                    end = xyz.shape[0]
                else:
                    end = start + partsize
                out_pcd = o3d.geometry.PointCloud()
                out_pcd.points = o3d.utility.Vector3dVector(xyz[start:end, :])
                start = end
                o3d.io.write_point_cloud(output_filename, out_pcd)
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
