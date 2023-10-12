
"""
extract a pointcloud2 message from rosbag and save the xyz components as a pcd file
"""
import argparse
import os
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d

def extract_pcd(bagfile, topic, starttime, endtime, pcddir):
    """
    extract a pointcloud2 message from rosbag and save as a pcd file
    """
    bag = rosbag.Bag(bagfile)
    bagstarttime = rospy.Time(bag.get_start_time())
    msgstarttime = starttime + bagstarttime
    msgendtime = endtime + bagstarttime

    for topic, msg, t in bag.read_messages(topics=[topic]):
        if t >= msgstarttime and t <= msgendtime:
            gen = pc2.read_points(msg, skip_nans=True)
            int_data = list(gen)
            xyz = np.array([[0, 0, 0]])
            for x in int_data:
                xyz = np.append(xyz, [[x[0], x[1], x[2]]], axis=0)
            out_pcd = o3d.geometry.PointCloud()
            out_pcd.points = o3d.utility.Vector3dVector(xyz)
            output_filename = os.path.join(pcddir, str(t) + ".pcd")
            o3d.io.write_point_cloud(output_filename, out_pcd)
            print("saved pcd file: {}".format(output_filename))



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bagfile', type=str, help='rosbag file')
    parser.add_argument('--topic', type=str, help='topic name', default='/velodyne_points')
    parser.add_argument('--starttime', type=float, help='start time, (default: %(default)s)', default=40.0)
    parser.add_argument('--endtime', type=float, help='end time, (default: %(default)s)', default=42.0)
    parser.add_argument('--pcddir', type=str, help='pcd file directory')
    args = parser.parse_args()
    extract_pcd(args.bagfile, args.topic, rospy.Duration(args.starttime), rospy.Duration(args.endtime), args.pcddir)
