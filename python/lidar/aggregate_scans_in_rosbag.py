
import os
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d


def save_pcd_xyz_intensity(filename, points, intensities):
    """
    Save a PCD file with XYZ and intensity fields.
    
    :param filename: Output PCD filename.
    :param points: Nx3 NumPy array of XYZ coordinates.
    :param intensities: N-length NumPy array of intensity values.
    """
    num_points = points.shape[0]
    
    with open(filename, 'w') as f:
        # Write PCD header
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity\n")
        f.write("SIZE 4 4 4 4\n")  # 4 bytes per float (float32)
        f.write("TYPE F F F F\n")  # All are floats
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {num_points}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {num_points}\n")
        f.write("DATA ascii\n")
        
        # Write point data
        for i in range(num_points):
            f.write(f"{points[i,0]} {points[i,1]} {points[i,2]} {intensities[i]}\n")


def aggregate_static_scans(bagname, outpcdfile, starttime=0, endtime=10000, topic="/livox/lidar"):
    """
    Aggregates PointCloud2 messages from a ROS bag and saves the points to a PCD file.
    
    Args:
        bagname (str): Path to the ROS bag file.
        outpcdfile (str): Path to the output PCD file.
        starttime (float): Start time (relative to the start of the bag) in seconds.
        endtime (float): End time (relative to the start of the bag) in seconds.
        topic (str): Topic name to extract PointCloud2 messages from.
    """
    all_points = []
    all_intensities = []

    with rosbag.Bag(bagname, 'r') as bag:
        bag_start_time = bag.get_start_time()
        start_time_absolute = bag_start_time + starttime
        end_time_absolute = bag_start_time + endtime

        # Iterate through the messages in the specified time range and topic
        for topic_name, msg, t in bag.read_messages(topics=[topic], 
                                                    start_time=rospy.Time.from_sec(start_time_absolute), 
                                                    end_time=rospy.Time.from_sec(end_time_absolute)):
            # Parse the PointCloud2 message
            for point in pc2.read_points(msg, field_names=["x", "y", "z", "intensity", "tag", "line", "timestamp"], skip_nans=True):
                all_points.append(point)
                all_intensities.append(point[3])

    # Convert the collected points into a numpy array
    points_np = np.array(all_points, dtype=np.float32)
    intensities_np = np.array(all_intensities, dtype=np.float32)
    max_intensity = np.max(intensities_np)
    if max_intensity > 255.0:
        print(f'Warn: max_intensity {max_intensity} > 255.0 and may cause wrong color in open3d saved pcd.')
    normalized_intensities = intensities_np / 255.0

    # Map normalized intensities to grayscale (R=G=B)
    colors_np = np.stack([normalized_intensities] * 3, axis=-1)

    # Create an Open3D point cloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_np[:, :3])
    pcd.colors = o3d.utility.Vector3dVector(colors_np)

    save_pcd_xyz_intensity(outpcdfile, points_np, intensities_np)
    # Save the point cloud to a PCD file
    colorpcdfile = os.path.join(os.path.dirname(outpcdfile), os.path.splitext(os.path.basename(outpcdfile))[0] + '_rgb.pcd')
    o3d.io.write_point_cloud(colorpcdfile, pcd)
    print(f"Saved aggregated point cloud to {outpcdfile}")
