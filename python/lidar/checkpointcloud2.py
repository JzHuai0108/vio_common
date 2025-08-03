import sensor_msgs.point_cloud2 as pc2

import rosbag

import sys

if len(sys.argv) < 3:
    print('Usage: {} <bagname> <topic> [sensor(velodyne, hesai, ouster)]'.format(sys.argv[0]))
    exit(1)

# pcl datatypes
bagname = sys.argv[1]
pointcloudtopic = sys.argv[2]
sensor = ''
if len(sys.argv) >= 4:
    sensor = sys.argv[3]
else:
    if 'velodyne' in pointcloudtopic:
        sensor = 'velodyne'
    elif 'hesai' in pointcloudtopic:
        sensor = 'hesai'
    elif 'os1' in pointcloudtopic:
        sensor = 'ouster'
    else:
        sensor = sys.argv[2]

bag = rosbag.Bag(bagname)
i = 0
for topic, msg, t in bag.read_messages(topics=[pointcloudtopic]):
    if i == 0:
        print('Pointcloud2 fields: {}'.format(msg.fields))

    print('{}, header time: {}, ros time: {}'.format(i, msg.header.stamp, t))
    j = 0
    if sensor == 'velodyne':
        velodyne_gen = pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "ring", "time"), skip_nans=True)
        a = 0
        for p in velodyne_gen:
            a += 1
        velodyne_gen = pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "ring", "time"), skip_nans=True)
        # For every point in velodyne pointcloud2, its timestamp in seconds + message header timestamp is its firing time..
        for p in velodyne_gen:
            if j % 100 == 0 or j < 5 or j > a - 6:
                print("%d, x : %f  y: %f  z: %f intensity: %f ring: %d time %f sec" % (j, p[0], p[1], p[2], p[3], p[4], p[5]))
            j += 1
    elif sensor == 'hesai':
        hesai_gen = pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "timestamp", "ring"), skip_nans=True)
        # For every point in hesai pointcloud2, its timestamp in seconds is its actual firing time.
        a = 0
        for p in hesai_gen:
            a += 1
        hesai_gen = pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "timestamp", "ring"), skip_nans=True)
        for p in hesai_gen:
            if j % 100 == 0 or j < 5 or j > a - 6:
                print("%d, msg time: %d.%09d, x : %f  y: %f  z: %f intensity: %f ring: %d time %.9f sec" % (
                    j, msg.header.stamp.secs, msg.header.stamp.nsecs, p[0], p[1], p[2], p[3], p[5], p[4]))
            j += 1
    elif sensor == "ouster": # coloradar 64beam ouster
        ouster64_coloradar_gen = pc2.read_points(msg, field_names=
            ("x", "y", "z", "intensity", "t", "reflectivity", "ring", "noise", "range"), skip_nans=True)
        for p in ouster64_coloradar_gen:
            print("x: %f y: %f z: %f intensity: %f t: %f reflectivity: %f ring: %d noise: %f range: %f" % (
                p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8]))
            j += 1
            if j > 5:
                break
    elif sensor == 'points_raw':
        velodyne_raw_gen = pc2.read_points(msg, field_names=("x", "y", "z", "time", "ring"), skip_nans=True)
        for p in velodyne_raw_gen:
            print(" x : %f  y: %f  z: %f ring: %d time %f us" % (p[0], p[1], p[2], p[4], p[3]))
            j += 1
            if j > 5:
                break
    # livox lidar uses a custom message type http://docs.ros.org/en/kinetic/api/livox_ros_driver/html/msg/CustomMsg.html
    # its fields are demoed here https://github.com/hku-mars/FAST_LIO/blob/cd49bd8af90253cd0fe64edce07d1eaa72354b41/src/preprocess.cpp
    i += 1
    if i > 5:
        break

bag.close()
