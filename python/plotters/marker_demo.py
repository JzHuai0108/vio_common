#!/usr/bin/env python

## Simple marker publisher demo that published marker line strip messages
# to run the demo,

# catkin_make vio_common
# source devel/setup.bash
# rosrun vio_common marker_demo

import rospy

import rviz_camera_frustum
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

def marker_publisher():
    path_pub = rospy.Publisher('path', Path, queue_size=10)
    pub_odom_traj = rospy.Publisher('marker', Marker, queue_size=10)
    rospy.init_node('marker_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    i = 0
    last_pose = None
    while not rospy.is_shutdown():
        time_str = "Current time %s" % rospy.get_time()
        rospy.loginfo(time_str)
        path = Path()
        path.header.stamp = rospy.Time(1000 + i * 0.05)
        path.header.seq = i
        path.header.frame_id = "world"
        t_q = [i * 0.2, (i % 5) * 0.2, 0, 0, 0, 0, 1.0]
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = t_q[0]
        pose.pose.position.y = t_q[1]
        pose.pose.position.z = t_q[2]

        pose.pose.orientation.x = t_q[3]
        pose.pose.orientation.y = t_q[4]
        pose.pose.orientation.z = t_q[5]
        pose.pose.orientation.w = t_q[6]
        if last_pose:
            path.poses.append(last_pose)
        path.poses.append(pose)
        path_pub.publish(path)
        last_pose = pose

        marker = rviz_camera_frustum.generate_frustum_marker(t_q, 1.0)
        pub_odom_traj.publish(marker)
        i += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        marker_publisher()
    except rospy.ROSInterruptException:
        pass
