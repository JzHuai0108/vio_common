#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import tf_helpers

FOCUS = 2.5
WIDTH = 4.0
HEIGHT = 2.25
ABCDE = np.array([[0, 0, 0, 1.0],
                  [-WIDTH / 2.0, HEIGHT / 2.0, FOCUS, 1.0],
                  [WIDTH / 2.0, HEIGHT / 2.0, FOCUS, 1.0],
                  [WIDTH / 2.0, -HEIGHT / 2.0, FOCUS, 1.0],
                  [-WIDTH / 2.0, -HEIGHT / 2.0, FOCUS, 1.0]])
APEX_ORDER = "ABCDEACDAEB"
APEX_ORDER_NUM = [ord(cha) - ord('A') for cha in APEX_ORDER]

T_BC = np.array([[0, -1, 0, 0],
                 [-1, 0, 0 , 0],
                 [0, 0, -1, 0],
                 [0, 0, 0, 1]])

def generate_frustum_marker(t_q, scale=1.0, rostime=rospy.Time(0, 0),
                            red=1.0, green=1.0, blue=0.0):
    """

    :param t_q: txyz, qxyzw
    :param scale:
    :return:
    """
    marker = Marker()
    marker.header.frame_id = "/world"
    marker.header.stamp = rostime
    marker.ns = "namespace"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.r = red
    marker.color.g = green
    marker.color.b = blue
    marker.color.a = 1.0
    marker.id += 1


    T4X4 = tf_helpers.transformtransformation(t_q)
    for point_index in APEX_ORDER_NUM:
        pinC = ABCDE[point_index, :]
        pinC_scaled = pinC * scale
        pinC_scaled[3] = 1.0
        T_WC = np.dot(T4X4, T_BC)
        pinW = np.dot(T_WC, pinC_scaled)

        p = Point()
        p.x = pinW[0]
        p.y = pinW[1]
        p.z = pinW[2]
        marker.points.append(p)
    return marker

