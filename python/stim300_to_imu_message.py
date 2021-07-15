import math
import shutil

from sensor_msgs.msg import Imu
import rosbag
from tf.transformations import quaternion_from_euler

import os
import sys


def print_stim300(msg, t):
    print("stim300 header time {} ros time {} gyro(deg/s) {} {} {} acc(gravity) {} {} {} gyro_temp(C) {} {} {} acc_temp(C) {} {} {} "
          "gyro flag {} acc flag {} gyro t flag {} acc t flag {} yaw {} pitch {} roll {} lat {} lon {} h {}".format(
        msg.header.stamp, t, msg.x_gyro, msg.y_gyro, msg.z_gyro, msg.x_acc, msg.y_acc, msg.z_acc,
        msg.x_gyro_t, msg.y_gyro_t, msg.z_gyro_t, msg.x_acc_t, msg.y_acc_t, msg.z_acc_t,
        msg.gyro_flag, msg.acc_flag, msg.gyro_t_flag, msg.acc_t_flag, msg.yaw, msg.pitch, msg.roll,
        msg.lat, msg.lon, msg.alt))


def stim300_to_imu(stim300_msg, gyro_unit, acc_unit):
    rosimu = Imu()
    rosimu.header = stim300_msg.header
    rosimu.angular_velocity.x = stim300_msg.x_gyro * gyro_unit
    rosimu.angular_velocity.y = stim300_msg.y_gyro * gyro_unit
    rosimu.angular_velocity.z = stim300_msg.z_gyro * gyro_unit
    rosimu.linear_acceleration.x = stim300_msg.x_acc * acc_unit
    rosimu.linear_acceleration.y = stim300_msg.y_acc * acc_unit
    rosimu.linear_acceleration.z = stim300_msg.z_acc * acc_unit

    # RPY to convert: 90deg, 0, -90deg
    # q = quaternion_from_euler(1.5707, 0, -1.5707)
    q = quaternion_from_euler(stim300_msg.roll, stim300_msg.pitch, stim300_msg.yaw)
    rosimu.orientation.x = q[0]
    rosimu.orientation.y = q[1]
    rosimu.orientation.z = q[2]
    rosimu.orientation.w = q[3]
    return rosimu


def save_snippet(input_bag, output_bag, maxlidarcount):
    imucount = 0
    lidarcount = 0
    rate = 100
    with rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag).read_messages():
            if topic == "/imu_stim300":
                if imucount % 1000 == 0:
                    print_stim300(msg, t)
                outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
                imucount += 1
            else:
                if lidarcount % rate == 0:
                    print("lidar header time {} ros time {}".format(msg.header.stamp, t))
                outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
                lidarcount += 1
            if lidarcount > maxlidarcount:
                break


def convert_stim_messages(input_bag, output_bag, output_imu_topic):
    if os.path.isfile(output_bag):
        os.remove(output_bag)
    shutil.copy2(input_bag, output_bag)

    gravity = 9.80665
    degtorad = math.pi / 180
    imucount = 0
    lidarcount = 0
    rate = 1000
    with rosbag.Bag(output_bag, 'a') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag).read_messages():
            if topic == "/imu_stim300":
                if imucount % (rate * 10) == 0:
                    print_stim300(msg, t)
                imumsg = stim300_to_imu(msg, degtorad, gravity)
                outbag.write(output_imu_topic, imumsg, imumsg.header.stamp)
                imucount += 1
            else:
                if lidarcount % 100 == 0:
                    print("lidar header time {} ros time {}".format(msg.header.stamp, t))
                lidarcount += 1


def main():
    if len(sys.argv) < 2:
        print("Usage: {} <stim300 bagfile> [output_bag]".format(sys.argv[0]))
        sys.exit(1)
    if len(sys.argv) == 2:
        script, input_bag = sys.argv
        output_bag = os.path.join(os.path.dirname(input_bag), "converted.bag")
    else:
        script, input_bag, output_bag = sys.argv

    in_bag = rosbag.Bag(input_bag, "r")
    topic_list = in_bag.get_type_and_topic_info()[1].keys()
    print('Topics {} are in {}'.format(topic_list, input_bag))
    # save_snippet(input_bag, output_bag, 300)
    convert_stim_messages(input_bag, output_bag, '/imu0')


if __name__ == "__main__":
    main()

