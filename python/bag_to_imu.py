#!/usr/bin/python
import os

import rosbag
import argparse

def extract(bagfile, imu_topic, out_filename):
    n = 0
    f = open(out_filename, 'w')
    f.write('#remote_timestamp ang_vel_x ang_vel_y ang_vel_z lin_acc_x lin_acc_y lin_acc_z host_time\n')
    with rosbag.Bag(bagfile, 'r') as bag:
        for (_, msg, ts) in bag.read_messages(topics=str(imu_topic)):
            f.write('%d.%09d %.8f %.8f %.8f %.8f %.8f %.8f %d.%09d\n' %
                    (msg.header.stamp.secs, msg.header.stamp.nsecs,
                     msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                     msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                     ts.secs, ts.nsecs))
            n += 1
    print('wrote ' + str(n) + ' imu messages to the file: ' + out_filename)
    f.close()

def extract_encoder(bagfile, encoder_topic, out_filename):
    n = 0
    f = open(out_filename, 'w')
    f.write('#remote_timestamp angle(rad) host_time\n')
    with rosbag.Bag(bagfile, 'r') as bag:
        for (_, msg, ts) in bag.read_messages(topics=str(encoder_topic)):
            f.write('%d.%09d %.9f %.9f %.9f %d.%09d\n' %
                    (msg.header.stamp.secs, msg.header.stamp.nsecs,
                     msg.vector.x, msg.vector.y, msg.vector.z, ts.secs, ts.nsecs))
            n += 1
    print('wrote ' + str(n) + ' encoder messages to the file: ' + out_filename)
    f.close()

def extract_time_ref(bagfile, topic, out_filename):
    n = 0
    f = open(out_filename, 'w')
    f.write('#remote_timestamp time_ref host_time\n')
    with rosbag.Bag(bagfile, 'r') as bag:
        for (_, msg, ts) in bag.read_messages(topics=str(topic)):
            f.write('%d.%09d %d.%09d %d.%09d\n' %
                    (msg.header.stamp.secs, msg.header.stamp.nsecs,
                     msg.time_ref.secs, msg.time_ref.nsecs,
                     ts.secs, ts.nsecs))
            n += 1
    print('wrote ' + str(n) + ' time_ref messages to the file: ' + out_filename)
    f.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Extracts IMU messages from a bagfile.
    ''')
    parser.add_argument('bag', help='Bagfile')
    parser.add_argument('topic', help='IMU Topic')
    parser.add_argument('--outputfile', help='Output file', default="")
    args = parser.parse_args()
    if args.outputfile == "":
        outputfile = args.bag[:-4] + "_" + os.path.basename(args.topic) + ".txt"
    else:
        outputfile = args.outputfile
    print(outputfile)
    extract(args.bag, args.topic, outputfile)
