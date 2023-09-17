#!/usr/bin/python

import rosbag
import argparse
import os

def extract(bagfile, topic, out_filename):
    n = 0
    f = open(out_filename, 'w')
    f.write("#remote_time host_time\n")
    with rosbag.Bag(bagfile, 'r') as bag:
        for (_, msg, ts) in bag.read_messages(topics=str(topic)):
            f.write("%d.%09d %d.%09d\n" % (msg.header.stamp.secs, msg.header.stamp.nsecs, ts.secs, ts.nsecs))
            n += 1
    print('wrote ' + str(n) + ' time messages to the file: ' + out_filename)
    f.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Extracts time for messages on a bagfile topic.
    ''')
    parser.add_argument('bag', help='Bagfile')
    parser.add_argument('topic', help='Topic')
    parser.add_argument('--outputfile', help='Output file', default="")
    args = parser.parse_args()
    if args.outputfile == "":
        outputfile = args.bag[:-4] + "_" + os.path.basename(args.topic) + "_times.txt"
    else:
        outputfile = args.outputfile
    print("Output file: " + outputfile)
    extract(args.bag, args.topic, outputfile)
