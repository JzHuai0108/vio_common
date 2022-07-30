#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import os

import rosbag
import rospy
from cv_bridge import CvBridge
import cv2


def print_image_info(cv_img):
    h, w = cv_img.shape[:2]
    dtype = cv_img.dtype
    lenshape = len(cv_img.shape)
    if lenshape == 2:
        channels = 1
    else:
        channels = cv_img.shape[2]
    print('Image h {} w {} data type {} channels {}'.format(
        h, w, dtype, channels))


def createImagePath(outputdir, datatype, index, bundleFusionLayout):
    """
    bundleFusionLayout true bundleFusion rgbd file structure
        false open3d rgbd file structure
    """
    if bundleFusionLayout:
        if datatype == 'color':
            return os.path.join(outputdir, 'frame-{:06d}.color.jpg'.format(index))
        elif datatype == 'depth':
            return os.path.join(outputdir, 'frame-{:06d}.depth.png'.format(index))
        else:
            return os.path.join(outputdir, 'frame-{:06d}.{}.txt'.format(index, datatype))
    else: # open3d layout
        return os.path.join(outputdir, datatype, '{:06d}.png'.format(index))


def createTimePath(outputdir, datatype, bundleFusionLayout):
    if bundleFusionLayout:
        return os.path.join(outputdir, 'timestamps.{}.txt'.format(datatype))
    else:
        return os.path.join(outputdir, datatype, "timestamps.txt")

def saveIdentityPose(posefile):
    stream = open(posefile, 'w')
    stream.write("1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1")
    stream.close()


def main():
    parser = argparse.ArgumentParser(
        description="Extract synced rgb and depth images from a ROS bag. "
        "Supports 16UC1, but compressedImage is not supported.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("--image_topics",
                        nargs=2, default='[/rgb/image_raw, /depth/image_raw]',
                        help="Display images from which topic(s). (default: %(default)s)")
    parser.add_argument("--outputdir",
                        default=None,
                        help="Dump the images to the directory. (default: %(default)s)")
    parser.add_argument('--time_from_to',
                        type=float,
                        nargs=2,
                        default=[0, 1e8],
                        help='Show the frames starting from up to this'
                        ' time [s] relative to the bag start time. (default: %(default)s)')
    parser.add_argument('--sync_tol',
                        type=int,
                        default=2000,
                        help='Time tolerance in microseconds to consider two frames from '
                         'two topics match well in time. (default: %(default)s)')
    parser.add_argument("--bundle_fusion_layout",
                        action='store_true',
                        help='save the rgb and depth images in the bundle fusion layout, '
                        'otherwise by default, in the open3d layout.')
    args = parser.parse_args()

    in_bag = rosbag.Bag(args.bag_file, "r")
    startTime = in_bag.get_start_time()

    bundleFusionLayout = args.bundle_fusion_layout
    topic_list = in_bag.get_type_and_topic_info()[1].keys()
    print('Topics {} are in {}'.format(topic_list, args.bag_file))

    print('Press q key to quit.')

    outputdir = args.outputdir
    if outputdir is None:
        outputdir = args.bag_file.split('.')[0]
    if not os.path.exists(outputdir):
        os.mkdir(outputdir)

    msgIter = in_bag.read_messages(topics=args.image_topics, start_time=rospy.Time(startTime + args.time_from_to[0]),
                                          end_time=rospy.Time(startTime + args.time_from_to[1]))
    
    # A simple sync strategy
    synced = False
    syncTol = rospy.Duration(args.sync_tol // int(1e6), (args.sync_tol % int(1e6)) * 1000)
    syncTolNsec = syncTol.to_nsec()

    bridge = CvBridge()

    basenames = [topic.strip('/').split('/')[0] for topic in args.image_topics]
    outputbasenames = []
    for id, bn in enumerate(basenames):
        if bn.startswith('depth'):
            outputbasenames.append('depth')
        elif bn.startswith('rgb'):
            outputbasenames.append('color')
        else:
            outputbasenames.append(bn)

    time_streams = []
    msgCounts = []
    lastMessageOfTopics = []
    for tid, topic in enumerate(args.image_topics):
        time_streams.append(None)
        msgCounts.append(0)
        lastMessageOfTopics.append(None)
        subdir = os.path.join(outputdir, outputbasenames[tid])
        if not bundleFusionLayout and not os.path.exists(subdir):
            os.mkdir(subdir)

    maxGapUnderTol = rospy.Duration(0, 0)
    msgCount = 0
    matchCount = 0
    for topic, msg, t in msgIter:
        tid = args.image_topics.index(topic)
        otherid = (tid + 1) % 2  # TODO(jhuai): improve this hack.

        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if msgCount == 0:
            print('Video frame info:')
            print_image_info(cv_img)

            for id, bn in enumerate(outputbasenames):
                time_streams[id] = open(createTimePath(outputdir, bn, bundleFusionLayout), 'w')
                time_streams[id].write('#index, sensor time, host time\n')
        matched = False
        if lastMessageOfTopics[otherid]:
            gap = msg.header.stamp - lastMessageOfTopics[otherid][1].header.stamp
            if abs(gap.to_nsec()) < syncTolNsec:
                matched = True
                print('matched times {} {} of gap {} ns.'.format(msg.header.stamp.to_nsec(), 
                    lastMessageOfTopics[otherid][1].header.stamp.to_nsec(), gap.to_nsec()))
                if gap > maxGapUnderTol:
                    maxGapUnderTol = gap

        if matched:
            time_streams[tid].write('{},{},{}\n'.format(matchCount, msg.header.stamp, t))
            time_streams[otherid].write('{},{},{}\n'.format(matchCount, lastMessageOfTopics[otherid][1].header.stamp, lastMessageOfTopics[otherid][2]))
            
            imagename = createImagePath(outputdir, outputbasenames[tid], matchCount, bundleFusionLayout)
            cv2.imwrite(imagename, cv_img)
            
            cv_img = bridge.imgmsg_to_cv2(lastMessageOfTopics[otherid][1], desired_encoding="passthrough")
            imagename = createImagePath(outputdir, outputbasenames[otherid], matchCount, bundleFusionLayout)
            cv2.imwrite(imagename, cv_img)

            if bundleFusionLayout:
                posename = createImagePath(outputdir, 'pose', matchCount, bundleFusionLayout)
                saveIdentityPose(posename)

            matchCount += 1
            for id, bn in enumerate(outputbasenames):
                lastMessageOfTopics[id] = None
        else:
            lastMessageOfTopics[tid] = [topic, msg, t]
      
        cv2.imshow(outputbasenames[tid], cv_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        msgCount += 1
        msgCounts[tid] += 1

    for stream in time_streams:
        if stream:
            stream.close()
    cv2.destroyAllWindows()
    in_bag.close()
    print('Topic\t\t#Messages')
    for id, count in enumerate(msgCounts):
        print('{}\t{}'.format(args.image_topics[id], count))
    print("Displayed {} images of {} on topics {}. Max time gap {} microsecs for #matched frames {}.\n".format(
        msgCount, args.bag_file, args.image_topics, maxGapUnderTol.to_nsec() // 1000, matchCount))


if __name__ == '__main__':
    main()
