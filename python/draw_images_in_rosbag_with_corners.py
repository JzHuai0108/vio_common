#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Display images in a rosbag on a specific image topic, and draw 
the calibration target corners saved by TartanCalib.
and optionally save image messages and their timestamps.
"""
from __future__ import print_function

import argparse
import os

import numpy as np
import rosbag
import rospy
from cv_bridge import CvBridge
import cv2
from scipy.io import loadmat

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


def getObjectAndImagePoints(board, corners):
    opts = []
    ipts = []

    boardpts = board['boards'][0, 0]['X']  # 2 x M
    numviews = corners['corners'].shape[1]
    for im in range(numviews):
        m = corners['corners'][0, im]['x'][0, 0].shape[1]
        if m > 0:
            objpts = np.zeros((m, 1, 3), np.float32)
            imgpts = np.zeros((m, 1, 2), np.float32)
            ids = corners['corners'][0, im]["cspond"][0, 0].astype(int)
            imgarr = corners['corners'][0, im]['x'][0, 0]

            for kp in range(m):
                imgpts[kp, 0, :] = imgarr[:, kp]
                pid = ids[0, kp] - 1
                objpts[kp, 0, :2] = boardpts[:, pid]
            opts.append(objpts)
            ipts.append(imgpts)

    return opts, ipts

def main():
    parser = argparse.ArgumentParser(
        description="Display images in a ROS bag. compressedImage is not supported.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("matdir", type=str, default="/path/to/boardandcorners",
                      help="the folder containing board.mat and corners.mat")

    parser.add_argument("--image_topic",
                        help="Display images from which topic.",
                        default='/cam0/image_raw')
    parser.add_argument("--outputdir",
                        default=None,
                        help="Dump the images to the directory.")
    parser.add_argument('--time_from_to',
                        type=float,
                        nargs=2,
                        default=[0, 1e8],
                        help='Show the frames starting from up to this'
                        ' time [s] relative to the bag start time.')
    args = parser.parse_args()

    in_bag = rosbag.Bag(args.bag_file, "r")
    startTime = in_bag.get_start_time()

    topic_list = in_bag.get_type_and_topic_info()[1].keys()
    print('Topics {} are in {}'.format(topic_list, args.bag_file))

    print('Press q key to quit.')
    bridge = CvBridge()
    count = 0
    image_local_time = [None, None]
    image_remote_time = [None, None]
    time_stream = None

    boardfile = os.path.join(args.matdir, 'board.mat')
    cornerfile = os.path.join(args.matdir, 'corners.mat')

    board = loadmat(boardfile)
    corners = loadmat(cornerfile)

    origopts, origipts = getObjectAndImagePoints(board, corners)
    origtimes = corners['times']
    tol = 1e-6
    k = 0

    for _, msg, t in in_bag.read_messages(topics=[args.image_topic], start_time=rospy.Time(startTime + args.time_from_to[0]),
                                          end_time=rospy.Time(startTime + args.time_from_to[1])):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if count == 0:
            print('Video frame info:')
            print_image_info(cv_img)
            image_local_time[0] = t
            image_remote_time[0] = msg.header.stamp
            if args.outputdir:
                time_stream = open(os.path.join(args.outputdir, "timestamps.txt"), 'w')
                time_stream.write('#sensor time, computer time\n')
        if time_stream:
            time_stream.write('{},{}\n'.format(msg.header.stamp, t))
        image_local_time[1] = t
        image_remote_time[1] = msg.header.stamp
        if args.outputdir:
            imagename = os.path.join(args.outputdir, '{}.jpg'.format(msg.header.stamp))
            cv2.imwrite(imagename, cv_img)
        if abs(t.to_sec() - origtimes[0, k]) < tol:
            qtime = origtimes[0, k]
            blank = np.zeros((1, 1))
            keypoints = []
            numkps = origipts[k].shape[0]
            minrc = [100000, 100000]
            maxrc = [0, 0]
            imgsize = cv_img.shape
            for i in range(numkps):
                kp = cv2.KeyPoint(origipts[k][i, 0, 0], origipts[k][i, 0, 1], 2)
                minrc[0] = min(minrc[0], origipts[k][i, 0, 1])
                minrc[1] = min(minrc[1], origipts[k][i, 0, 0])
                maxrc[0] = max(maxrc[0], origipts[k][i, 0, 1])
                maxrc[1] = max(maxrc[1], origipts[k][i, 0, 0])
                keypoints.append(kp)
            blobs = cv2.drawKeypoints(cv_img, keypoints, blank, (0, 0, 255),
                                      cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            blobs = blobs[int(round(max(minrc[0] - 100, 0))):int(round(min(maxrc[0] + 100, imgsize[0]))),
                    int(round(max(minrc[1] - 100, 0))):int(round(min(maxrc[1] + 100, imgsize[1])))]
            text = "Time {}, #Keypoints {}".format(qtime, len(keypoints))
            cv2.putText(blobs, text, (20, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 100, 255), 2)
            k += 1
            cv2.imshow("Detected corners", blobs)
            if cv2.waitKey(2500) & 0xFF == ord('q') or k == origtimes.shape[1]:
                break
        count += 1
    if time_stream:
        time_stream.close()
    cv2.destroyAllWindows()
    in_bag.close()
    print("Displayed {} images of {} on topic {}\n\t\t\tlocal time\t\t\tremote time\nfirst image\t{}\t{}\n"
          "last image\t{}\t{}\n".format(count, args.bag_file, args.image_topic,
                                        image_local_time[0], image_remote_time[0],
                                        image_local_time[1], image_remote_time[1]))


if __name__ == '__main__':
    main()
