"""
split a rosbag into several segments
"""
import argparse
import os
import sys
import random

import rospy
import rosbag
from cv_bridge import CvBridge
import cv2

def parse_args():
    parser = argparse.ArgumentParser(description='split a rosbag into several segments.')

    parser.add_argument('image_bag',
                        help='ROS bag file containing an image topic')

    parser.add_argument('--image_topic',
                        type=str,
                        default="/cam0/image_raw",
                        help='image topic')

    parser.add_argument("--frames_per_split", type=int, default=110,
                        help="number of frames in a split. (default: %(default)s)")
    parser.add_argument('--splits_to_save', type=int, default=3,
                        help="save how many splits?. (default: %(default)s)")

    if len(sys.argv) < 2:
        msg = 'Example usage: {} image.bag\n'. \
            format(sys.argv[0])
        print(msg)
        parser.print_help()
        sys.exit(1)

    parsed = parser.parse_args()
    return parsed


def main():
    parsed = parse_args()
    output_path = os.path.dirname(parsed.image_bag)
    stubname = os.path.basename(parsed.image_bag).split('.')[0]
    output_names = [os.path.join(output_path, stubname + "_part" + str(x) + '.bag')
                    for x in range(parsed.splits_to_save)]
    output_bags = []
    output_count = []
    for fn in output_names:
        output_bags.append(rosbag.Bag(fn, 'w'))
        output_count.append(0)
    in_bag = rosbag.Bag(parsed.image_bag, "r")
    totalFrames = in_bag.get_message_count(parsed.image_topic)
    frames_per_split = parsed.frames_per_split
    if totalFrames < frames_per_split * parsed.splits_to_save:
        frames_per_split = totalFrames // parsed.splits_to_save
    print("total frames: {}, frames per split: {}".format(totalFrames, frames_per_split))

    idlist = list(range(totalFrames))
    random.shuffle(idlist)
    windows = [idlist[i * frames_per_split : (i + 1) * frames_per_split] for i in range(parsed.splits_to_save)]

    for i, w in enumerate(windows):
        windows[i] = sorted(w)
        print("window: {}, #frames {} first {} last {}".format(
            i, len(windows[i]), windows[i][0], windows[i][-1]))

    bridge = CvBridge()
    count = 0
    lastindices = [0 for _ in windows]

    for _, msg, t in in_bag.read_messages(topics=[parsed.image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv2.imshow('Frame', cv_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        for wid, index in enumerate(lastindices):
            if index >= len(windows[wid]):
                continue
            if count == windows[wid][index]:
                output_bags[wid].write(parsed.image_topic, msg, t)
                output_count[wid] += 1
                lastindices[wid] += 1
                break
        count += 1
    in_bag.close()

    for i, bag in enumerate(output_bags):
        print('Split {} saved #frames {} to {}.'.format(i, output_count[i], output_names[i]))
        bag.close()


if __name__ == "__main__":
    main()
