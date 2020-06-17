#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""
Half sample images from a rosbag and saved to another rosbag.
The data from other topics are copied directly to the output rosbag.
The image topics are preset to be /cam0/image_raw and /cam1/image_raw.
"""

import os
import argparse

import cv2

import rosbag
from cv_bridge import CvBridge
import play_images_in_rosbag


def decide_output_encoding(cv_img):
    coding = 'passthrough'
    if cv_img.dtype == 'uint16':
        coding = 'mono16'
    return coding


def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Downsample images in a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("--out_bag_file", help="Output ROS bag file.", default=None)

    args = parser.parse_args()
    out_bag_file = args.out_bag_file
    if args.out_bag_file is None:
        out_bag_file = os.path.join(os.path.splitext(args.bag_file)[0] + '_512_16.bag')

    in_bag = rosbag.Bag(args.bag_file, "r")
    topic_list = in_bag.get_type_and_topic_info()[1].keys()
    print('Topics {} are in {}'.format(topic_list, args.bag_file))

    out_bag = rosbag.Bag(out_bag_file, 'w')
    bridge = CvBridge()
    for k in range(2):
        count = 0
        image_topic = '/cam{}/image_raw'.format(k)
        encoding = ''
        for _, msg, t in in_bag.read_messages(topics=[image_topic]):
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            h, w = cv_img.shape[:2]
            cv_half_img = cv2.pyrDown(cv_img, dstsize=(w / 2, h / 2))
            if count == 0:
                print('Image info before and after half sampling:')
                play_images_in_rosbag.print_image_info(cv_img)
                play_images_in_rosbag.print_image_info(cv_half_img)
                encoding = decide_output_encoding(cv_img)

            cv2.imshow('Downsampled frame', cv_half_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            count += 1

            # For 16UC1 input image type, we may need to use mono16 as output encoding option.
            # see http://library.isr.ist.utl.pt/docs/roswiki/cv_bridge(2f)Tutorials(2f)UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages.html
            rosimage = bridge.cv2_to_imgmsg(cv_half_img, encoding=encoding)
            rosimage.header.stamp = t  # This is needed because header.stamp could have been 0.0.
            out_bag.write(image_topic, rosimage, t)
        print('Saved {} images on topic {}'.format(count, image_topic))

    # copy the other messages
    for topic in topic_list:
        if 'image_raw' in topic:
            continue
        count = 0
        for returned_topic, msg, t in in_bag.read_messages(topics=[topic]):
            msg.header.stamp = t  # This is needed because header.stamp could have been very different from t.
            out_bag.write(topic, msg, t)
            count += 1
        print('Saved {} messages on topic {}'.format(count, topic))

    cv2.destroyAllWindows()
    out_bag.close()
    in_bag.close()

    print("Downsampled images of {} on topics /cam0/image_raw /cam1/image_raw into {}".
          format(args.bag_file, out_bag_file))


if __name__ == '__main__':
    main()
