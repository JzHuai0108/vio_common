# given a rosbag with bagname
# extract an image on topic /cam0/image_raw at bag begin time + start time
# then save the image
import rosbag # pip install pycryptodome pycryptodomex gnupg rospkg
import cv2 # conda install opencv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def extract_and_save_image(bagname, topic, start_time, outputfile):
    """
    Extracts an image from a ROS bag at a specific time and saves it.

    Args:
        bagname (str): Path to the ROS bag file.
        topic (str): Topic to extract the image from (e.g., /cam0/image_raw).
        start_time (float): Time in seconds relative to the start of the bag.
        outputfile (str): Path to save the extracted image.
    """
    bridge = CvBridge()

    with rosbag.Bag(bagname, 'r') as bag:
        # Get the start time of the bag
        bag_start_time = bag.get_start_time()
        target_time = bag_start_time + start_time

        # Iterate through messages on the specified topic
        for topic_name, msg, t in bag.read_messages(topics=[topic]):
            if t.to_sec() >= target_time:
                # Convert ROS Image message to OpenCV image
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                
                # Save the image
                cv2.imwrite(outputfile, cv_image)
                print(f"Image saved to {outputfile}")
                return
        
        print("No image found at the specified time.")
