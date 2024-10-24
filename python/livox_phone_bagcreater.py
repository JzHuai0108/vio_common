
import os
import subprocess
import sys
from rosbag import Bag

def create_bag(seq_dir):
    # Step 1: Call the kalibr_bagcreater.py script with the specified arguments
    video_path = os.path.join(seq_dir, 'movie.mp4')
    imu_path = os.path.join(seq_dir, 'gyro_accel.csv')
    video_time_file_path = os.path.join(seq_dir, 'frame_timestamps.txt')
    output_bag_path = os.path.join(seq_dir, 'movie.bag')

    kalibr_command = [
        'python', 'kalibr_bagcreater.py',
        '--video', video_path,
        '--imu', imu_path,
        '--video_time_file', video_time_file_path,
        '--output_bag', output_bag_path
    ]

    try:
        subprocess.run(kalibr_command, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error during bag creation: {e}")
        sys.exit(1)

def merge_mid360_bag(seq_dir):
    # Step 2: Read messages from mid360.bag and write to movie.bag
    mid_bag_path = os.path.join(seq_dir, 'mid360.bag')
    movie_bag_path = os.path.join(seq_dir, 'movie.bag')

    try:
        count = 0
        with Bag(mid_bag_path, 'r') as mid_bag, Bag(movie_bag_path, 'a') as movie_bag:
            for topic, msg, t in mid_bag.read_messages():
                # Write the message to movie.bag with the original timestamp
                movie_bag.write(topic, msg, msg.header.stamp)
                count += 1
        print("Finished writing {} messages to movie.bag".format(count))
    except Exception as e:
        print(f"Error reading or writing bag files: {e}")
        sys.exit(1)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <seq_dir>")
        sys.exit(1)

    seq_dir = sys.argv[1]

    create_bag(seq_dir)
    merge_mid360_bag(seq_dir)

