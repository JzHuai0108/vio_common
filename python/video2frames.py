#!/usr/bin/env python
"""convert a video to a seq of images
   Ref: https://www.pyimagesearch.com/2017/01/09/count-the-total-number-of-frames-in-a-video-with-opencv-and-python/
   assume opencv3 is used
"""

from __future__ import print_function
import time
import sys
import os
import shutil
import argparse
import cv2

def parseArgs():
    #setup the argument list
    parser = argparse.ArgumentParser(
        description='Convert a video to a sequence of image frames, optionally downsample.')
    parser.add_argument('--output-folder', metavar='output_folder',
                        help='An folder to output the image sequences', required=True)
    parser.add_argument('--video',  metavar='video_file',
                        nargs='?', help='Video filename', required=True)
    parser.add_argument('--video-from-to', metavar='video_from_to', type=int,
                        nargs=2,
                        help='Use the video frames starting from up to this'
                             ' number based on the video frames(0-based indexing).')
    parser.add_argument('--choose-every-n', type=int,
                        nargs='?',
                        help='Downsample the frames to frame_count / choose_every_n.')
    parser.add_argument('--downsample-by-2',  action='store_true', dest='downsample_by_2',
                        help='Downsample a frame to half of its original width and length.')

    #print help if no argument is specified
    if len(sys.argv)<2:
        msg = 'Example usage: {} --video video_and_frame_timestamps/IMG_2805.MOV ' \
               '--output-folder video_and_frame_timestamps/IMG_2805/ ' \
              '--video-from-to 0 500 --choose-every-n 2 --downsample-by-2\n '.format(sys.argv[0])

        print(msg)
        parser.print_help()
        sys.exit(1)

    parsed = parser.parse_args()
    return parsed

def emptyfolder(folder):
    for the_file in os.listdir(folder):
        file_path = os.path.join(folder, the_file)
        try:
            if os.path.isfile(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path): shutil.rmtree(file_path)
        except Exception as e:
            print(e)

def video_to_frames(input_loc, output_loc, video_from_to=None,
                    choose_every_n=None,
                    downsample_by_2=None):
    """Function to extract frames from input video file
    and save them as separate frames in an output directory.
    Args:
        input_loc: Input video file.
        output_loc: Output directory to save the frames.
    Returns:
        None
    """
    try:
        os.mkdir(output_loc)
    except OSError:
        pass
    emptyfolder(output_loc)
    # Log the time
    time_start = time.time()
    # Start capturing the feed
    cap = cv2.VideoCapture(input_loc)
    # Find the number of frames
    video_length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print("#frames in video: %d" % video_length)

    if video_from_to:
        minframeid = max(0, video_from_to[0])
        maxframeid = min(video_length - 1, video_from_to[1])
    else:
        minframeid = 0
        maxframeid = video_length - 1

    if choose_every_n:
        frame_ids = range(minframeid, maxframeid + 1, choose_every_n)
    else:
        frame_ids = range(minframeid, maxframeid + 1)

    print("Converting video...\n")
    count = 0
    savedcount = 0
    while cap.isOpened():
        # Extract the frame
        ret, frame = cap.read()
        if frame is None:
            print('Empty frame, break the video stream, latest frame id %d' % count)
            break
        if count in frame_ids:
            image_np = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            h, w = image_np.shape[:2]
            if downsample_by_2:
                image_np = cv2.pyrDown(image_np, dstsize=(w / 2, h / 2))
            # Write the results back to output location.
            cv2.imwrite(os.path.join(output_loc, "%05d.jpg" % count), image_np)
            savedcount += 1
            cv2.imshow('frame', image_np)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        count += 1
        if count > (video_length - 1):
            print("Reach end of video, lastest frame id %d" % count)
            break

    # Log the time again
    time_end = time.time()
    # Release the feed
    cap.release()
    cv2.destroyAllWindows()
    # Print stats
    print("Done extracting frames.\n%d frames extracted out of #frames in video %d" % (savedcount, video_length))
    print("It took %d seconds for conversion." % (time_end-time_start))


def main():
    parsed = parseArgs()
    video_to_frames(parsed.video, parsed.output_folder, parsed.video_from_to,
                    parsed.choose_every_n, parsed.downsample_by_2)

if __name__ == "__main__":
    main()
