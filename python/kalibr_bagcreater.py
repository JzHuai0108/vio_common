#!/usr/bin/env python
print "importing libraries"

import rosbag
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
import ImageFile
import time, sys, os
import argparse
import cv2
import numpy as np
import csv
import math

# case 1 create a rosbag from a folder as specified by the kalibr format
# https://github.com/ethz-asl/kalibr/wiki/bag-format
#structure
# dataset/cam0/TIMESTAMP.png
# dataset/camN/TIMESTAMP.png
# dataset/imu.csv

# case 2 create a rosbag from a video, a IMU file, and a video time file
# the saved bag will use the IMU clock to timestamp the messages

SECOND_TO_NANOS = 1000000000
MAX_VIDEO_FRAME_HEIGHT = 640  # For a frame in a video, if min(rows, cols) > this val, it will be half sampled.

def parseArgs():
    #setup the argument list
    parser = argparse.ArgumentParser(
        description='create a ROS bag containing image and imu topics '
                    'from either image sequences or a video and inertial data.')
    # case 1 arguments used by the kalibr to create a rosbag from a folder
    parser.add_argument('--folder',  metavar='folder', nargs='?',
                        help='Data folder whose content is structured as '
                             'specified at\nhttps://github.com/ethz-asl/kalibr/wiki/bag-format')
    parser.add_argument('--output_bag',
                        default="output.bag", help='ROS bag file %(default)s')
    
    # case 2 arguments to create a rosbag from a video, a IMU file,
    # and a video time file 
    parser.add_argument('--video',  metavar='video_file',
                        nargs='?', help='Video filename')
    parser.add_argument('--imu',  metavar='imu_file', nargs='?',
                        help='Imu filename. Except for the optional header,'
                             ' each line format\n'
                             'time[sec], gx[rad/s], gy[rad/s], gz[rad/s],'
                             ' ax[m/s^2], ay[m/s^2], az[m/s^2]')
    parser.add_argument('--video_time_offset',
                        type=float, default=0.0,
                        help='The time of the first video frame based on the'
                             ' Imu clock (default: %(default)s)',
                        required=False)
    parser.add_argument('--video_from_to', type=float,
                        nargs=2,
                        help='Use the video frames starting from up to this'
                             ' time [s] based on the video clock.')
    parser.add_argument('--video_time_file',
                        default='', nargs='?',
                        help='The csv file containing timestamps of every '
                             'video frames in IMU clock(default: %(default)s).'
                             ' Except for the header, each row has the '
                             'timestamp in sec as its first component',
                        required=False)

    if len(sys.argv)<2:
        msg = 'Example usage 1: {} --folder dataset_dir ' \
              '--output_bag awsome.bag\n'.format(sys.argv[0])
        msg += 'Example usage 2: {} --video dataset_dir/IMG_2805.MOV ' \
               '--imu dataset_dir/gyro_accel.csv --video_time_file ' \
               'dataset_dir/movie_metadata.csv ' \
               '--output_bag dataset_dir/IMG_2805.bag\n'.format(sys.argv[0])
        msg += 'Example usage 3: {} --video dataset_dir/IMG_2805.MOV ' \
               '--imu dataset_dir/gyro_accel.csv --video_time_file ' \
               'dataset_dir/frame_timestamps.txt ' \
               '--output_bag dataset_dir/IMG_2805.bag\n '.format(sys.argv[0])
        msg += ('For the latter two cases, the number of entries in the '
                'video_time_file excluding its header lines has to be the '
                'same as the number of frames in the video.\nOtherwise, '
                'exception will be thrown')

        print(msg)
        parser.print_help()
        sys.exit(1)

    parsed = parser.parse_args()
    return parsed

def getImageFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    image_files = list()
    timestamps = list()
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg']:
                    image_files.append( os.path.join( path, f ) )
                    timestamps.append(os.path.splitext(f)[0]) 
    #sort by timestamp
    sort_list = sorted(zip(timestamps, image_files))
    image_files = [file[1] for file in sort_list]
    return image_files

def getCamFoldersFromDir(dir):
    '''Generates a list of all folders that start with cam e.g. cam0'''
    cam_folders = list()
    if os.path.exists(dir):
        for path, folders, files in os.walk(dir):
            for folder in folders:                
                if folder[0:3] == "cam":
                    cam_folders.append(folder)
    return cam_folders

def getImuCsvFiles(dir):
    '''Generates a list of all csv files that start with imu'''
    imu_files = list()
    if os.path.exists(dir):
        for path, folders, files in os.walk(dir):
            for file in files:
                if file[0:3] == 'imu' and os.path.splitext(file)[1] == ".csv":
                    imu_files.append( os.path.join( path, file ) )
    
    return imu_files

def loadImageToRosMsg(filename):
    image_np = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
    
    timestamp_nsecs = os.path.splitext(os.path.basename(filename))[0]
    timestamp = rospy.Time( secs=int(timestamp_nsecs[0:-9]), nsecs=int(timestamp_nsecs[-9:]) )

    rosimage = Image()
    rosimage.header.stamp = timestamp
    rosimage.height = image_np.shape[0]
    rosimage.width = image_np.shape[1]
    rosimage.step = rosimage.width  #only with mono8! (step = width * byteperpixel * numChannels)
    rosimage.encoding = "mono8"
    rosimage.data = image_np.tostring()
    
    return rosimage, timestamp

def parse_time(timestamp_str):
    """
    convert a timestamp string to a rospy time
    if a dot is in the string, the string is taken as an int in nanosecs
    otherwise, taken as an float in secs
    :param timestamp_str:
    :return:
    """


    secs = 0
    nsecs = 0
    if '.' in timestamp_str:
        index = timestamp_str.find('.')
        if index == 0:
            nsecs = int(float(timestamp_str[index:]) * SECOND_TO_NANOS)
        elif index == len(timestamp_str) - 1:
            secs = int(timestamp_str[:index])
        else:
            secs = int(timestamp_str[:index])
            nsecs = int(float(timestamp_str[index:]) * SECOND_TO_NANOS)
        return secs, nsecs
    else:
        return int(timestamp_str[0:-9]), int(timestamp_str[-9:])


def createImuMessge(timestamp_str, omega, alpha):
    secs, nsecs = parse_time(timestamp_str)
    timestamp = rospy.Time(secs, nsecs)
    rosimu = Imu()
    rosimu.header.stamp = timestamp
    rosimu.angular_velocity.x = float(omega[0])
    rosimu.angular_velocity.y = float(omega[1])
    rosimu.angular_velocity.z = float(omega[2])
    rosimu.linear_acceleration.x = float(alpha[0])
    rosimu.linear_acceleration.y = float(alpha[1])
    rosimu.linear_acceleration.z = float(alpha[2])
    
    return rosimu, timestamp

def playAVideo(videoFilename):
    cap = cv2.VideoCapture(videoFilename)
    rate = cap.get(cv2.CAP_PROP_FPS)
    print "video frame rate", rate
    mnStartId = 0
    mnFinishId = 1e6
    cap.set(cv2.CAP_PROP_POS_FRAMES, mnStartId); #start from mnStartId, 0 based index
    if(mnFinishId == -1):
        mnFinishId = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)-1)
    else:
        mnFinishId = min(mnFinishId, int(cap.get(cv2.CAP_PROP_FRAME_COUNT)-1))
    print 'start', mnStartId, 'finish', mnFinishId

    mnCurrentId = mnStartId

    while(cap.isOpened()):
        videoFrameId = cap.get(cv2.CAP_PROP_POS_FRAMES)
        if videoFrameId != mnCurrentId:
            print "Expected frame id", mnCurrentId, "and actual one in video", videoFrameId, "differ."
            print "Likely reached end of video file. Note mnFinishId", mnFinishId
            break        
           
        time_frame= cap.get(cv2.CAP_PROP_POS_MSEC)/1000.0
        # print('currentFrameId {} and timestamp in video {:.9f}'.format(mnCurrentId, time_frame))
        ret, frame = cap.read()
        if frame is None:
            print 'Empty frame, break the video stream'
            break  
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h,w = gray.shape[:2]
        if h > MAX_VIDEO_FRAME_HEIGHT:
            gray = cv2.pyrDown(gray, dstsize = (w/2,h/2))

        cv2.imshow('frame',gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        mnCurrentId += 1
        if mnCurrentId > mnFinishId:
            print 'Exceeding mnFinishId %d' % mnFinishId + ', break the video stream'              
            break            

    cap.release()
    cv2.destroyAllWindows()


def writeVideoToRosBag(bag, videoFilename, video_from_to, video_time_offset=0.0, frame_timestamps=None):
    """return video time range in imu clock"""
    cap = cv2.VideoCapture(videoFilename)
    rate = cap.get(cv2.CAP_PROP_FPS);
    print "video frame rate", rate

    mnStartId = 0
    mnFinishId = 1000000
    literalvideofromto = list()
    framesinvideo = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    print('#Frames {} in the video {}'.format(framesinvideo, videoFilename))
    if frame_timestamps is None:
        frame_timestamps = list()
    if len(frame_timestamps) > 0:
        if len(frame_timestamps) != framesinvideo:
            raise Exception("Number of frames in the video disagrees with the length of the provided timestamps".
                            format(framesinvideo, len(frame_timestamps)))
    if mnFinishId == -1:
        mnFinishId = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)-1)
    else:
        mnFinishId = int(min(mnFinishId, framesinvideo-1))
    if video_from_to and rate > 1.0:
        mnStartId = int(max(mnStartId, video_from_to[0]*rate))
        mnFinishId = int(min(mnFinishId, video_from_to[1]*rate))
    literalvideofromto.append(max(float(mnStartId)/rate - 1.0, 0.0) + video_time_offset)
    literalvideofromto.append(float(mnFinishId)/rate + 1.0 + video_time_offset)
    print 'video frame index start', mnStartId, 'finish', mnFinishId
    cap.set(cv2.CAP_PROP_POS_FRAMES, mnStartId); #start from mnStartId, 0 based index
    mnCurrentId = mnStartId
    framecount = 0
    while(cap.isOpened()):
        if mnCurrentId > mnFinishId:
            print 'Exceeding mnFinishId %d' % mnFinishId + ', break the video stream'
            break

        videoFrameId = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        if videoFrameId != mnCurrentId:
            message = "Expected frame id {} and actual id in video {} differ.\n".format(mnCurrentId, videoFrameId)
            message += "Likely reached end of video file. Note mnFinishId is {}".format(mnFinishId)
            print(message)
            break

        time_frame = cap.get(cv2.CAP_PROP_POS_MSEC)/1000.0
        time_frame_offset = time_frame + video_time_offset
        # print('currentFrameId {} and timestamp in video {:.9f}'.format(mnCurrentId, time_frame))
        ret, frame = cap.read()
        if frame is None:
            print 'Empty frame, break the video stream'
            break  

        image_np = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = image_np.shape[:2]

        if h > MAX_VIDEO_FRAME_HEIGHT:
            image_np = cv2.pyrDown(image_np,dstsize = (w/2,h/2))
        if w < h:
            image_np = cv2.rotate(image_np, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imshow('frame', image_np)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if len(frame_timestamps) > 0: # external
            timestamp = frame_timestamps[videoFrameId]
        else:
            decimal, integer = math.modf(time_frame_offset)
            timestamp = rospy.Time( secs=int(integer), nsecs=int(decimal*1e9) )

        rosimage = Image()
        rosimage.header.stamp = timestamp
        rosimage.height = image_np.shape[0]
        rosimage.width = image_np.shape[1]
        rosimage.step = rosimage.width  # only with mono8! (step = width * byteperpixel * numChannels)
        rosimage.encoding = "mono8"
        rosimage.data = image_np.tostring()

        topic_prefix = 'cam0'
        framecount += 1
        mnCurrentId += 1
        bag.write("/{0}/image_raw".format(topic_prefix), rosimage, timestamp)
    cap.release()
    cv2.destroyAllWindows()
    print('Saved {} out of {} video frames as image messages to the rosbag'.format(framecount, framesinvideo))
    return literalvideofromto

def loadtimestamps(video_time_file):
    '''Except for the header, each row has the timestamp in sec as its first component'''
    frame_rostimes = list()
    with open(video_time_file, 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        next(reader, None) # headers
        for row in reader:
            secs, nsecs = parse_time(row[0])
            frame_rostimes.append(rospy.Time(secs, nsecs))
    return frame_rostimes

def is_float(element_str):
    """check if a string represent a float. To this function, 30e5 is float, but 2131F or 2344f is not float"""
    try:
        float(element_str)
        return True
    except ValueError:
        return False

#create the bag
def main():
    parsed = parseArgs()
    
    bag = rosbag.Bag(parsed.output_bag, 'w')
    if not parsed.video is None:
        print 'given video time offset', parsed.video_time_offset
        if parsed.video_from_to:
            print 'video_from_to:', parsed.video_from_to
        frame_timestamps = list()
        if parsed.video_time_file:
            frame_timestamps = loadtimestamps(parsed.video_time_file)
            print('Loaded {} timestamps for frames'.format(len(frame_timestamps)))
        videotimerange = writeVideoToRosBag(bag, parsed.video, parsed.video_from_to,
                                            video_time_offset=parsed.video_time_offset,
                                            frame_timestamps=frame_timestamps)

    elif not parsed.folder is None:
        #write images
        camfolders = getCamFoldersFromDir(parsed.folder)
        for camfolder in camfolders:
            camdir = parsed.folder + "/{0}".format(camfolder)
            image_files = getImageFilesFromDir(camdir)
            for image_filename in image_files:
                image_msg, timestamp = loadImageToRosMsg(image_filename)
                bag.write("/{0}/image_raw".format(camfolder), image_msg, timestamp)
    else:
        raise Exception('Invalid/Empty video file and image folder')

    #write imu data
    if parsed.imu is None and parsed.folder is None:
        print "Neither a folder nor any imu file is provided. Rosbag will have only visual data"
    elif parsed.imu is None:
        imufiles = getImuCsvFiles(parsed.folder)
        for imufile in imufiles:
            topic = os.path.splitext(os.path.basename(imufile))[0]
            with open(imufile, 'rb') as csvfile:
                reader = csv.reader(csvfile, delimiter=',')
                headers = next(reader, None)
                for row in reader:
                    imumsg, timestamp = createImuMessge(row[0], row[1:4], row[4:7])
                    bag.write("/{0}".format(topic), imumsg, timestamp)
    else:
        imufile = parsed.imu
        topic = 'imu0'

        with open(imufile, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            headers = next(reader, None)
            rowcount = 0
            imucount = 0
            for row in reader: # note row[i] are strings
                if not is_float(row[0]):
                    continue
                imumsg, timestamp = createImuMessge(row[0], row[1:4], row[4:7])
                timestampinsec = timestamp.to_sec()
                rowcount += 1
                # check the below conditions when video and imu use different clocks and their lengths differ much
                if abs(parsed.video_time_offset) > 1e-8 and videotimerange and \
                        (timestampinsec < videotimerange[0] - MARGIN_TIME or
                         timestampinsec > videotimerange[1] + MARGIN_TIME):
                    continue
                imucount += 1
                bag.write("/{0}".format(topic), imumsg, timestamp)
            print('Saved {} out of {} inertial messages to the rosbag'.format(imucount, rowcount))

    bag.close()
    print 'Saved to bag file', parsed.output_bag

MARGIN_TIME = 5 # sec

if __name__ == "__main__":
    main()
