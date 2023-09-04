# !/usr/bin/python
import os
import rosbag
import argparse
import pyproj
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp


def wgs2ecef(lat, lon, alt):
    crs_lla = pyproj.CRS.from_epsg(4326)
    crs_ECEF = pyproj.CRS.from_epsg(4978)
    transformer = pyproj.Transformer.from_crs(crs_lla, crs_ECEF, always_xy=True)
    x, y, z = transformer.transform(lon, lat, alt)
    return x, y, z

def gps_to_ecef_pyproj(lat, lon, alt):
    ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    x, y, z = pyproj.transform(lla, ecef, lon, lat, alt, radians=False)
    return x, y, z

def wgs2korea5187(lat, lon, alt):
    crs_lla = pyproj.CRS.from_epsg(4326)
    crs_korea = pyproj.CRS.from_epsg(5187)
    transformer = pyproj.Transformer.from_crs(crs_lla, crs_korea, always_xy=True)
    x, y, z = transformer.transform(lon, lat, alt)
    return x, y, z

def extract_positions(bagfile, position_topic, out_filename):
    pn = 0
    positions = []
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages([position_topic]):
            if topic == position_topic:
                x, y, z = wgs2korea5187(msg.latitude, msg.longitude, msg.altitude)
                positions.append([msg.header.stamp.to_sec(), x, y, z])
                pn += 1
    print('position messages {}'.format(pn))
    with open(out_filename, 'w') as f:
        f.write('# timestamp korea5187_p_antenna_x korea5187_p_antenna_y korea5187_p_antenna_z\n')
        for i, p in enumerate(positions):
            f.write('%.9f %.8f %.8f %.8f\n' %
                    (p[0], p[1], p[2], p[3]))
    print('wrote ' + str(pn) + ' position messages to the file: ' + out_filename)

def extract(bagfile, position_topic, orientation_topic, out_filename):
    pn = 0
    rn = 0

    positions = []
    orientations = []
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages([position_topic, orientation_topic]):
            if topic == position_topic:
                # x, y, z = wgs2ecef(msg.latitude, msg.longitude, msg.altitude)
                x, y, z = wgs2korea5187(msg.latitude, msg.longitude, msg.altitude)
                # nx, ny, nz = gps_to_ecef_pyproj(msg.latitude, msg.longitude, msg.altitude)
                # assert abs(x - nx) < 1e-5
                # assert abs(y - ny) < 1e-5
                # assert abs(z - nz) < 1e-5
                positions.append([msg.header.stamp.to_sec(), x, y, z])
                pn += 1
            elif topic == orientation_topic:
                orientations.append([msg.header.stamp.to_sec(), msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
                rn += 1
    print('position messages {}, orientation messages {}'.format(pn, rn))
    # orientation data is much more dense than position data, so we interpolate the orientation data.
    intorientations = []
    j = 0
    for i, p in enumerate(positions):
        while j < len(orientations) and orientations[j][0] < p[0]:
            j += 1
        if j == 0:
            left = 0
            right = 1
        elif j == len(orientations):
            left = len(orientations) - 2
            right = len(orientations) - 1
        else:
            left = j - 1
            right = j
        tl = orientations[left][0]
        tr = orientations[right][0]
        key_times = [tl, tr]
        key_rots = Rotation.from_quat([orientations[left][1:5], orientations[right][1:5]])
        slerp = Slerp(key_times, key_rots)
        rot = slerp([p[0]]).as_quat()[0]
        # print('left time {}, left quat {}, right time {}, right quat {}, time {}, rot {}'.format(
        #     tl, orientations[left][1:5], tr, orientations[right][1:5], p[0], rot))
        intorientations.append([p[0], rot[0], rot[1], rot[2], rot[3]])

    with open(out_filename, 'w') as f:
        f.write('# timestamp korea5187_p_antenna_x korea5187_p_antenna_y korea5187_p_antenna_z '
                'MagNorth_q_MTi_x MagNorth_q_MTi_y MagNorth_q_MTi_z MagNorth_q_MTi_w\n')
        for i, p in enumerate(positions):
            f.write('%.9f %.8f %.8f %.8f %.9f %.9f %.9f %.9f\n' %
                    (p[0], p[1], p[2], p[3],
                     intorientations[i][1], intorientations[i][2], intorientations[i][3], intorientations[i][4]))
    print('wrote ' + str(pn) + ' pose messages to the file: ' + out_filename)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Extracts IMU messages from bagfile.
    ''')
    parser.add_argument('folder', help='The folder containing all bagfiles')
    parser.add_argument('--position_topic', help='position topic of RTK GPS', default='/fix')
    parser.add_argument('--rotation_topic', help='rotation topic of MTI630 IMU', default='/imu/data')
    args = parser.parse_args()
    # recursively find all bags in args.folder
    allbags = []
    for root, dirs, files in os.walk(args.folder):
        for file in files:
            if file.endswith(".bag"):
                allbags.append(os.path.join(root, file))
    # print bags
    for i, bag in enumerate(allbags):
        print('{}: {}'.format(i, bag))

    for bag in allbags:
        print('processing {}'.format(bag))
        outputtextfile = bag[:-4] + '_gt.txt'
        extract(bag, args.position_topic, args.rotation_topic, outputtextfile)
        outputfile = bag[:-4] + '_zed_f9p_gps.txt'
        extract_positions(bag, "/ublox_gps/fix", outputfile)
