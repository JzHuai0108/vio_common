# !/usr/bin/python
import os
import rosbag
import argparse
import math
import numpy as np
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

def wgs2utm52n(lat, lon, alt):
    crs_lla = pyproj.CRS.from_epsg(4326)
    crs_korea = pyproj.CRS.from_epsg(32652)
    transformer = pyproj.Transformer.from_crs(crs_lla, crs_korea, always_xy=True)
    e, n, u = transformer.transform(lon, lat, alt)
    return e, n, u

def extract_positions(bagfile, position_topic, out_filename):
    pn = 0
    positions = []
    geographiccoords = []
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages([position_topic]):
            if topic == position_topic:
                geographiccoords.append([msg.header.stamp.to_sec(), msg.latitude, msg.longitude, msg.altitude])
                e, n, u = wgs2utm52n(msg.latitude, msg.longitude, msg.altitude)
                positions.append([msg.header.stamp.to_sec(), e, n, u])
                pn += 1
    print('position messages {}'.format(pn))
    with open(out_filename, 'w') as f:
        f.write('#timestamp,utm52s_p_antenna_e,utm52s_p_antenna_n,utm52s_p_antenna_u,lat,lon,wgs84_ellipsoid_height_antenna\n')
        for i, p in enumerate(positions):
            f.write('%.9f,%.8f,%.8f,%.8f,%.9f,%.9f,%.5f\n' %
                    (p[0], p[1], p[2], p[3], geographiccoords[i][1], geographiccoords[i][2], geographiccoords[i][3]))
    print('wrote ' + str(pn) + ' position messages to the file: ' + out_filename)

def extract(bagfile, position_topic, orientation_topic, out_filename):
    pn = 0
    rn = 0

    B_p_ant = np.array([-0.38, 0, 0.0])
    positions = []
    orientations = []
    geographiccoords = []
    positions_body = []
    imudata = []
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages([position_topic, orientation_topic]):
            if topic == position_topic:
                geographiccoords.append([msg.header.stamp.to_sec(), msg.latitude, msg.longitude, msg.altitude])
                e, n, u = wgs2utm52n(msg.latitude, msg.longitude, msg.altitude)
                positions.append([msg.header.stamp.to_sec(), e, n, u])
                pn += 1
            elif topic == orientation_topic:
                imudata.append([msg.header.stamp.to_sec(), msg.angular_velocity.x,
                                msg.angular_velocity.y, msg.angular_velocity.z])
                quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
                orientations.append([msg.header.stamp.to_sec(), quat[0], quat[1], quat[2], quat[3]])
                rn += 1
    print('position messages {}, orientation messages {}'.format(pn, rn))
    # orientation data is much more dense than position data, so we interpolate the orientation data.
    intorientations = []
    j = 0
    for i, p in enumerate(positions):
        while j < len(orientations) and orientations[j][0] < p[0]:
            j += 1
        if j == 0:
            rot = orientations[0][1:5]
            intorientations.append([p[0], rot[0], rot[1], rot[2], rot[3]])
        elif j == len(orientations):
            rot = orientations[-1][1:5]
            intorientations.append([p[0], rot[0], rot[1], rot[2], rot[3]])
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

    # estimate the yaw between the body frame and the IMU frame by least squares method.
    A = []
    b = []
    step = 3
    imuid = 0

    for i, p in enumerate(positions):
        if i >= len(positions) - step:
            continue
        while imuid < len(imudata) and imudata[imuid][0] < p[0]:
            imuid += 1
        if imuid == len(imudata):
            break
        if imudata[imuid][0] - p[0] > 0.005:
            continue
        if abs(imudata[imuid][3]) > 0.006:
            continue

        quat = intorientations[i][1:5]
        R = Rotation.from_quat(quat).as_matrix()
        A.append([R[0, 0], -R[1, 0]])
        A.append([R[1, 0], R[0, 0]])
        vE = np.array([positions[i + step][1] - positions[i][1],
                       positions[i + step][2] - positions[i][2],
                       positions[i + step][3] - positions[i][3]])
        vE = vE / np.linalg.norm(vE)
        b.append(vE[0])
        b.append(vE[1])
    print("selected {} out of {} position samples for yaw estimation".format(len(A) // 2, len(positions)))
    x = np.linalg.lstsq(A, b, rcond=None)[0]

    nx = np.linalg.norm(x)
    ux = x / nx
    ctheta = ux[0]
    stheta = ux[1]
    print('solved params {}, norm {}, estimated theta {}'.format(x, nx, np.arccos(ctheta) * 180 / np.pi))
    E_R_W = np.array([[ctheta, -stheta, 0], [stheta, ctheta, 0], [0, 0, 1]])
    E_q_W = Rotation.from_matrix(E_R_W)

    for i, p in enumerate(positions):
        W_q_B = Rotation.from_quat(intorientations[i][1:5])
        E_q_B = E_q_W * W_q_B
        quat = E_q_B.as_quat()
        intorientations[i][1:5] = quat
        E_p_ant = np.array([p[1], p[2], p[3]])
        E_p_B = E_p_ant - E_q_B.apply(B_p_ant)
        positions_body.append([p[0], E_p_B[0], E_p_B[1], E_p_B[2]])

    with open(out_filename, 'w') as f:
        f.write('#timestamp,utm52s_p_antenna_e,utm52s_p_antenna_n,utm52s_p_antenna_u,'
                'LocalENU_q_MTi_x,LocalENU_q_MTi_y,LocalENU_q_MTi_z,LocalENU_q_MTi_w,lat_antenna,lon_antenna,wgs84_ellipsoid_height_antenna\n')
        for i, p in enumerate(positions_body):
            f.write('%.9f,%.8f,%.8f,%.8f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.5f\n' %
                    (p[0], p[1], p[2], p[3],
                     intorientations[i][1], intorientations[i][2], intorientations[i][3], 
                     intorientations[i][4], geographiccoords[i][1], geographiccoords[i][2], geographiccoords[i][3]))
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

    # we tried to align the IMU orientation to the RTK GPS trajectories,
    # but the magnetic declination compensation does not work well,
    # so we use the least squares to estimate the magnetic declination.
    magneticdeclination = 0 # -9.09 * math.pi / 180.0 is from https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#declination
    for bag in allbags:
        print('processing {}'.format(bag))
        outputtextfile = bag[:-4] + '_gt.txt'
        extract(bag, args.position_topic, args.rotation_topic, outputtextfile)
        outputfile = bag[:-4] + '_zed_f9p_gps.txt'
        extract_positions(bag, "/ublox_gps/fix", outputfile)
