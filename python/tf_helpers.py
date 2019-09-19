#!/usr/bin/env python

'''
This module is copied from https://github.com/segwayrobotics/segway_DRIVE_benchmark/blob/master/scripts/tf_helpers.py
This module includes functions to manipulate ros transforms

As rospy depends on python, this module works with only python2.
'''

from __future__ import print_function

import math
import warnings
import numpy as np


import tf


def multiply_inverse(T_AB, T_CB):  # pylint: disable=invalid-name
  '''return T_AB*(T_CB)^(-1), each is a list of [tx,y,z, qx,y,z,w]'''
  mat_AB = transformtransformation(T_AB)  # pylint: disable=invalid-name
  mat_CB = transformtransformation(T_CB)  # pylint: disable=invalid-name

  mat_BC = np.linalg.pinv(mat_CB)  # pylint: disable=invalid-name
  mat_AC = np.dot(mat_AB, mat_BC)  # pylint: disable=invalid-name

  # go back to quaternion and 3x1 arrays
  return transformtransformation(mat_AC)


def left_multiply_transform(rostime_txyz_qxyzw_list, mat_multiplier):
  """
  left multiply a list of transforms by a transform

  :param rostime_txyz_qxyzw_list: a list of timed transforms, each transform
      [rostime tx ty tz qx qy qz qw] or
      ["time", "tx", "ty", "tz", "qx", "qy", "qz", "qw"]
  :param mat_multiplier: a numpy 4x4 array representing a transform
  :return: resulting list of timed transforms, each transform
      [rostime (or "time") tx ty tz qx qy qz qw]
  """
  res_list = []
  for row in rostime_txyz_qxyzw_list:
    mat_T_GA = transformtransformation(row[1:8])  # pylint: disable=invalid-name
    mat_T_WA = np.dot(mat_multiplier, mat_T_GA)  # pylint: disable=invalid-name
    res_list.append([row[0]] + transformtransformation(mat_T_WA))
  return res_list


def right_multiply_transform(rostime_txyz_qxyzw_list, mat_multiplier):
  """
  right multiply a list of transforms by a transform

  :param rostime_txyz_qxyzw_list: a list of timed transforms, each transform
      [rostime tx ty tz qx qy qz qw] or
      ["time", "tx", "ty", "tz", "qx", "qy", "qz", "qw"]
  :param mat_multiplier: a numpy 4x4 array representing a transform
  :return: resulting list of timed transforms, each transform
      [rostime (or "time") tx ty tz qx qy qz qw]
  """
  res_list = []
  for row in rostime_txyz_qxyzw_list:
    mat_T_GA = transformtransformation(row[1:8])  # pylint: disable=invalid-name
    mat_T_GB = np.dot(mat_T_GA, mat_multiplier)  # pylint: disable=invalid-name
    res_list.append([row[0]] + transformtransformation(mat_T_GB))
  return res_list


def transformtransformation(something):
  """
  transform between [tx,y,z,qx,y,z,w] and T4x4
  :param something: [tx,y,z,qx,y,z,w], ['tx','y','z','qx','y','z','w'],
      or np.array T4x4
  :return: np.array T4X4, np.array T4X4, or [tx,y,z,qx,y,z,w]
  """
  if len(something) == 7:
    txyzqxyzw = something
    trans = txyzqxyzw[:3]
    rot = txyzqxyzw[3:]
    trans_mat = tf.transformations.translation_matrix(trans)
    rot_mat = tf.transformations.quaternion_matrix(rot)
    return np.dot(trans_mat, rot_mat)
  elif len(something) == 4:
    T4x4 = something  # pylint: disable=invalid-name
    rot2 = tf.transformations.quaternion_from_matrix(T4x4)
    trans2 = tf.transformations.translation_from_matrix(T4x4)
    return list(np.hstack((trans2, rot2)))
  else:
    raise Exception('Pass something of len %d to transformtransformation'
                    % len(something))


def measure_difference(T_AC):  # pylint: disable=invalid-name
  """
  compute the distance and rotation about z axis from a SE(3) transform
  :param T_AC: a SE(3) transform
  :return: norm of translation and rotation angle about z
  """
  displacement = np.linalg.norm(T_AC[:3])
  arr = np.array(T_AC)
  arr_normalized = arr[3:] / np.linalg.norm(arr[3:])
  rotangle = math.acos(arr_normalized[3])*2
  return displacement, rotangle


def find_centroid_brute_force(all_T_GW, minexpectedinlierratio=0.5):  # pylint: disable=invalid-name
  '''find the 'median' of all_T_GW, which is a list of [tx,y,z, qx,y,z,w]
  of float type'''
  length = len(all_T_GW)
  distaccumulator = np.full((length, length), 0.0, dtype=float)
  angleaccumulator = np.full((length, length), 0.0, dtype=float)
  if length < 3:
    return -1, np.full((length), 1e6, dtype=float), \
           np.full((length), 16, dtype=float)

  mindistangle = [1e6, 8]
  maxdistangle = [0, 0]
  for index in range(length):
    for jude in range(index+1, length, 1):

      apple, banana = measure_difference(multiply_inverse(all_T_GW[index],
                                                          all_T_GW[jude]))
      distaccumulator[index, jude] = apple
      angleaccumulator[index, jude] = banana
      distaccumulator[jude, index] = distaccumulator[index, jude]
      angleaccumulator[jude, index] = angleaccumulator[index, jude]
      if distaccumulator[index, jude] < mindistangle[0]:
        mindistangle[0] = distaccumulator[index, jude]
      if angleaccumulator[index, jude] < mindistangle[1]:
        mindistangle[1] = angleaccumulator[index, jude]
      if distaccumulator[index, jude] > maxdistangle[0]:
        maxdistangle[0] = distaccumulator[index, jude]
      if angleaccumulator[index, jude] > maxdistangle[1]:
        maxdistangle[1] = angleaccumulator[index, jude]

  # count inliers
  refthresholds = [0.2, 0.087]
  scales = [1.44, 4, 9, 25, 64, 125, 296]
  foundcentroid = False
  val = -1
  ind = -1
  for scale in scales:
    votebin = np.full((length, 1), 0, dtype=int)
    for index in range(length):
      for jude in range(index+1, length, 1):
        if distaccumulator[index, jude] < scale*refthresholds[0] and\
            angleaccumulator[index, jude] < math.sqrt(scale)*refthresholds[1]:
          votebin[index] += 1
          votebin[jude] += 1
    val = np.max(votebin, axis=None)
    ind = np.argmax(votebin, axis=None)
    if float(val) > length*minexpectedinlierratio:
      message = 'Found centroid with dist and angle threshold %.4f, %.4f of ' \
                'scale %.3f of external supports %d' % \
                (scale*refthresholds[0], math.sqrt(scale)*refthresholds[1],
                 scale, val)
      print(message)  # pylint: disable=superfluous-parens
      print('Estimated T_GW is', '_'.join([str(x) for x in all_T_GW[ind]]))  # pylint: disable=superfluous-parens
      foundcentroid = True
      break
  if not foundcentroid:
    # in this case, -1 entry is returned anyway
    warnings.warn('Failed to find a centroid with min dist and angle %.4f,'
                  ' %.4f' % (scales[-1]*refthresholds[0],
                             math.sqrt(scales[-1])*refthresholds[1]))

  return ind, distaccumulator[ind, :], angleaccumulator[ind, :]


def to_yaw_angle_quat(qxyzw):
  """

  :param qxyzw: a list or numpy array
  :return: a float angle [-pi, pi)
  """
  qxyzw = qxyzw / np.linalg.norm(qxyzw)
  if qxyzw[2] < 0:
    theta = math.acos(-qxyzw[3])*2
  else:
    theta = math.acos(qxyzw[3])*2
  if theta > math.pi:
    theta -= 2*math.pi
  return theta


def to_yaw_angle(mat_T_AB): # pylint: disable=invalid-name
  """
  find the yaw angle
  :param mat_T_AB: a 4x4 numpy array
  :return:
  """
  return to_yaw_angle_quat(
      tf.transformations.quaternion_from_matrix(mat_T_AB))


def to_xy_theta(mat_T_AB): # pylint: disable=invalid-name
  """
  find x, y, and yaw angle
  :param mat_T_AB: a 4x4 numpy array
  :return:
  """
  return [mat_T_AB[0, 3], mat_T_AB[1, 3], to_yaw_angle(mat_T_AB)]
