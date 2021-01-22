#!/usr/bin/env python
import math
import warnings

import cv2
import numpy as np

import largestComponent

SENTINEL = 1000  # sentinel for invalid circle center coordinate.

def nearestNeighbors(trainData, k=3):
    """
    find the nearest neighbors for each element of trainData.
    :param trainData: e.g. np.random.randint(0,100,(25,2)).astype(np.float32)
    :return: the k nearest neighbors of each element of trainData.
    [[(angle, dist, index in trainData), ...], ...]
    """

    length = trainData.shape[0]
    # Label each one with indices
    responses = np.arange(0, length).astype(np.float32)
    knn = cv2.ml.KNearest_create()
    knn.train(trainData, cv2.ml.ROW_SAMPLE, responses)

    neighbor_array = []
    for newcomer in trainData:
        query = np.reshape(newcomer, (1, 2))
        ret, results, neighbors, dist = knn.findNearest(query, k+1)
        neighbor_array.append(list(map(int, neighbors[0, 1:]))) # exclude itself.
    return neighbor_array


def relativeAngleAndDistance(a, b):
    dxy = b - a
    angle = math.atan2(dxy[1], dxy[0])
    return angle, np.linalg.norm(dxy)


def isCloseToInteger(val, tol):
    return abs(val - round(val)) < tol


def assignCoordinatesToPoints(circle_centers, expected_circle_dist):
    """
    Find the largest circle group, mark coordinates starting from its element closest to the origin.
    Determine the horizontal and vertical dx, dy.
    For other non marked circles, use dx, dy to compute their coordinates.
    :param circle_centers: each element is an OpenCV Point(x, y) x column, y row
    x from left to right, y from top to bottom
    :param expected_circle_dist: expected circle distance
    :return: coordinates for each circle's center, will be set to sentinel if not computed.
      dxy [delta x, delta y] average circle center distance in pixels, x for columns, y for rows.
    """
    angle_tolerance = 10 * math.pi / 180  # the allowed angle uncertainty in checking if two circles
    # satisfy the horizontal, vertical or diagonal relative positions.
    dist_tolerance = 10  # the uncertainty allowed in checking if two circles satisfy the expected distance.
    round_tolerance = 0.2  # allowed uncertainty in checking if a coordinate is close to an integer.

    expected_diagonal_dist = expected_circle_dist * math.sqrt(2)

    neighbor_list = nearestNeighbors(circle_centers, 4)

    # find largest connected component of circles
    numcircles = circle_centers.shape[0]
    g = largestComponent.Graph(numcircles)
    for index, neighbors in enumerate(neighbor_list):
        for neighbor in neighbors:
            dxy = circle_centers[neighbor, :] - circle_centers[index, :]
            if np.linalg.norm(dxy) < expected_diagonal_dist + dist_tolerance:
                g.addEdge(index, neighbor)
    cc = g.connectedComponents()
    maxcomponent_index = np.argmax([len(c) for c in cc])
    maxcomponent = cc[maxcomponent_index]
    remaining_circles = []
    for index, component in enumerate(cc):
        if index == maxcomponent_index:
            continue
        remaining_circles.extend(component)

    # find the circle center closest to the origin
    dist = []
    for c in maxcomponent:
        dist.append(np.linalg.norm(circle_centers[c, :]))
    seed = np.min(maxcomponent)

    # mark the circles in the largest component with nominal coordinates
    coordinates = np.full((numcircles, 2), SENTINEL, dtype=int)
    coordinates[seed, :] = 0
    visited = np.full((numcircles, 1), False, dtype=bool)
    queue = [seed]
    while len(queue):
        c = queue.pop()
        visited[c] = True
        for neighbor in neighbor_list[c]:
            neighbor_updated = False
            angle, dist = relativeAngleAndDistance(circle_centers[c, :], circle_centers[neighbor, :])
            delta = np.array([SENTINEL, SENTINEL])
            if abs(dist - expected_circle_dist) < dist_tolerance:
                if - angle_tolerance < angle < angle_tolerance:
                    delta = np.array([1, 0])
                if math.pi * 0.5 - angle_tolerance < angle < math.pi * 0.5 + angle_tolerance:
                    delta = np.array([0, 1])
                if math.pi - angle_tolerance < angle or angle < - math.pi + angle_tolerance:
                    delta = np.array([-1, 0])
                if - math.pi * 0.5 - angle_tolerance < angle < - math.pi * 0.5 + angle_tolerance:
                    delta = np.array([0, -1])
            elif abs(dist - expected_diagonal_dist) < dist_tolerance:
                if - angle_tolerance < angle - math.pi * 0.25 < angle_tolerance:
                    delta = np.array([1, 1])
                if - angle_tolerance < angle - math.pi * 0.75 < angle_tolerance:
                    delta = np.array([-1, 1])
                if - angle_tolerance < angle + math.pi * 0.25 < angle_tolerance:
                    delta = np.array([1, -1])
                if - angle_tolerance < angle + math.pi * 0.75 < angle_tolerance:
                    delta = np.array([-1, -1])

            if delta[0] != SENTINEL:
                if coordinates[c, 0] != SENTINEL:
                    expected_coord = coordinates[c, :] + delta
                    if coordinates[neighbor, 0] != SENTINEL:
                        if coordinates[neighbor, 0] != expected_coord[0] or \
                                coordinates[neighbor, 1] != expected_coord[1]:
                            msg = "neighbor {} of coordinates {} disagrees with computed coordinates {}. " \
                                  "Try to adjust the circle distance!".\
                                format(neighbor, coordinates[neighbor], expected_coord)
                            warnings.warn(msg)
                            return np.full((numcircles, 2), SENTINEL, dtype=int)
                    else:
                        coordinates[neighbor, :] = expected_coord
                        queue.append(neighbor)
                        neighbor_updated = True
                else:
                    if coordinates[neighbor, 0] != SENTINEL:
                        expected_coord = coordinates[neighbor, :] - delta
                        coordinates[c, :] = expected_coord
                        queue.append(c)
                    else:
                        pass

            if not visited[neighbor] and not neighbor_updated:
                queue.append(neighbor)

    # compute refined dx and dy based on labeled circles
    delta_samples = [[], []]
    for c in maxcomponent:
        for neighbor in neighbor_list[c]:
            if coordinates[c, 0] != SENTINEL and coordinates[neighbor, 0] != SENTINEL:
                for j in range(0, 2):
                    dc = coordinates[c, j] - coordinates[neighbor, j]
                    if dc == 0:
                        delta_samples[j].append(0)
                    else:
                        delta_samples[j].append((circle_centers[c, j] - circle_centers[neighbor, j]) / dc)

    dxy = [np.median(samples) for samples in delta_samples]
    print('Refined delta at x y: {}, both should be close to the given circle distance.'.format(dxy))
    if abs(np.linalg.norm(dxy) - expected_diagonal_dist) >= dist_tolerance:
        warnings.warn('Refined delta at x y: {} are a bit off from {}. '
                      'Try to adjust the circle distance.'.
                      format(dxy, expected_circle_dist))
        return coordinates, dxy

    # mark the remaining circles by calculation with the adjusted dx, dy
    # 1. compute the center of the largest component
    center_pixels = np.array([0, 0], dtype=np.float64)
    center_coordinates = np.array([0, 0], dtype=np.float64)
    valid_coordinates = 0
    for c in maxcomponent:
        if coordinates[c, 0] != SENTINEL:
            center_pixels += circle_centers[c, :]
            center_coordinates += coordinates[c, :]
            valid_coordinates += 1
    if valid_coordinates == 0:
        warnings.warn('Unable to find valid coordinates for circles in the max component.'
                      ' Try to adjust the circle distance!')
        return coordinates, dxy

    center_pixels /= valid_coordinates
    center_coordinates /= valid_coordinates

    # 2. assign coordinates to every circle outside of the largest component
    # according to their distance to the gravity center of the largest component.
    for circle_index in remaining_circles:
        delta_pixels = circle_centers[circle_index, :] - center_pixels
        delta_coordinates = np.divide(delta_pixels, dxy)
        circle_coord = center_coordinates + delta_coordinates
        if isCloseToInteger(circle_coord[0], round_tolerance) and \
                isCloseToInteger(circle_coord[1], round_tolerance):
            coordinates[circle_index, 0] = round(circle_coord[0])
            coordinates[circle_index, 1] = round(circle_coord[1])

    return coordinates, dxy


def drawCoordinates(circles, coordinates, img):
    """
    draw coordinates of circles on the image, (x, y), x for column, y for row.
    :param circles:
    :param coordinates:
    :param img:
    :return:
    """
    circle_img = img.copy()
    d_red = (65, 55, 150)
    l_red = (200, 200, 250)
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.5
    thickness = 1
    for index, f in enumerate(circles):
        cv2.circle(circle_img, (int(f.pt[0]), int(f.pt[1])), int(f.size / 2), d_red, 2, cv2.LINE_AA)
        cv2.circle(circle_img, (int(f.pt[0]), int(f.pt[1])), int(f.size / 2), l_red, 1, cv2.LINE_AA)
        cv2.putText(circle_img, '{:.0f},{:.0f}'.format(coordinates[index, 0], coordinates[index, 1]),
                    (int(f.pt[0]), int(f.pt[1])), font, fontScale, (0, 255, 0), thickness, cv2.LINE_AA)

    h, w = img.shape[:2]
    vis = np.zeros((h, w * 2 + 5), np.uint8)
    vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
    vis[:h, :w] = img
    vis[:h, w + 5:w * 2 + 5] = circle_img

    cv2.imshow("image", vis)
    # cv2.imwrite("coordinates.jpg", circle_img)
    cv2.waitKey()
    cv2.destroyAllWindows()
