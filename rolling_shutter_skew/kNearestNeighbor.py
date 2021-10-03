#!/usr/bin/env python
import math
import warnings

import cv2
import numpy as np

import largestComponent

SENTINEL = 1000  # sentinel for invalid circle center coordinate.

def nearestNeighbors(trainData, k=3, cutoffratio=0.7):
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

    neighbor_list = []
    distance_list = []
    for newcomer in trainData:
        query = np.reshape(newcomer, (1, 2))
        ret, results, neighbors, distances = knn.findNearest(query, k+1)
        # distances are squares of Euclidean distances in ascending order.
        lastindex = len(distances[0, :])
        for index, d in enumerate(distances[0, :]):
            if d * cutoffratio * cutoffratio > distances[0, 1]: # 1 to exclude itself.
                lastindex = index
                break

        neighbor_list.append(list(map(int, neighbors[0, 1:lastindex])))
        distance_list.append([math.sqrt(x) for x in distances[0, 1:lastindex]])
    return neighbor_list, distance_list


def relativeAngleAndDistance(a, b):
    dxy = b - a
    angle = math.atan2(dxy[1], dxy[0])
    return angle, np.linalg.norm(dxy)


def isCloseToInteger(val, tol):
    return abs(val - round(val)) < tol


def assignCoordinatesToPoints(circle_centers, expected_circle_dist=-1, cutoffratio=0.7):
    """
    Assign coordinates to points.
    First find the largest connected component, then choose the point closest to its gravity center as the origin,
    and assign coordinates to points in the component.
    For other points not in the largest component, use estimated point distance to compute their coordinates.
    :param circle_centers: {(x, y)} typically in pixel units. x axis is from left to right, y axis from top to bottom.
    :param expected_circle_dist: expected circle distance. If -1, it will be estimated.
    :param cutoffratio: in selecting neighbors for a point, a candidate will be dismissed if
    its distance x cutoffratio > the nearest neighbor's distance.
    :return: coordinates for circle centers. A point's coordinates will be set to sentinel if
        coordinate determination fails.
        Also return the average circle distance.
    """
    angle_tolerance = 10 * math.pi / 180  # the allowed angle uncertainty in checking if two circles
    # satisfy the horizontal, vertical or diagonal relative positions.
    dist_tolerance = 10  # the uncertainty allowed in checking if two circles satisfy the expected distance.
    round_tolerance = 0.2  # allowed uncertainty in checking if a coordinate is close to an integer.
    led_panel_cols = 10

    neighbor_list, distance_list = nearestNeighbors(circle_centers, 4, cutoffratio)

    # estimate circle distance
    if expected_circle_dist <= 0:
        alldistances = []
        for distances in distance_list:
            alldistances.extend(distances)
        expected_circle_dist = np.median(alldistances)
        print("Estimated circle center distance in pixels {:.2f}".format(expected_circle_dist))
    expected_diagonal_dist = expected_circle_dist * math.sqrt(2)

    # curate neighbors based on distance
    culled_neighbor_list = []
    culled_distance_list = []
    for index, neighbors in enumerate(neighbor_list):
        culled_neighbors = []
        culled_distances = []
        for nid, neig in enumerate(neighbors):
            if distance_list[index][nid] < expected_diagonal_dist + dist_tolerance:
                culled_neighbors.append(neig)
                culled_distances.append(distance_list[index][nid])
        culled_neighbor_list.append(culled_neighbors)
        culled_distance_list.append(culled_distances)

    # find largest connected component of circles
    numcircles = circle_centers.shape[0]
    g = largestComponent.Graph(numcircles)
    for index, neighbors in enumerate(culled_neighbor_list):
        for neighbor in neighbors:
                g.addEdge(index, neighbor)
    cc = g.connectedComponents()
    maxcomponent_index = np.argmax([len(c) for c in cc])
    maxcomponent = cc[maxcomponent_index]
    remaining_circles = []
    for index, component in enumerate(cc):
        if index == maxcomponent_index:
            continue
        remaining_circles.extend(component)

    # find the circle center closest to the gravity center of the maximum component
    mc_points = []
    for c in maxcomponent:
        mc_points.append(circle_centers[c, :])
    mc_center = np.mean(mc_points, axis=0)
    distances = []
    for c in maxcomponent:
        distances.append(np.linalg.norm(circle_centers[c, :] - mc_center))

    seed_mc_index = np.argmin(distances)
    seed_index = maxcomponent[seed_mc_index]

    # mark the circles in the largest component with nominal coordinates
    coordinates = np.full((numcircles, 2), SENTINEL, dtype=int)
    coordinates[seed_index, :] = 0
    visited = np.full((numcircles, 1), False, dtype=bool)
    queue = [seed_index]
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

    # mark the remaining circles by calculation with the center distance
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
        return coordinates, expected_circle_dist

    center_pixels /= valid_coordinates
    center_coordinates /= valid_coordinates

    # 2. assign coordinates to every circle outside of the largest component
    # according to their distance to the gravity center of the largest component.
    for circle_index in remaining_circles:
        delta_pixels = circle_centers[circle_index, :] - center_pixels
        delta_coordinates = np.divide(delta_pixels, expected_circle_dist)
        circle_coord = center_coordinates + delta_coordinates
        if isCloseToInteger(circle_coord[0], round_tolerance) and \
                isCloseToInteger(circle_coord[1], round_tolerance) and \
                abs(round(circle_coord[0])) <= led_panel_cols and \
                abs(round(circle_coord[1])) <= led_panel_cols:
            coordinates[circle_index, 0] = round(circle_coord[0])
            coordinates[circle_index, 1] = round(circle_coord[1])
        else:
            print("Coordinate not close to integer {}, {}".format(circle_coord[0], circle_coord[1]))

    return coordinates, expected_circle_dist


def boundingRect(coordinates):
    """

    :param coordinates:
    :return: [top, left, bottom, right]
    """
    xcoords = coordinates[:, 0]
    ycoords = coordinates[:, 1]
    xcoords.sort()
    ycoords.sort()
    left = xcoords[0]
    top = ycoords[0]
    right = xcoords[-1]
    bottom = ycoords[-1]
    if right == SENTINEL:
        right = xcoords[-2]
    if bottom == SENTINEL:
        bottom = ycoords[-2]
    return [top, left, bottom, right]


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
