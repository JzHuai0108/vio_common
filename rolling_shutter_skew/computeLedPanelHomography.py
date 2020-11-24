#!/usr/bin/env python
import os
import sys

import cv2
import numpy as np
import findCircles
import kNearestNeighbor
import homography
import VideoManager
import matplotlib.pyplot as plt


def inBins(val, bin_list):
    for bin in bin_list:
        if val >= bin[0] and val < bin[1]:
            return True


def compute_homography_opencv(circles_for_homography, coordinates_for_homography, coord_shift, circle_dist):
    centers = np.zeros((len(circles_for_homography), 2), dtype=np.float32)
    coordinates_scaled = np.zeros((len(circles_for_homography), 2), dtype=np.float32) # for visualizing homography correction.
    coordinates = np.zeros((len(circles_for_homography), 2), dtype=np.float32) # for correcting perspective effect.
    for index, c in enumerate(circles_for_homography):
        centers[index, 0] = c.pt[0]
        centers[index, 1] = c.pt[1]
        coordinates_scaled[index, 0] = (coordinates_for_homography[index, 0] + coord_shift[0] + 10) * circle_dist
        coordinates_scaled[index, 1] = (coordinates_for_homography[index, 1] + coord_shift[1] + 3) * circle_dist
        coordinates[index, 0] = coordinates_for_homography[index, 0] + coord_shift[0]
        coordinates[index, 1] = coordinates_for_homography[index, 1] + coord_shift[1]

    scaled_h, status = cv2.findHomography(centers, coordinates_scaled)
    actual_h, status = cv2.findHomography(centers, coordinates)
    print('OpenCV found actual H\n{}\nscaled H\n{}'.format(actual_h, scaled_h))
    print("Index: Projection(col, row), Error(col, row)")
    for index in range(centers.shape[0]):
        point = np.zeros((3, 1))
        point[0] = centers[index, 0]
        point[1] = centers[index, 1]
        point[2] = 1.0
        projected = np.dot(scaled_h, point)
        projected /= projected[2]
        print('{}:({},{}),({},{})'.format(
            index, projected[0], projected[1],
            projected[0] - coordinates_scaled[index, 0], projected[1] - coordinates_scaled[index, 1]))
    return actual_h, scaled_h


def compute_homography_custom(circles_for_homography, coordinates_for_homography, coord_shift, circle_dist):
    """
    compute homography with custom DLT.
    :param circles_for_homography
    :param coordinates_for_homography
    :param coord_shift shift the coordinates to avoid negatives for better visualization.
    :param circle_dist average horizontal distance between LEDs in pixels.
    :return:
    """
    centers2 = np.zeros((3, len(circles_for_homography)), dtype=np.float32)
    coordinates_scaled2 = np.zeros((3, len(circles_for_homography)), dtype=np.float32)
    coordinates2 = np.zeros((3, len(circles_for_homography)), dtype=np.float32)
    for index, c in enumerate(circles_for_homography):
        centers2[0, index] = c.pt[0]
        centers2[1, index] = c.pt[1]
        centers2[2, index] = 1
        coordinates_scaled2[0, index] = (coordinates_for_homography[index, 0] + coord_shift[0] + 10) * circle_dist
        coordinates_scaled2[1, index] = (coordinates_for_homography[index, 1] + coord_shift[1] + 3) * circle_dist
        coordinates_scaled2[2, index] = 1
        coordinates2[0, index] = coordinates_for_homography[index, 0] + coord_shift[0]
        coordinates2[1, index] = coordinates_for_homography[index, 1] + coord_shift[1]
        coordinates2[2, index] = 1
    scaled_h = homography.H_from_points(centers2, coordinates_scaled2)
    actual_h = homography.H_from_points(centers2, coordinates2)
    print('Custom found actual H\n{}\nscaled H\n{}'.format(actual_h, scaled_h))
    print("Index: Projection(col, row), Error(col, row)")
    for index in range(centers2.shape[1]):
        point = centers2[:, index]
        projected = np.dot(scaled_h, point)
        projected /= projected[2]
        print('{}:({},{}),({},{})'.format(
            index, projected[0], projected[1],
            projected[0] - coordinates_scaled2[0, index], projected[1] - coordinates_scaled2[1, index]))
    return actual_h, scaled_h


def computeLedPanelHomography(video, circle_dist=45, num_accumulated_images = 7):
    """
    compute the homography to remove the perspective effect of the camera viewing 
    the LED panel in rolling shutter mode.
    :param video: video file
    :param circle_dist: the estimated horizontal distance between LED circle centers in pixels
    :param num_accumulated_images: how many images to accumulate detected circles?
    :return:
    """
    output_dir = os.path.dirname(video)
    homography_txt = os.path.join(output_dir, "homography.txt")

    video_manager = VideoManager.VideoManager(video)
    if not video_manager.isOpened():
        video_manager.close()
        print("Failed to open video {}".format(video))
        return
    draw_img = None
    accumulated_circles = None
    for image_index in range(num_accumulated_images):
        _, img = video_manager.nextFrame()
        draw_img = img
        img = findCircles.android_image_to_opencv_image(img)

        circles = findCircles.find_circles_mser(img, draw_result=False)
        if accumulated_circles:
            num_kept_circles = len(accumulated_circles)
            accumulated_circles.extend(circles)
            new_accumulated_circles = [x for x in accumulated_circles if
                                     not findCircles.suppress(accumulated_circles, x)]
            accumulated_circles = new_accumulated_circles
        else:
            accumulated_circles = circles
            num_kept_circles = 0
        num_new_circles = len(accumulated_circles) - num_kept_circles
        print('#New circles {} #Total circles {}'.format(num_new_circles, len(accumulated_circles)))
        centers = np.zeros((len(accumulated_circles), 2), dtype=np.float32)
        for index, c in enumerate(accumulated_circles):
            centers[index, 0] = c.pt[0]
            centers[index, 1] = c.pt[1]
        coordinates = kNearestNeighbor.assignCoordinatesToPoints(centers, circle_dist)
        # kNearestNeighbor.drawCoordinates(accumulated_circles, coordinates, img)

    # filter circles by size percentile,
    radius_list = [circle.size for circle in accumulated_circles]
    num_bins = 10
    binsize, bins, patches = plt.hist(radius_list, num_bins, facecolor='blue', alpha=0.5)
    print('n:{}\nbins:{}\n'.format(binsize, bins))
    # plt.show()

    # pick top 3 bins and get the indices of circles in these bins
    sort_indices = [i[0] for i in sorted(enumerate(binsize), key=lambda x: x[1])]
    numKeptBins = 3
    keep_bin_indices = sort_indices[-numKeptBins:]
    print("Chosen bins {}".format(keep_bin_indices))
    boundaries = []
    circle_count = 0
    for bin_index in keep_bin_indices:
        boundaries.append((bins[bin_index], bins[bin_index + 1]))
        circle_count += binsize[bin_index]

    # copy the circles and coordinates out
    circles_for_homography = []
    coordinates_for_homography = np.zeros((0, 2))
    for index, circle in enumerate(accumulated_circles):
        if inBins(circle.size, boundaries) and coordinates[index, 0] != kNearestNeighbor.SENTINEL and \
                coordinates[index, 1] != kNearestNeighbor.SENTINEL:
            circles_for_homography.append(circle)
            coordinates_for_homography = np.vstack([coordinates_for_homography, coordinates[index, :]])

    assert len(circles_for_homography) <= circle_count, "Chosen circles {} and expected circle count {}".\
        format(len(circles_for_homography), circle_count)
    print("Chosen #circles {}".format(len(circles_for_homography)))

    # draw filtered circles
    kNearestNeighbor.drawCoordinates(circles_for_homography, coordinates_for_homography, draw_img)

    min_coordinates = np.amin(coordinates_for_homography, axis=0)
    coord_shift = min_coordinates
    for index, coord in enumerate(min_coordinates):
        if coord < 0:
            coord_shift[index] = -coord
        else:
            coord_shift[index] = 0
    print('coord shift {}'.format(coord_shift))

    actual_h, scaled_h = compute_homography_opencv(
        circles_for_homography, coordinates_for_homography, coord_shift, circle_dist)

    # compute_homography_custom(circles_for_homography, coordinates_for_homography, coord_shift, circle_dist)

    # Warp image with homography
    im_warp = cv2.warpPerspective(draw_img, scaled_h, (draw_img.shape[1], draw_img.shape[0]))
    h, w = draw_img.shape[:2]
    vis = np.zeros((h, w * 2 + 5), np.uint8)
    vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
    vis[:h, :w] = draw_img
    vis[:h, w + 5:w * 2 + 5] = im_warp
    cv2.imshow("Source (left) Warped image (right)", vis)
    cv2.imwrite("warp.jpg", im_warp)
    cv2.waitKey(0)

    with open(homography_txt, "a") as stream:
        stream.write("#Homography computed with image from {} to {} with #correspondences {}\n".format(
            0, num_accumulated_images-1, len(circles_for_homography)))
        for row in actual_h:
            stream.write('{}\n'.format(' '.join(map(str, row))))
        print('Homography saved to {}'.format(homography_txt))

    video_manager.close()


def main():
    if len(sys.argv) < 2:
        print("Usage:{} video-of-LED-panel".format(sys.argv[0]))
        exit(1)
    script, video = sys.argv
    computeLedPanelHomography(video)


if __name__ == '__main__':
    main()
    


