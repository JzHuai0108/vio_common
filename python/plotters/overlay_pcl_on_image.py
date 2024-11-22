# given a pcd file and an image
# project the points in the pcd onto the image to visualize the calibration accuracy.
# Each point follows p = pi(C_T_L P^L, K, D)

# Steps
# load pcd file at pcdfile
# for each point in the pcd
# transform it and then project it to the image
# load image at image
# plot the points on the image
# save the resulting image


import os
import numpy as np
import open3d as o3d
import cv2

def load_pcd(pcdfile):
    """Load PCD file and return points as a numpy array."""
    pcd = o3d.io.read_point_cloud(pcdfile)
    return np.asarray(pcd.points), np.asarray(pcd.colors)

def transform_points(points, transform):
    """Apply a 4x4 transformation matrix to a set of 3D points."""
    points_h = np.hstack((points, np.ones((points.shape[0], 1))))  # Convert to homogeneous coordinates
    transformed_points_h = (transform @ points_h.T).T
    return transformed_points_h[:, :3]  # Return as 3D points

def project_points(points, K, D):
    """Project 3D points onto the image plane using pinhole camera model with distortion."""
    # Intrinsics
    fx, fy, cx, cy = K

    # Normalize points
    x = points[:, 0] / points[:, 2]
    y = points[:, 1] / points[:, 2]

    # Apply radial tangential distortion
    r2 = x**2 + y**2
    k1, k2, p1, p2 = D
    x_distorted = x * (1 + k1 * r2 + k2 * r2**2) + 2 * p1 * x * y + p2 * (r2 + 2 * x**2)
    y_distorted = y * (1 + k1 * r2 + k2 * r2**2) + p1 * (r2 + 2 * y**2) + 2 * p2 * x * y

    # Convert to pixel coordinates
    u = fx * x_distorted + cx
    v = fy * y_distorted + cy

    return np.vstack((u, v)).T


def plot_points_on_image(image, points_2d, colors, pointsize):
    """
    Overlay points on an image with specified colors.

    Args:
        image (np.ndarray): The input image.
        points_2d (np.ndarray): Projected 2D points on the image (Nx2).
        colors (np.ndarray): Array of colors (Nx3) in BGR format for each point.

    Returns:
        np.ndarray: The image with overlaid points.
    """
    for point, color in zip(points_2d, colors):
        x, y = int(point[0]), int(point[1])
        if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:  # Check if the point is within image bounds
            color_bgr = tuple(map(int, color))  # Ensure color values are integers
            cv2.circle(image, (x, y), radius=pointsize, color=color_bgr, thickness=-1)
    return image

def fake_color(value):
    """
    Maps a float value to a color using a gradient logic similar to a colormap.

    Args:
        value (float): A normalized value (0 to 1) to map to a color.

    Returns:
        tuple: A (B, G, R) tuple representing the color.
    """
    pos_slope = 255 / 60.0
    neg_slope = -255 / 60.0
    value *= 255  # Scale value to the range [0, 255]

    color = np.zeros(3, dtype=np.float32)  # [B, G, R]

    if value < 60:
        color[0] = 255
        color[1] = pos_slope * value
        color[2] = 0
    elif value < 120:
        color[0] = neg_slope * value + 2 * 255
        color[1] = 255
        color[2] = 0
    elif value < 180:
        color[0] = 0
        color[1] = 255
        color[2] = pos_slope * value - 2 * 255
    elif value < 240:
        color[0] = 0
        color[1] = neg_slope * value + 4 * 255
        color[2] = 255
    elif value < 300:
        color[0] = pos_slope * value - 4 * 255
        color[1] = 0
        color[2] = 255
    else:
        color[0] = 255
        color[1] = 0
        color[2] = neg_slope * value + 6 * 255

    # Ensure the color values are clamped to the range [0, 255]
    color = np.clip(color, 0, 255).astype(np.uint8)

    return tuple(color)  # Return as (B, G, R)

def overlay(pcdfile, imagefile, outputfile):
    # we assume the radial tangential distortion for now.
    # resolution: 1280 x 720(h)
    intrinsics = [519.4588780362625, 519.032899533456, 644.395939609842, 362.3814704200585]
    ratio = [1.0, 1.0]
    intrinsics = [intrinsics[0] * ratio[0], intrinsics[1] * ratio[0], intrinsics[2], intrinsics[3]]

    distortion_coeffs = [0.019442915619071716, -0.012026334544560754, -0.0003783904135957078, 0.0005456090834102392]
    pointsize = 1
    # C_T_L is given by C_T_I * I_T_L
    cam_T_livoximu = np.array([
        [-0.9993787053340518, 0.03516807894897209, -0.0023258434756262165, 0.08726182551675578],
        [-0.007733824175670411, -0.2831992241798044, -0.9590299199647404, -0.03528605760568212],
        [-0.034385917007607375, -0.9584160921264812, 0.2832952577516083, 0.010855668614364086],
        [0.0, 0.0, 0.0, 1.0]
    ])
    livoximu_R_lidar = np.array([
        [0.999987525559687, 0.002667093232845, 0.004223190583018],
        [-0.00266998490789, 0.999996204910806, 0.000679223510758],
        [-0.004221363003204, -0.000690490892945, 0.999990851616514]
    ])
    livoximu_p_lidar = np.array([-0.011, -0.02329, 0.04412])
    livoximu_T_lidar = np.eye(4)
    livoximu_T_lidar[:3, :3] = livoximu_R_lidar
    livoximu_T_lidar[:3, 3] = livoximu_p_lidar

    C_T_L = cam_T_livoximu @ livoximu_T_lidar

    # Load PCD and image
    points, intensities = load_pcd(pcdfile) # intensities Nx3 are in range [0, 1], scale to uint8 by 255
    image = cv2.imread(imagefile)

    # Transform points
    transformed_points = transform_points(points, C_T_L)
    distances = np.linalg.norm(transformed_points, axis=1)  # Calculate Euclidean distances

    # Filter points in front of the camera
    mask = transformed_points[:, 2] > 0.01  # Retain points with positive Z (in front of the camera)
    transformed_points = transformed_points[mask]
    intensities = intensities[mask, 0]
    distances = distances[mask]

    points_2d = project_points(transformed_points, intrinsics, distortion_coeffs)

    # Determine coloring method (intensity or distance)
    intensity_color = False  # Set to True for intensity-based coloring
    if intensity_color:
        colors = np.array([fake_color(i) for i in intensities], dtype=np.uint8)  # Normalize intensity for fake_color
    else:
        max_distance = distances.max() if distances.size > 0 else 1.0  # Avoid division by zero
        colors = np.array([fake_color(d / max_distance) for d in distances], dtype=np.uint8)  # Normalize distance for fake_color

    output_image = plot_points_on_image(image, points_2d, colors, pointsize)

    # Save or display the output image
    cv2.imwrite(outputfile, output_image)
    print(f"Result saved to {outputfile}")


