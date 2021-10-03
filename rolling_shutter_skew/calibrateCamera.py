import cv2
assert cv2.__version__[0] == '3', 'The fisheye module requires opencv version >= 3.0.0'
import numpy as np


def calibrate_camera(objp, corners, img_shape):
    """

    :param objp: 3d point in real world space, 1xNx3,
    :param corners: 2d points in image plane, 1xNx2,
    :param img_shape: (w, h)
    :return: K 3X3 D 4x1, R_CW
    """
    objpoints = []
    imgpoints = []
    objpoints.append(objp)
    imgpoints.append(corners)

    N_OK = len(objpoints)
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

    # fisheye camera calibration does not work well.
    # calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
    # rms, K, D, rvecs, tvecs = \
    #     cv2.fisheye.calibrate(
    #         objpoints,
    #         imgpoints,
    #         img_shape,
    #         K, D, rvecs, tvecs,
    #         calibration_flags,
    #         (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    #     )

    calibration_flags = cv2.CALIB_ZERO_TANGENT_DIST
    rms, K, D, rvecs, tvecs = \
        cv2.calibrateCamera(
            objpoints,
            imgpoints,
            img_shape,
            calibration_flags,
            (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        )
    R_CW, _ = cv2.Rodrigues(rvecs[0])
    t_CW = tvecs[0]
    print("Found " + str(N_OK) + " valid images for calibration")
    print("DIM=" + str(img_shape))
    print("K=np.array(" + str(K.tolist()) + ")")
    print("D=np.array(" + str(D.tolist()) + ")")
    return K, D, R_CW, t_CW


def undistort(K, D, R_CW, image):
    """

    :param K:
    :param D:
    :param image
    :return:
    """
    h, w = image.shape[:2]

    map1, map2 = cv2.initUndistortRectifyMap(K, D, R_CW.transpose(), K, (w, h), cv2.CV_16SC2)

    undistorted_img = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    vis = np.zeros((h, w * 2 + 5), np.uint8)
    vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
    vis[:h, :w, :] = image
    vis[:h, w + 5:w * 2 + 5, :] = undistorted_img

    dh = 60
    for i in range(1, h // dh - 1):
        vis[dh * i -1:dh * i+1, :, 0] = 255
    for i in range(1, (w * 2 + 5) // dh - 1):
        vis[:, dh * i - 1:dh * i + 1, 0] = 255

    # cv2.imshow("Undistorted image (right)", vis)
    # cv2.waitKey()
    # cv2.destroyAllWindows()

    return undistorted_img


def projectPoints(coordinates, R_CW, t_CW, K):
    """
    project world points to the rectified camera.
    The original camera frame is the frame at which the camera takes the LED panel image.
    The world frame is the LED light frame, right, down, forward
    The rectified camera frame is the camera frame (right down forward) aligned with the world frame
    and have the same origin as the original camera frame in the world frame.
    :param coordinates: world point coordinates
    :param R_CW: Corig_R_W
    :param t_CW: Corig_t_W
    :param K: Intrinsic matrix
    :return: projected points in the rectified camera
    """
    W_t_Corig = np.matmul(R_CW.transpose(), -t_CW)
    W_t_Crect = W_t_Corig
    Crect_R_W = np.eye(3)
    Crect_t_W = - np.matmul(Crect_R_W, W_t_Crect)
    numPoints = coordinates.shape[0]
    projected = np.zeros((numPoints, 2), np.float32)
    for i in range(numPoints):
        pC = np.array([[coordinates[i, 0]], [coordinates[i, 1]], [0]]) + Crect_t_W
        rawp = np.matmul(K, pC)
        projected[i, :] = rawp[:2].transpose() / rawp[2]
    return projected

