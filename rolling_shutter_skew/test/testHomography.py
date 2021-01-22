import homography

import numpy as np


def testHomographyIdentity():
    fp = np.array([[1, 2, 1, -1, -2, -1], [1, 0, -1, -1, 0, 1], [1, 1, 1, 1, 1, 1]])
    expectedH = np.eye(3)
    tp = np.dot(expectedH, fp)

    actualH = homography.H_from_points(fp, tp)
    assert np.allclose(expectedH, actualH, 1e-7)


def testHomographyRandom():
    fp = np.array([[1, 2, 1, -1, -2, -1], [1, 0, -1, -1, 0, 1], [1, 1, 1, 1, 1, 1]])
    expectedH = np.random.rand(3, 3)
    expectedH[2, 2] = 1.0
    tp = np.dot(expectedH, fp)
    for i in range(tp.shape[1]):
        tp[:, i] /= tp[2, i]

    actualH = homography.H_from_points(fp, tp)
    assert np.allclose(expectedH, actualH, 1e-7), "expected H\n{}\nactual H:{}\n".format(expectedH, actualH)


def testHomographyOutliers():
    fp = np.array([[1, 2, 1, -1, -2, -1, 0, 0], [1, 0, -1, -1, 0, 1, 2, -2], [1, 1, 1, 1, 1, 1, 1, 1]])
    expectedH = np.random.rand(3, 3)
    expectedH[2, 2] = 1.0
    tp = np.dot(expectedH, fp)
    for i in range(tp.shape[1]):
        tp[:, i] /= tp[2, i]
    numOutliers = 1
    outliers = np.random.rand(3, numOutliers) * 100000000
    outliers[2, :] = 1.0
    tp[:, -numOutliers:] = outliers

    actualH = homography.H_from_points(fp, tp)
    assert np.allclose(expectedH, actualH, atol=0.2), "expected H\n{}\nactual H:{}\n".format(expectedH, actualH)
