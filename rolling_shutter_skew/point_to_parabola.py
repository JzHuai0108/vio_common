import numpy as np


def pointToParabola(parabolaCoeffs, point):
    """
    :param parabolaCoeffs: [a, b, c]; y = ax^2 + bx + c.
    :param point:
    :param guess:
    :return:
    """
    mineqcoeffs = [2*parabolaCoeffs[0]**2, 3 * parabolaCoeffs[0] * parabolaCoeffs[1],
                   2 * parabolaCoeffs[0] * (parabolaCoeffs[2] - point[1]) + 1 +  parabolaCoeffs[1]**2,
                   -point[0] + (parabolaCoeffs[2] - point[1]) * parabolaCoeffs[1]]
    roots = np.roots(mineqcoeffs)
    candidates = []
    for val in roots:
        if np.isreal(val):
            candidates.append([val, np.polyval(parabolaCoeffs, val)])

    if len(candidates) == 0:
        raise Exception("No real perpendicular foot found on the parabola for point {}".format(point))
    distances = []

    for c in candidates:
        distances.append(np.linalg.norm([point[0] - c[0], point[1] - c[1]]))
    i = np.argmin(distances)
    return candidates[i], distances[i]
