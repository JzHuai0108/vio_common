import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp


def pos_interpolate(p1, p2, t1, t2, t):
    return p1 + ((p2 - p1) / (t2 - t1)) * (t - t1)


def rot_slerp(q1, q2, t1, t2, t):
    rot_matrices = Rotation.random(2, random_state=2342345)
    rot_matrices[0] = Rotation.from_quat(q1)
    rot_matrices[1] = Rotation.from_quat(q2)
    slerp = Slerp([t1, t2], rot_matrices)
    interpolated_quaternion = slerp(t)
    return interpolated_quaternion


def rot_slerp_batch(keytimes, keyquats, querytimes):
    keyrots = Rotation.from_quat(keyquats)
    slerp = Slerp(keytimes, keyrots)
    interp_rots = slerp(querytimes)
    return interp_rots.as_quat()

def pos_interpolate_batch(keytimes, keypositions, querytimes):
    interp_positions = np.zeros((len(querytimes), 3))
    for i in range(keypositions.shape[1]):
        interp_positions[:, i] = np.interp(querytimes, keytimes, keypositions[:, i])
    return interp_positions

def test_pos_interpolate():
    keytimes = np.array([0.0, 1.0, 2.0, 3.0])
    keypositions = np.array([[0.0, 0.0, 0.0],
                             [1.0, 1.0, 1.0],
                             [2.0, 2.0, 2.0],
                             [3.0, 3.0, 3.0]])
    querytimes = np.array([0.5, 1.5, 2.5])
    interp_positions = pos_interpolate_batch(keytimes, keypositions, querytimes)
    print(interp_positions)
    # use pos_interpolate to compute again
    interp_positions2 = np.zeros((len(querytimes), 3))
    for i in range(len(querytimes)):
        interp_positions2[i, :] = pos_interpolate(keypositions[i, :], keypositions[i+1, :], keytimes[i], keytimes[i+1], querytimes[i])
    assert np.allclose(interp_positions, interp_positions2)

def test_rot_slerp():
    keytimes = np.array([0.0, 1.0, 2.0, 3.0])
    keyquats = np.array([[0.0, 0.0, 0.0, 1.0],
                         [0.0, 0.0, 1.0, 0.0],
                         [0.0, 1.0, 0.0, 0.0],
                         [1.0, 0.0, 0.0, 0.0]])
    querytimes = np.array([0.5, 1.5, 2.5])
    interp_quats = rot_slerp_batch(keytimes, keyquats, querytimes)
    print(interp_quats)
    # use rot_slerp to compute again
    interp_quats2 = np.zeros((len(querytimes), 4))
    for i in range(len(querytimes)):
        interp_quat = rot_slerp(keyquats[i, :], keyquats[i+1, :], keytimes[i], keytimes[i+1], querytimes[i])
        interp_quats2[i, :] = interp_quat.as_quat()
    assert np.allclose(interp_quats, interp_quats2)

if __name__ == "__main__":
    test_pos_interpolate()
    test_rot_slerp()
