import numpy as np
import tf.transformations
from scipy.spatial.transform import Rotation

def quaternion_to_euler(qxyzw):
    rpy = tf.transformations.euler_from_quaternion(qxyzw)
    return rpy

def rot_to_euler(R):
    rpy = Rotation.from_matrix(R).as_euler('xyz', degrees=False)
    return rpy

def quat_to_euler(qxyzw):
    rpy = Rotation.from_quat(qxyzw).as_euler('xyz', degrees=False)
    return rpy

def test_euler(rpy, R):
    rpy1 = rot_to_euler(R)
    quat = Rotation.from_matrix(R).as_quat()
    rpy2 = quaternion_to_euler(quat)
    rpy3 = quat_to_euler(quat)
    assert np.allclose(rpy, rpy1, atol=1e-6)
    assert np.allclose(rpy, rpy2)
    assert np.allclose(rpy, rpy3)


if __name__ == "__main__":
    rpy = np.array([0.792207329559554, 0.959492426392903, 0.655740699156587])
    R = np.array([[0.454899440647356, 0.0338573378941509, 0.889898971552077],
                  [0.349955283760892, 0.9120908369274, -0.213592145363327],
                  [-0.818900359180869, 0.408587794560479, 0.403061057247717]])
    test_euler(rpy, R)

    rpy = np.array([0.0357116785741896, 0.849129305868777, 0.933993247757551])

    R = np.array([[0.392833380385338, -0.787550246177219, 0.474812115474354],
                  [0.531152729522812, 0.615798865118291, 0.581952348254985],
                  [-0.750705476985145, 0.0235874429565128, 0.660215812715266]])
    test_euler(rpy, R)
    rpy = np.array([-0.215545960931664,
                    0.310955780355113,
                    -0.657626624376876])

    R = np.array([[0.753488691105156, 0.545303111171868, 0.367286413202176],
        [-0.581926056783167, 0.813131002983231, -0.0134177652490806],
        [-0.305968718684496, -0.203623399769689, 0.930011104370696]])
    test_euler(rpy, R)
