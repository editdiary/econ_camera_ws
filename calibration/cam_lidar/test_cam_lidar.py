"""순수 로직 테스트(ROS·이미지·bag 불필요).

실행: cd calibration/cam_lidar && python3 -m pytest -q
"""
import numpy as np

from chain import se3, se3_inv, se3_to_rvec_tvec, se3_from_rpy_xyz, transform


def test_se3_identity_roundtrip():
    T = se3(np.zeros(3), np.zeros(3))
    assert np.allclose(T, np.eye(4))


def test_se3_inv_is_inverse():
    T = se3_from_rpy_xyz(10, -20, 30, 0.1, -0.2, 0.3)
    assert np.allclose(T @ se3_inv(T), np.eye(4), atol=1e-12)


def test_se3_to_rvec_tvec_roundtrip():
    rvec = np.array([0.2, -0.5, 0.1])
    tvec = np.array([1.0, 2.0, -3.0])
    T = se3(rvec, tvec)
    r2, t2 = se3_to_rvec_tvec(T)
    assert np.allclose(se3(r2, t2), T, atol=1e-12)


def test_transform_translation():
    T = se3(np.zeros(3), np.array([1.0, 2.0, 3.0]))
    P = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]])
    out = transform(T, P)
    assert np.allclose(out, [[1, 2, 3], [2, 3, 4]])


def test_se3_from_rpy_90deg_z():
    T = se3_from_rpy_xyz(0, 0, 90, 0, 0, 0)
    out = transform(T, np.array([[1.0, 0.0, 0.0]]))
    assert np.allclose(out[0], [0, 1, 0], atol=1e-9)
