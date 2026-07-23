"""순수 로직 테스트(하드웨어·이미지 불필요).

실행: cd calibration/verify && python3 -m pytest test_verify.py -q
"""
import numpy as np

from ds_model import DoubleSphereCamera
from rectify import cylindrical_rays, pinhole_rays, rays_to_maps

# calib_260723 front 카메라 실측값(회귀 방지용 고정 상수)
CAM = DoubleSphereCamera(
    xi=-0.19044687, alpha=0.60641073,
    fx=295.42965753, fy=294.59126009, cx=650.85752144, cy=351.40563724,
    width=1280, height=720, name="front")


def test_center_pixel_maps_to_optical_axis():
    d, ok = CAM.unproject(np.array([CAM.cx]), np.array([CAM.cy]))
    assert bool(ok[0])
    # 주점 역투영 ~ 광축(+z)
    assert abs(d[0, 0]) < 1e-6 and abs(d[0, 1]) < 1e-6
    assert d[0, 2] > 0.999


def test_roundtrip_unproject_then_project():
    us = np.linspace(60, CAM.width - 60, 50)
    vs = np.linspace(60, CAM.height - 60, 40)
    U, V = np.meshgrid(us, vs)
    dirs, oku = CAM.unproject(U, V)
    u2, v2, okp = CAM.project(dirs)
    m = oku & okp
    assert m.mean() > 0.9  # 대부분 유효
    err = np.sqrt((u2 - U) ** 2 + (v2 - V) ** 2)[m]
    assert err.max() < 1e-4  # 왕복 오차 서브픽셀 훨씬 이하


def test_unproject_returns_unit_vectors():
    U, V = np.meshgrid(np.linspace(100, 1180, 20), np.linspace(100, 620, 12))
    dirs, ok = CAM.unproject(U, V)
    n = np.linalg.norm(dirs[ok], axis=-1)
    assert np.allclose(n, 1.0, atol=1e-6)


def test_project_behind_camera_is_invalid():
    _, _, ok = CAM.project(np.array([[0.0, 0.0, -1.0]]))
    assert not bool(ok[0])


def test_pinhole_rays_center_is_forward():
    rays = pinhole_rays(200, 200, 90.0)
    c = rays[100, 100]
    assert c[2] == 1.0 and abs(c[0]) < 1e-9 and abs(c[1]) < 1e-9


def test_cylindrical_rays_are_unit_and_span():
    rays = cylindrical_rays(360, 100, hfov_deg=180.0)
    n = np.linalg.norm(rays, axis=-1)
    assert np.allclose(n, 1.0, atol=1e-6)
    # 좌우 끝은 ±90도 부근(+x / -x 방향)
    assert rays[50, -1, 0] > 0.99   # 우측 끝 ~ +x
    assert rays[50, 0, 0] < -0.99   # 좌측 끝 ~ -x


def test_rays_to_maps_shapes_and_validity():
    rays = pinhole_rays(120, 90, 90.0)
    mx, my, valid = rays_to_maps(CAM, rays)
    assert mx.shape == (90, 120) and my.shape == (90, 120)
    assert valid.dtype == bool
    # 중앙 정면은 유효하고 소스 주점 근처로 매핑
    assert valid[45, 60]
    assert abs(mx[45, 60] - CAM.cx) < 1.0 and abs(my[45, 60] - CAM.cy) < 1.0
