"""ipm 순수 로직 테스트(이미지 전체 → 지면 IPM 투영). 하드웨어 불필요."""
import os
import sys

import numpy as np

BA = os.path.dirname(os.path.abspath(__file__))
for _p in (BA, os.path.join(BA, "..", "cam_lidar"), os.path.join(BA, "..", "verify")):
    sys.path.insert(0, os.path.abspath(_p))

from bev_label import BevSpec                       # noqa: E402
from ds_model import DoubleSphereCamera             # noqa: E402
from chain import project                            # noqa: E402
from ipm import ipm_project_rgb, ipm_canvas          # noqa: E402


def _pinhole(fx=300.0, cx=640.0, cy=360.0):
    # xi=0, alpha=0 → 순수 핀홀(투영/역투영 정확 역관계)
    cam = DoubleSphereCamera.__new__(DoubleSphereCamera)
    cam.fx = cam.fy = fx; cam.cx = cx; cam.cy = cy
    cam.xi = 0.0; cam.alpha = 0.0
    cam.width = 1280; cam.height = 720
    return cam


# ego→cam: 카메라가 바로 아래(ego -z)를 봄. ego(x,y,z)->cam(x,-y,-z).
_T_CAM_LIDAR = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]], float)


def test_ipm_transports_color_and_hits_expected_cell():
    spec = BevSpec(); H = 0.87; cam = _pinhole()
    # 단색 이미지 → 채워진 셀은 정확히 그 색(평균이 색을 희석하지 않음)
    img = np.full((720, 1280, 3), (10, 20, 30), np.uint8)   # BGR
    sumbgr, cnt = ipm_project_rgb(img, cam, _T_CAM_LIDAR, H, spec)
    assert cnt.sum() > 0
    m = cnt > 0
    avg = sumbgr[m] / cnt[m][:, None]
    assert np.allclose(avg, [10, 20, 30])

    # 특정 지면점이 기대 BEV 셀에 떨어지는지(project 와 역관계)
    Xg, Yg = 0.5, 0.3
    u, v, valid = project(np.array([Xg, Yg, -H]), np.eye(4), _T_CAM_LIDAR, cam)
    assert valid
    r = int(round((spec.XF - Xg) / spec.RES))
    c = int(round((spec.YH - Yg) / spec.RES))
    assert cnt[r, c] >= 1


def test_ipm_skips_upward_rays():
    # ego +z(위)를 보는 카메라(항등) → 바닥과 교차 없음 → 빈 격자
    spec = BevSpec(); cam = _pinhole()
    img = np.full((720, 1280, 3), (10, 20, 30), np.uint8)
    _, cnt = ipm_project_rgb(img, cam, np.eye(4), 0.87, spec)
    assert cnt.sum() == 0


def test_ipm_canvas_shape_and_fill():
    spec = BevSpec(); cam = _pinhole()
    imgs = {"front": np.full((720, 1280, 3), (10, 20, 30), np.uint8),
            "left": None, "right": None}
    canvas = ipm_canvas(imgs, {"front": cam}, {"front": _T_CAM_LIDAR},
                        np.eye(4), 0.87, spec, use_names=("front",))
    assert canvas.shape == (spec.NX, spec.NY, 3) and canvas.dtype == np.uint8
    assert (canvas > 0).any()
