"""ipm_review 순수 로직 테스트 (IPM 투영 라운드트립 + 융합 규칙). 하드웨어 불필요."""
import os
import sys

import numpy as np

BA = os.path.dirname(os.path.abspath(__file__))
for _p in (BA, os.path.join(BA, "..", "cam_lidar"), os.path.join(BA, "..", "verify")):
    sys.path.insert(0, os.path.abspath(_p))

from bev_label import BevSpec                       # noqa: E402
from ds_model import DoubleSphereCamera             # noqa: E402
from chain import project                            # noqa: E402
from ipm_review import ipm_project_mask, fuse_labels, _cell_dist  # noqa: E402
from dataset_flatten import parse_sample_cam                       # noqa: E402


def test_parse_sample_cam():
    assert parse_sample_cam("sample_000012__cam_front.jpg") == ("sample_000012", "front")
    # 툴이 접미사를 붙여도 인식
    assert parse_sample_cam("sample_000000__cam_left_png.rf.ABC.png") == ("sample_000000", "left")
    assert parse_sample_cam("sample_000003__cam_right.png") == ("sample_000003", "right")
    # rear 는 USE 밖 → 미인식(제외)
    assert parse_sample_cam("sample_000003__cam_rear.png") == (None, None)
    assert parse_sample_cam("random.png") == (None, None)


def _pinhole(fx=300.0, cx=640.0, cy=360.0):
    # xi=0, alpha=0 → 순수 핀홀(투영/역투영 정확 역관계)
    cam = DoubleSphereCamera.__new__(DoubleSphereCamera)
    cam.fx = cam.fy = fx; cam.cx = cx; cam.cy = cy
    cam.xi = 0.0; cam.alpha = 0.0
    return cam


# ego→cam: 카메라가 바로 아래(ego -z)를 봄. ego(x,y,z)->cam(x,-y,-z).
_T_CAM_LIDAR = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]], float)


def test_ipm_roundtrip_recovers_ground_cell():
    spec = BevSpec(); H = 0.87; cam = _pinhole()
    Xg, Yg = 0.5, 0.3
    Pg = np.array([Xg, Yg, -H])                       # 바닥면(z0=C_z-H=-H) 위 한 점
    u, v, valid = project(Pg, np.eye(4), _T_CAM_LIDAR, cam)
    assert valid
    mask = np.zeros((720, 1280), np.uint8)
    mask[int(round(v)), int(round(u))] = 255
    grid = ipm_project_mask(mask, cam, _T_CAM_LIDAR, H, spec)
    r = int(round((spec.XF - Xg) / spec.RES))
    c = int(round((spec.YH - Yg) / spec.RES))
    assert grid[r, c], "투영된 셀이 원래 지면점 셀과 일치해야 함"
    assert grid.sum() <= 2                             # 픽셀 1개 → 셀 1개 부근


def test_ipm_skips_upward_rays():
    # ego +z(위)를 보는 카메라 → 바닥과 교차 없음 → 빈 격자
    spec = BevSpec(); cam = _pinhole()
    T_up = np.eye(4)                                   # ego→cam 항등: 광선 +z(위)
    mask = np.zeros((720, 1280), np.uint8); mask[360, 640] = 255
    assert ipm_project_mask(mask, cam, T_up, 0.87, spec).sum() == 0


def test_fuse_rules():
    spec = BevSpec()
    lidar = np.full((spec.NX, spec.NY), 2, np.uint8)
    obs_rc = np.zeros((spec.NX, spec.NY), bool)
    ipm = np.zeros((spec.NX, spec.NY), bool)
    re, ce = spec.R_EGO, spec.C_EGO
    lidar[re, ce] = 1                                  # ego drivable(연결 시드)
    obs_rc[re - 5, ce] = True                          # 물리 장애물 셀
    ipm[re - 5, ce] = True                             # 이미지도 바닥으로 칠함 → 주황(=obstacle)
    ipm[re, ce + 1] = True                             # ego 옆(근거리) 후보 → drivable 승격
    far_r = 0                                          # 맨 위(전방 3m, near=2m 밖)
    ipm[far_r, ce] = True                              # 원거리 후보 → ignore 유지
    fused, agree = fuse_labels(lidar, obs_rc, ipm, spec, near_m=2.0)
    assert fused[re - 5, ce] == 0                       # 장애물 우선
    assert tuple(agree[re - 5, ce]) == (0, 140, 255)    # 주황(review)
    assert fused[re, ce + 1] == 1                       # 근거리 후보 승격
    assert fused[far_r, ce] == 2                        # 원거리 후보는 ignore
    # near/far 경계가 실제 2m 기준인지
    assert _cell_dist(spec)[far_r, ce] > 2.0
