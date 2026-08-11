"""슬래브 라벨 렌더 테스트."""
import pathlib
import sys

import numpy as np
import pytest

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent / "verify"))
from bev_label import BevSpec       # noqa: E402
import slab_render as sr            # noqa: E402
import slab_sheet as ss             # noqa: E402


def test_save_indexed_roundtrip(tmp_path):
    from PIL import Image
    lab = np.array([[0, 1], [1, 0]], np.uint8)
    p = tmp_path / "occupancy.png"
    sr.save_indexed(str(p), lab, sr.PALETTE_OCC)
    got = np.array(Image.open(p))
    assert got.tolist() == lab.tolist()          # 인덱스값이 그대로 보존
    assert Image.open(p).mode == "P"


def test_four_color_distinguishes_all_combinations():
    occ = np.array([[0, 0], [1, 1]], np.uint8)   # 0=obstacle, 1=drivable
    vis = np.array([[1, 0], [1, 0]], np.uint8)
    img = sr.four_color(occ, vis)
    assert img.shape == (2, 2, 3)
    cols = {tuple(img[r, c]) for r in range(2) for c in range(2)}
    assert len(cols) == 4                         # 네 조합이 서로 다른 색


def test_review_png_scales_and_marks_ego():
    spec = BevSpec(XF=1.0, XR=1.0, YH=1.0)        # 40x40
    occ = np.ones((spec.NX, spec.NY), np.uint8)
    vis = np.ones((spec.NX, spec.NY), np.uint8)
    img = sr.review_png(occ, vis, spec, scale=4)
    assert img.shape == (spec.NX * 4, spec.NY * 4, 3)
    ey, ex = spec.R_EGO * 4, spec.C_EGO * 4
    # ego 마커는 정확히 cyan 색으로 그려진다 (grid line이 아님)
    assert tuple(img[ey, ex]) == sr._C_EGO


def test_review_png_camera_strip_is_optional_and_taller():
    """cam_imgs 를 안 주면 기존과 완전히 같은 BEV 전용 이미지, 주면 그 위에 카메라 줄만 붙는다."""
    spec = BevSpec(XF=1.0, XR=1.0, YH=1.0)        # 40x40
    occ = np.ones((spec.NX, spec.NY), np.uint8)
    vis = np.ones((spec.NX, spec.NY), np.uint8)
    base = sr.review_png(occ, vis, spec, scale=4)
    cams = {"front": np.full((72, 128, 3), (10, 20, 30), np.uint8),
            "left": np.full((72, 128, 3), (40, 50, 60), np.uint8),
            "right": np.full((72, 128, 3), (70, 80, 90), np.uint8)}
    strip = sr.review_png(occ, vis, spec, scale=4, cam_imgs=cams)
    assert strip.shape[1] == base.shape[1]                  # 폭은 그대로
    assert strip.shape[0] > base.shape[0]                   # 카메라 줄만큼 더 높다
    assert np.array_equal(strip[-base.shape[0]:], base)     # 아래쪽 BEV 는 그대로


def test_center_reach_far_survives_near_ego_gap():
    """근접 사각(ego 바로 앞 행들이 안 보임)이 있어도 더 먼 가시 구간이 far 로 잡혀야 한다.

    구 지표(ego 인접행부터 끊김없는 사슬을 따라감)의 버그를 재현하지 않는지 확인한다 —
    그 지표는 인접행이 안 보이는 순간 즉시 멈춰 far=0 을 냈다.
    """
    spec = BevSpec(XF=1.0, XR=1.0, YH=1.0)        # NX=NY=40, R_EGO=C_EGO=20
    b = {"R_EGO": spec.R_EGO, "C_EGO": spec.C_EGO, "RES": spec.RES}
    vis = np.zeros((spec.NX, spec.NY), bool)
    vis[5:10, b["C_EGO"]] = True                  # 먼 곳(행 5~9)만 보임
    # 행 10~19(ego 바로 앞, 근접 사각)는 계속 안 보임
    far, near = ss.center_reach(vis, b)
    assert far == pytest.approx((spec.R_EGO - 5) * spec.RES)    # 0.75m — 근접 사각과 무관
    assert near == pytest.approx((spec.R_EGO - 9) * spec.RES)   # 0.55m


def test_center_reach_all_invisible_is_zero():
    spec = BevSpec(XF=1.0, XR=1.0, YH=1.0)
    b = {"R_EGO": spec.R_EGO, "C_EGO": spec.C_EGO, "RES": spec.RES}
    vis = np.zeros((spec.NX, spec.NY), bool)
    assert ss.center_reach(vis, b) == (0.0, 0.0)


def test_center_reach_hits_front_edge():
    spec = BevSpec(XF=1.0, XR=1.0, YH=1.0)
    b = {"R_EGO": spec.R_EGO, "C_EGO": spec.C_EGO, "RES": spec.RES}
    vis = np.zeros((spec.NX, spec.NY), bool)
    vis[0:spec.R_EGO, b["C_EGO"]] = True          # 맨 앞줄(행 0)까지 전부 보임
    far, _ = ss.center_reach(vis, b)
    assert far == pytest.approx(spec.XF)          # 1.0m


def test_blend_slab_tints_obstacle_only_regardless_of_visibility():
    ipm = np.full((2, 3, 3), 100, np.uint8)
    occ = np.array([[0, 1, 0], [1, 0, 1]], np.uint8)     # 0=obstacle 1=drivable
    out = sr.blend_slab(ipm, occ, alpha=0.5)
    assert (out[occ == 1] == 100).all()                   # drivable → IPM 원본 그대로
    assert not (out[occ == 0] == 100).any()               # obstacle → 전부 색이 얹힌다
    # 보정 대상은 occupancy 뿐이므로 visibility 는 인자로 받지 않는다(raycast 로 재생성)
    assert "visibility" not in sr.blend_slab.__code__.co_varnames


def test_blend_slab_alpha_zero_is_untouched_ipm():
    ipm = np.full((2, 2, 3), 77, np.uint8)
    occ = np.zeros((2, 2), np.uint8)
    assert (sr.blend_slab(ipm, occ, alpha=0.0) == 77).all()
