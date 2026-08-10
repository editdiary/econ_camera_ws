"""슬래브 라벨 렌더 테스트."""
import pathlib
import sys

import numpy as np

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent / "verify"))
from bev_label import BevSpec       # noqa: E402
import slab_render as sr            # noqa: E402


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
