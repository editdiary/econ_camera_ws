"""gather_slab 순수 로직 테스트(파일 IO 는 tmp_path). 하드웨어·dataset 불필요."""
import os
import sys

import numpy as np
from PIL import Image

BA = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BA)

import gather_slab as gs                            # noqa: E402


def _png(path, arr):
    Image.fromarray(np.asarray(arr, np.uint8)).save(str(path))


def test_scaled_copy_default_is_byte_identical(tmp_path):
    """label base 는 CVAT 마스크가 곧 정답이라 재인코딩으로 화소가 변하면 안 된다."""
    src = tmp_path / "overlay.png"
    _png(src, np.random.RandomState(0).randint(0, 256, (7, 5, 3)))
    dst = tmp_path / "out.png"
    gs.scaled_copy(src, dst)
    assert dst.read_bytes() == src.read_bytes()


def test_scaled_copy_enlarges_with_nearest(tmp_path):
    """review 확대는 최근접이어야 셀 경계가 흐려지지 않는다(새 화소값이 생기면 안 된다)."""
    src = tmp_path / "review.png"
    _png(src, np.array([[0, 255], [255, 0]]))
    dst = tmp_path / "big.png"
    gs.scaled_copy(src, dst, 3)
    out = np.array(Image.open(dst).convert("L"))
    assert out.shape == (6, 6)
    assert set(np.unique(out)) == {0, 255}


def test_write_guided_label_uses_ipm_and_occupancy_without_resizing(tmp_path):
    sd = tmp_path / "sample_000000"
    sd.mkdir()
    ipm = np.full((40, 40, 3), 100, np.uint8)
    occ = np.ones((40, 40), np.uint8)
    _png(sd / "ipm_rgb.png", ipm)
    _png(sd / "occupancy.png", occ)
    dst = tmp_path / "guided.png"
    gs.write_guided_label(sd, dst, gs.BevSpec(XF=1.0, XR=1.0, YH=1.0), alpha=0.55)
    out = np.array(Image.open(dst).convert("RGB"))
    assert out.shape == ipm.shape
    assert tuple(out[10, 5]) == (70, 70, 70)
