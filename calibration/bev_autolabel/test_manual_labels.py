"""manual_labels pure logic and CLI tests. No hardware or real dataset required."""
import json
import pathlib
import sys

import numpy as np
from PIL import Image

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

import manual_labels as ml  # noqa: E402


def _png(path, arr):
    Image.fromarray(np.asarray(arr, np.uint8)).save(str(path))


def test_infer_dataset_name_from_manual_annotation_dir():
    assert ml.infer_dataset_name("raws1_120x120_annotation") == "raws1"
    assert ml.infer_dataset_name("/tmp/rawos3_120x120_annotation") == "rawos3"


def test_mask_rgb_to_occupancy_uses_labelmap_color():
    rgb = np.zeros((2, 3, 3), np.uint8)
    rgb[0, 0] = (61, 61, 245)
    rgb[1, 2] = (63, 60, 244)

    occ = ml.mask_rgb_to_occupancy(rgb, (61, 61, 245), color_tol=3)

    assert occ.dtype == np.uint8
    assert occ.tolist() == [[0, 1, 1], [1, 1, 0]]


def test_generate_manual_labels_writes_npy_png_review_and_manifest(tmp_path):
    manual = tmp_path / "manual_annotated" / "raws1_120x120_annotation"
    seg = manual / "SegmentationClass"
    seg.mkdir(parents=True)
    (manual / "labelmap.txt").write_text(
        "# label:color_rgb:parts:actions\n"
        "background:0,0,0::\n"
        "occupancy:61,61,245::\n"
    )
    mask = np.zeros((40, 40, 3), np.uint8)
    mask[10:20, 18:22] = (61, 61, 245)
    _png(seg / "sample_000000.png", mask)

    dataset = tmp_path / "dataset" / "raws1" / "sample_000000"
    dataset.mkdir(parents=True)
    (dataset / "meta.json").write_text(json.dumps({
        "bev": {"XF": 1.0, "XR": 1.0, "YH": 1.0, "RES": 0.05}
    }))
    _png(dataset / "ipm_rgb.png", np.full((40, 40, 3), 100, np.uint8))

    out = tmp_path / "manual_labels"
    stats = ml.generate_manual_labels(manual, dataset.parent, out)

    root = out / "raws1"
    assert stats["samples"] == 1
    occ = np.load(root / "occupancy_npy" / "sample_000000.npy")
    vis = np.load(root / "visibility_npy" / "sample_000000.npy")
    assert occ.shape == (40, 40)
    assert vis.shape == (40, 40)
    assert set(np.unique(occ)) == {0, 1}
    assert set(np.unique(vis)) == {0, 1}
    assert np.array_equal(occ, np.array(Image.open(root / "occupancy_png" / "sample_000000.png")))
    assert np.array_equal(vis, np.array(Image.open(root / "visibility_png" / "sample_000000.png")))
    assert (root / "review_png" / "sample_000000.png").exists()
    assert (root / "labels.csv").read_text().splitlines()[0].startswith("sample,occupancy_npy")
    assert "0=obstacle" in (root / "README.md").read_text()
