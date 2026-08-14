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
    assert (root / "labels.csv").read_text().splitlines()[0].startswith("sample,rgb_dir,occupancy_npy")
    assert "0=obstacle" in (root / "README.md").read_text()


def test_generate_manual_labels_visibility_ignores_rear_self_box(tmp_path):
    manual = tmp_path / "manual_annotated" / "raws1_120x120_annotation"
    seg = manual / "SegmentationClass"
    seg.mkdir(parents=True)
    (manual / "labelmap.txt").write_text(
        "# label:color_rgb:parts:actions\n"
        "background:0,0,0::\n"
        "occupancy:61,61,245::\n"
    )
    _png(seg / "sample_000000.png", np.zeros((40, 40, 3), np.uint8))

    dataset = tmp_path / "dataset" / "raws1" / "sample_000000"
    dataset.mkdir(parents=True)
    (dataset / "meta.json").write_text(json.dumps({
        "bev": {"XF": 1.0, "XR": 1.0, "YH": 1.0, "RES": 0.05},
        "self_box": {"near": 0.2, "far": 0.6, "yh": 0.7},
    }))

    out = tmp_path / "manual_labels"
    ml.generate_manual_labels(manual, dataset.parent, out, review_scale=0)

    vis = np.load(out / "raws1" / "visibility_npy" / "sample_000000.npy")
    # x=-0.4, y=0.25 is inside the rear box, but visibility is now pure raycast.
    assert vis[28, 15] == 1
    # Same rear x but y=0.45 is outside |y|<=0.3 and remains visible.
    assert vis[28, 11] == 1


def test_generate_manual_labels_review_stacks_three_camera_images(tmp_path):
    manual = tmp_path / "manual_annotated" / "raws1_120x120_annotation"
    seg = manual / "SegmentationClass"
    seg.mkdir(parents=True)
    (manual / "labelmap.txt").write_text(
        "# label:color_rgb:parts:actions\n"
        "background:0,0,0::\n"
        "occupancy:61,61,245::\n"
    )
    _png(seg / "sample_000000.png", np.zeros((40, 40, 3), np.uint8))

    dataset = tmp_path / "dataset" / "raws1" / "sample_000000"
    dataset.mkdir(parents=True)
    (dataset / "meta.json").write_text(json.dumps({
        "bev": {"XF": 1.0, "XR": 1.0, "YH": 1.0, "RES": 0.05}
    }))
    _png(dataset / "ipm_rgb.png", np.full((40, 40, 3), 100, np.uint8))
    for i, name in enumerate(("front", "left", "right")):
        _png(dataset / f"cam_{name}.jpg", np.full((10, 20, 3), 40 + i * 50, np.uint8))

    out = tmp_path / "manual_labels"
    ml.generate_manual_labels(manual, dataset.parent, out, review_scale=2)

    review = np.array(Image.open(out / "raws1" / "review_png" / "sample_000000.png"))
    assert review.shape[1] == 160              # two 40x40 panels at scale 2
    assert review.shape[0] > 80                # camera strip is stacked above the BEV row


def test_generate_manual_labels_overlays_fixed_self_mask_on_review(tmp_path):
    manual = tmp_path / "manual_annotated" / "raws1_120x120_annotation"
    seg = manual / "SegmentationClass"
    seg.mkdir(parents=True)
    (manual / "labelmap.txt").write_text(
        "# label:color_rgb:parts:actions\n"
        "background:0,0,0::\n"
        "occupancy:61,61,245::\n"
    )
    _png(seg / "sample_000000.png", np.zeros((40, 40, 3), np.uint8))

    dataset = tmp_path / "dataset" / "raws1" / "sample_000000"
    dataset.mkdir(parents=True)
    mask_dir = tmp_path / "self_mask"
    mask_dir.mkdir()
    fixed = np.zeros((40, 40), np.uint8)
    fixed[5, 7] = 255
    _png(mask_dir / "bev_self_mask.png", fixed)
    rear = np.zeros((40, 40), np.uint8)
    rear[6, 8] = 255
    _png(mask_dir / "bev_rear_self_box_03.png", rear)
    (dataset / "meta.json").write_text(json.dumps({
        "bev": {"XF": 1.0, "XR": 1.0, "YH": 1.0, "RES": 0.05},
        "self_mask": str(mask_dir),
    }))
    _png(dataset / "ipm_rgb.png", np.full((40, 40, 3), 100, np.uint8))

    out = tmp_path / "manual_labels"
    stale = out / "raws1" / "mask_debug_png"
    stale.mkdir(parents=True)
    (stale / "old.png").write_bytes(b"stale")
    ml.generate_manual_labels(manual, dataset.parent, out, review_scale=2)

    root = out / "raws1"
    assert not (root / "mask_debug_png").exists()
    review = np.array(Image.open(root / "review_png" / "sample_000000.png"))
    y, x = 5 * 2, 7 * 2
    ry, rx = 6 * 2, 8 * 2
    assert review.shape == (80, 160, 3)
    r, g, b = review[y, x]
    assert r > 100 and g > 200 and b < 30
    rr, rg, rb = review[ry, rx]
    assert rr > 140 and rg < 100 and rb > 140


def test_generate_manual_labels_copies_rgb_images_for_training(tmp_path):
    manual = tmp_path / "manual_annotated" / "raws1_120x120_annotation"
    seg = manual / "SegmentationClass"
    seg.mkdir(parents=True)
    (manual / "labelmap.txt").write_text(
        "# label:color_rgb:parts:actions\n"
        "background:0,0,0::\n"
        "occupancy:61,61,245::\n"
    )
    _png(seg / "sample_000000.png", np.zeros((40, 40, 3), np.uint8))

    dataset = tmp_path / "dataset" / "raws1" / "sample_000000"
    dataset.mkdir(parents=True)
    (dataset / "meta.json").write_text(json.dumps({
        "bev": {"XF": 1.0, "XR": 1.0, "YH": 1.0, "RES": 0.05}
    }))
    for i, name in enumerate(("front", "left", "right")):
        src = dataset / f"cam_{name}.jpg"
        _png(src, np.full((10, 20, 3), 20 + i, np.uint8))

    out = tmp_path / "manual_labels"
    ml.generate_manual_labels(manual, dataset.parent, out, review_scale=0)

    root = out / "raws1"
    rgb_dir = root / "rgb_images" / "sample_000000"
    for name in ("front", "left", "right"):
        copied = rgb_dir / f"cam_{name}.jpg"
        assert copied.exists()
        assert copied.read_bytes() == (dataset / f"cam_{name}.jpg").read_bytes()
    row = (root / "labels.csv").read_text().splitlines()[1]
    assert row.startswith("sample_000000,rgb_images/sample_000000,")
