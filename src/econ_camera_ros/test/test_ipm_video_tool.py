import csv
from pathlib import Path

import cv2
import numpy as np

from tools.ipm_video_from_extract import (
    compose_video_frame,
    draw_bev_review,
    iter_extracted_frames,
    parse_args,
)


def test_iter_extracted_frames_maps_camera_indices_to_names(tmp_path):
    extract = tmp_path / "extract"
    frame0 = extract / "frame_000000"
    frame0.mkdir(parents=True)
    for idx, value in ((0, 10), (1, 20), (3, 30)):
        img = np.full((4, 5, 3), value, np.uint8)
        assert cv2.imwrite(str(frame0 / f"cam{idx}.jpg"), img)
    with (extract / "sets.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["idx", "stamp0", "stamp1", "stamp2", "stamp3", "spread_ms"])
        writer.writerow([0, "1.0", "1.0", "1.0", "1.0", "0.0"])

    frames = list(iter_extracted_frames(
        extract,
        {0: "front", 1: "right", 2: "rear", 3: "left"},
        ("front", "left"),
    ))

    assert len(frames) == 1
    assert frames[0].idx == 0
    assert frames[0].stamp == 1.0
    assert set(frames[0].imgs) == {"front", "left"}
    assert int(frames[0].imgs["front"][0, 0, 0]) == 10
    assert int(frames[0].imgs["left"][0, 0, 0]) == 30


def test_draw_bev_review_scales_and_marks_ego():
    class Spec:
        NX = 3
        NY = 4
        RES = 0.05
        XF = 0.1
        XR = 0.05
        YH = 0.1
        R_EGO = 2
        C_EGO = 2

    bev = np.zeros((3, 4, 3), np.uint8)
    bev[0, 0] = (1, 2, 3)

    out = draw_bev_review(bev, Spec(), scale=5, label="frame 7")

    assert out.shape == (15, 20, 3)
    assert tuple(out[0, 0]) == (1, 2, 3)
    assert out[Spec.R_EGO * 5, Spec.C_EGO * 5].sum() > 0


def test_cli_defaults_to_full_resolution_ipm():
    args = parse_args(["--out", "out.mp4"])

    assert args.pixel_step == 1


def test_compose_video_frame_adds_top_camera_strip():
    bev = np.full((2, 4, 3), (10, 20, 30), np.uint8)
    cam_imgs = {
        "left": np.full((3, 6, 3), (11, 0, 0), np.uint8),
        "front": np.full((3, 6, 3), (0, 22, 0), np.uint8),
        "right": np.full((3, 6, 3), (0, 0, 33), np.uint8),
    }

    out = compose_video_frame(bev, cam_imgs, width=12, camera_strip="top")

    assert out.shape == (8, 12, 3)
    assert tuple(out[0, 1]) == (11, 0, 0)
    assert tuple(out[0, 5]) == (0, 22, 0)
    assert tuple(out[0, 9]) == (0, 0, 33)
    assert tuple(out[-1, 0]) == (10, 20, 30)


def test_compose_video_frame_can_emit_bev_only():
    bev = np.full((2, 4, 3), (10, 20, 30), np.uint8)

    out = compose_video_frame(bev, {}, width=12, camera_strip="none")

    assert out.shape == (6, 12, 3)
    assert tuple(out[0, 0]) == (10, 20, 30)


def test_compose_video_frame_pads_odd_height_for_video_codec():
    bev = np.full((3, 4, 3), (10, 20, 30), np.uint8)

    out = compose_video_frame(bev, {}, width=12, camera_strip="none")

    assert out.shape == (10, 12, 3)
    assert tuple(out[-1, 0]) == (0, 0, 0)
