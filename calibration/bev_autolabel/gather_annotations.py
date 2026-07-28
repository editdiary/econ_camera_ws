#!/usr/bin/env python3
"""dataset 폴더의 각 sample → CVAT 업로드용 오버레이 + 참고용 확대 검수뷰 모음.

dataset 하나당 <out>/<dataset이름>/ 폴더 하나를 만들고 그 아래 두 하위 폴더로 나눈다:
  <이름>/label/  — 각 sample 의 ipm_rgb+label 을 render.blend_label 로 합성한 sample_NNNNNN.png
                   (**네이티브 80×80·장식 없음** → CVAT 라벨링 마스크가 곧 80×80 정답, resize 왕복 없음).
                   **이 폴더를 그대로** CVAT 등 annotation 툴에 올려 auto-label 을 사람이 보정한다.
  <이름>/review/ — 참고용 확대 검수뷰 sample_NNNNNN.png(원본 3어안 + 확대 BEV = review.png 를 크게).
                   sample 폴더를 하나씩 열지 않고 한곳에서 훑어보기 위함. --review-scale 0 이면 생략.
dataset 마다 폴더가 하나로 묶여, 여러 dataset 을 모아도 경로가 엉키지 않는다.

dataset 은 재생성하지 않으므로 ipm_rgb+label(+cam_*.jpg) 만 있으면 기존 산출물에도 바로 동작한다.

사용:
  cd calibration/bev_autolabel
  python3 gather_annotations.py \
    --dataset ../../data/bev/dataset/raws1 \
    --out ../../data/bev/annotations
"""
import argparse
import pathlib
import sys

import numpy as np
import cv2
from PIL import Image

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "cam_lidar"))
sys.path.insert(0, str(_HERE.parent / "verify"))
import render                             # noqa: E402
from bev_label import BevSpec             # noqa: E402

USE = ("front", "left", "right")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True, help="sample_* 들이 있는 dataset 폴더")
    ap.add_argument("--out", default=str(_HERE.parent.parent / "data" / "bev" / "annotations"),
                    help="모음 상위 폴더(하위에 dataset 이름 폴더 생성)")
    ap.add_argument("--name", default="", help="하위 폴더 이름(기본=dataset 폴더명)")
    ap.add_argument("--alpha", type=float, default=0.45, help="라벨 오버레이 불투명도")
    ap.add_argument("--review-scale", type=int, default=18,
                    help="참고용 review 합성 BEV 확대 배율(카메라 크기 결정, 0=생성 안 함)")
    a = ap.parse_args()

    ds = pathlib.Path(a.dataset)
    samples = sorted(d for d in ds.glob("sample_*") if d.is_dir())
    if not samples:
        sys.exit(f"no sample_* under {ds}")
    root = pathlib.Path(a.out) / (a.name or ds.name)
    label_dir = root / "label"
    label_dir.mkdir(parents=True, exist_ok=True)
    review_dir = root / "review"
    if a.review_scale:
        review_dir.mkdir(parents=True, exist_ok=True)

    spec = BevSpec()
    n = 0
    for sd in samples:
        rgb, lab = sd / "ipm_rgb.png", sd / "label.png"
        if not (rgb.exists() and lab.exists()):
            print(f"skip (missing ipm_rgb/label) {sd.name}")
            continue
        ipm = cv2.imread(str(rgb))
        label = np.array(Image.open(lab))              # P 모드 → 인덱스(0/1/2) 2D
        cv2.imwrite(str(label_dir / f"{sd.name}.png"),
                    render.blend_label(ipm, label, alpha=a.alpha))
        if a.review_scale:
            cams = {nm: cv2.imread(str(sd / f"cam_{nm}.jpg")) for nm in USE}
            cams = {k: v for k, v in cams.items() if v is not None}
            cv2.imwrite(str(review_dir / f"{sd.name}.png"),
                        render.review_overlay(ipm, label, cams, spec,
                                              alpha=a.alpha, scale=a.review_scale))
        n += 1
    msg = f"done: {n} images → {label_dir}"
    if a.review_scale:
        msg += f"  (+ review → {review_dir})"
    print(msg)


if __name__ == "__main__":
    main()
