#!/usr/bin/env python3
"""슬래브 dataset 의 각 sample → CVAT 업로드용 label 모음 + 참고용 검수뷰 모음.

dataset 하나당 <out>/<dataset이름>/ 폴더 하나를 만들고 그 아래 두 하위 폴더로 나눈다:
  <이름>/label/  — 각 sample 의 `overlay.png`(IPM + obstacle 오버레이) 를 sample_NNNNNN.png 로.
                   **합성하지 않고 그대로 복사한다** — overlay.png 자체가 이미 네이티브
                   해상도(NX×NY)·장식 없는 annotation base 라 다시 그릴 게 없다.
                   **이 폴더를 그대로** CVAT 등에 올려 auto-label 을 사람이 보정한다.
  <이름>/review/ — 각 sample 의 `review.png`(원본 3어안 + 4색 BEV + IPM+occupancy 두 패널).
                   sample 폴더를 하나씩 열지 않고 한곳에서 훑어보기 위함.
                   --review-scale 로 배율만 올릴 수 있고 0 이면 생략한다.
  <이름>/label_guided/ — --guided-labels 를 켰을 때만 생성. 같은 해상도(NX×NY)에 obstacle
                   alpha 를 높이고 0.5m 격자를 얹은 참고/대체 annotation base 다.

§A 의 `gather_annotations.py` 는 슬래브 산출물에 동작하지 않는다 — 그 스크립트는 sample 마다
`label.png`(0/1/2) 를 요구하는데 슬래브는 라벨이 `occupancy.png`+`visibility.png` 두 채널로
나뉘어 있어 전 sample 이 skip 된다. 두 포맷은 그리드도 다르므로(80×80 vs 120×120) 한 스크립트로
합치지 않고 이렇게 따로 둔다.

사용:
  cd calibration/bev_autolabel
  python3 gather_slab.py --dataset ../../data/bev/slab/raws1 --out ../../data/bev/annotations
  python3 gather_slab.py --dataset ../../data/bev/slab/raws1 --out ../../data/bev/annotations --guided-labels
"""
import argparse
import pathlib
import shutil
import sys

import cv2
import numpy as np
from PIL import Image

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
from gather_annotations import BevSpec, spec_from_meta     # noqa: E402  (BEV 범위 복원 로직 재사용)
import slab_render as sr                          # noqa: E402


def scaled_copy(src, dst, scale=1.0):
    """scale==1 이면 재인코딩 없이 바이트 그대로 복사한다.

    label base 는 CVAT 에서 그 위에 그린 마스크가 곧 정답이 되므로 화소가 변하면 안 된다.
    확대가 필요한 review 만 최근접 보간으로 키운다(셀 경계를 흐리지 않기 위해).
    """
    if scale == 1.0:
        shutil.copyfile(str(src), str(dst))
        return
    im = cv2.imread(str(src), cv2.IMREAD_UNCHANGED)
    h, w = im.shape[:2]
    cv2.imwrite(str(dst), cv2.resize(im, (int(w * scale), int(h * scale)),
                                     interpolation=cv2.INTER_NEAREST))


def write_guided_label(sample_dir, dst, spec, alpha=0.55):
    """Write a native-size annotation guide with stronger occupancy and grid."""
    sd = pathlib.Path(sample_dir)
    ipm = cv2.imread(str(sd / "ipm_rgb.png"))
    if ipm is None:
        raise FileNotFoundError(sd / "ipm_rgb.png")
    occupancy = np.array(Image.open(sd / "occupancy.png"))
    if occupancy.shape != (spec.NX, spec.NY):
        raise ValueError(f"{sd.name}/occupancy.png {occupancy.shape} != "
                         f"spec {(spec.NX, spec.NY)}")
    cv2.imwrite(str(dst), sr.guided_label(ipm, occupancy, spec, alpha=alpha))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True, help="sample_* 들이 있는 슬래브 dataset 폴더")
    ap.add_argument("--out", default=str(_HERE.parent.parent / "data" / "bev" / "annotations"),
                    help="모음 상위 폴더(하위에 dataset 이름 폴더 생성)")
    ap.add_argument("--name", default="", help="하위 폴더 이름(기본=dataset 폴더명)")
    ap.add_argument("--review-scale", type=float, default=1.0,
                    help="review.png 확대 배율(1=원본 그대로 복사, 0=생성 안 함)")
    ap.add_argument("--guided-labels", action="store_true",
                    help="label_guided/ 에 alpha 높은 obstacle+0.5m grid 참고 이미지를 추가 생성")
    ap.add_argument("--guided-alpha", type=float, default=0.55,
                    help="label_guided/ obstacle 오버레이 불투명도")
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
    guided_dir = root / "label_guided"
    if a.guided_labels:
        guided_dir.mkdir(parents=True, exist_ok=True)

    spec = spec_from_meta(samples[0])
    print(f"BEV {spec.NX}x{spec.NY} (XF={spec.XF} XR={spec.XR} YH={spec.YH} RES={spec.RES})")
    n = 0
    for sd in samples:
        ov = sd / "overlay.png"
        if not ov.exists():
            print(f"skip (missing overlay.png — --no-ipm 로 만든 dataset?) {sd.name}")
            continue
        w, h = Image.open(ov).size                 # 헤더만 읽는다(디코딩 없음)
        if (h, w) != (spec.NX, spec.NY):
            sys.exit(f"{sd.name}/overlay.png {(h, w)} != spec {(spec.NX, spec.NY)}"
                     " — meta.json 의 범위와 라벨 해상도가 다릅니다")
        scaled_copy(ov, label_dir / f"{sd.name}.png")
        rv = sd / "review.png"
        if a.review_scale and rv.exists():
            scaled_copy(rv, review_dir / f"{sd.name}.png", a.review_scale)
        if a.guided_labels:
            try:
                write_guided_label(sd, guided_dir / f"{sd.name}.png", spec,
                                   alpha=a.guided_alpha)
            except (FileNotFoundError, ValueError) as e:
                print(f"skip guided ({e}) {sd.name}")
        n += 1
    msg = f"done: {n} images → {label_dir}"
    if a.review_scale:
        msg += f"  (+ review → {review_dir})"
    if a.guided_labels:
        msg += f"  (+ guided → {guided_dir})"
    print(msg)


if __name__ == "__main__":
    main()
