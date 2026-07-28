"""annotation tool 연동용 flatten/gather 헬퍼.

annotation tool 은 모든 이미지를 한 폴더에 넣어야 하는데 sample 마다 cam_front/left/right.jpg
이름이 겹친다. 그래서 sample 번호를 파일명에 인코딩해 펼치고(export), 작업 후 마스크를
다시 sample 구조로 되돌린다(gather).

  export:      <dataset>/sample_NNNNNN/cam_{name}.jpg  ->  <flat>/sample_NNNNNN__cam_{name}.jpg
  gather:      <flat-masks>/sample_NNNNNN__cam_{name}.png  ->  <ann>/sample_NNNNNN/cam_{name}.png
  gather-cvat: <cvat-export>/SegmentationClass/sample_NNNNNN__cam_{name}.png  ->  <ann>/sample_NNNNNN/cam_{name}.png

gather 는 파일명 어디에든 `sample_NNNNNN__cam_<name>` 이 들어있으면 인식(툴이 접미사를
붙여도 OK: 예 sample_000000__cam_front_png.rf.HASH.png). 마스크는 >0 을 drivable(255)로 이진화.

gather-cvat 는 CVAT "Segmentation mask 1.1" 내보내기용. SegmentationClass PNG 는 0/1 이진이
아니라 클래스별 RGB 컬러(색은 프로젝트 설정마다 다름)라, 같은 폴더의 labelmap.txt 에서 대상
클래스(기본 drivable)의 색을 읽어 그 색과 정확히 일치하는 픽셀만 255 로 만든다(설정 무관).
"""
import argparse
import glob
import os
import re
import shutil
import sys

import cv2
import numpy as np

USE = ("front", "left", "right")
_PAT = re.compile(r"(sample_\d+)__cam_(front|left|right)")


def parse_sample_cam(filename):
    """파일명에서 (sample, cam) 추출. 못 찾으면 (None, None)."""
    m = _PAT.search(os.path.basename(filename))
    return (m.group(1), m.group(2)) if m else (None, None)


def do_export(dataset_dir, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    n = 0
    for sdir in sorted(glob.glob(os.path.join(dataset_dir, "sample_*"))):
        name = os.path.basename(sdir)
        for cam in USE:
            src = os.path.join(sdir, f"cam_{cam}.jpg")
            if os.path.exists(src):
                shutil.copy(src, os.path.join(out_dir, f"{name}__cam_{cam}.jpg"))
                n += 1
    print(f"export: {n} images -> {out_dir}")


def parse_labelmap(path):
    """CVAT labelmap.txt → {label_name: (r, g, b)}. 주석(#)·빈 줄 무시."""
    lm = {}
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split(":")
            if len(fields) < 2 or not fields[1]:
                continue
            lm[fields[0]] = tuple(int(v) for v in fields[1].split(","))
    return lm


def color_to_binary(img_bgr, rgb):
    """BGR 이미지에서 RGB 색과 정확히 일치하는 픽셀 → 0/255 uint8."""
    r, g, b = rgb
    target = np.array([b, g, r], np.uint8)             # cv2 는 BGR
    return (np.all(img_bgr == target, axis=2).astype("uint8")) * 255


def do_gather_cvat(cvat_dir, out_dir, class_name="drivable"):
    lm_path = os.path.join(cvat_dir, "labelmap.txt")
    seg_dir = os.path.join(cvat_dir, "SegmentationClass")
    if not os.path.isfile(lm_path) or not os.path.isdir(seg_dir):
        sys.exit(f"[ERR] CVAT 내보내기 구조 아님(labelmap.txt + SegmentationClass/ 필요): {cvat_dir}")
    lm = parse_labelmap(lm_path)
    if class_name not in lm:
        sys.exit(f"[ERR] labelmap 에 '{class_name}' 클래스 없음 — 있는 클래스: {list(lm)}")
    rgb = lm[class_name]
    n, skipped = 0, 0
    for f in sorted(glob.glob(os.path.join(seg_dir, "*.png"))):
        sample, cam = parse_sample_cam(f)
        if sample is None:
            skipped += 1; continue
        img = cv2.imread(f, cv2.IMREAD_COLOR)
        if img is None:
            skipped += 1; continue
        m = color_to_binary(img, rgb)
        d = os.path.join(out_dir, sample); os.makedirs(d, exist_ok=True)
        cv2.imwrite(os.path.join(d, f"cam_{cam}.png"), m)
        n += 1
    print(f"gather-cvat: {n} masks -> {out_dir} (class={class_name} rgb={rgb})"
          + (f" (인식 못한 파일 {skipped}개 건너뜀)" if skipped else ""))


def do_gather(flat_masks, out_dir):
    n, skipped = 0, 0
    for f in sorted(glob.glob(os.path.join(flat_masks, "*"))):
        if not os.path.isfile(f):
            continue
        sample, cam = parse_sample_cam(f)
        if sample is None:
            skipped += 1; continue
        m = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if m is None:
            skipped += 1; continue
        m = ((m > 0).astype("uint8")) * 255            # 이진화(>0 = drivable)
        d = os.path.join(out_dir, sample); os.makedirs(d, exist_ok=True)
        cv2.imwrite(os.path.join(d, f"cam_{cam}.png"), m)
        n += 1
    print(f"gather: {n} masks -> {out_dir}" + (f" (인식 못한 파일 {skipped}개 건너뜀)" if skipped else ""))


def main():
    ap = argparse.ArgumentParser(description="annotation tool 연동 flatten/gather")
    sub = ap.add_subparsers(dest="mode", required=True)
    e = sub.add_parser("export", help="sample 이미지를 한 폴더로 펼치기(이름에 sample 인코딩)")
    e.add_argument("--dataset-dir", required=True)
    e.add_argument("--out", required=True, help="펼친 이미지 저장 폴더(tool 업로드용)")
    g = sub.add_parser("gather", help="펼쳐진 이진 마스크를 sample 구조로 되돌리기")
    g.add_argument("--flat-masks", required=True, help="sample_NNNNNN__cam_*.png 마스크 폴더")
    g.add_argument("--out", required=True, help="annotations/<bag> 루트(sample_NNNNNN/cam_*.png 생성)")
    c = sub.add_parser("gather-cvat", help="CVAT Segmentation mask 내보내기를 sample 구조로 되돌리기")
    c.add_argument("--cvat-dir", required=True, help="CVAT 내보내기 루트(labelmap.txt + SegmentationClass/)")
    c.add_argument("--out", required=True, help="annotations/<bag> 루트(sample_NNNNNN/cam_*.png 생성)")
    c.add_argument("--class", dest="class_name", default="drivable", help="추출할 클래스명(기본 drivable)")
    args = ap.parse_args()
    if args.mode == "export":
        do_export(args.dataset_dir, args.out)
    elif args.mode == "gather":
        do_gather(args.flat_masks, args.out)
    else:
        do_gather_cvat(args.cvat_dir, args.out, args.class_name)


if __name__ == "__main__":
    main()
