"""annotation tool 연동용 flatten/gather 헬퍼.

annotation tool 은 모든 이미지를 한 폴더에 넣어야 하는데 sample 마다 cam_front/left/right.jpg
이름이 겹친다. 그래서 sample 번호를 파일명에 인코딩해 펼치고(export), 작업 후 마스크를
다시 sample 구조로 되돌린다(gather).

  export:  <dataset>/sample_NNNNNN/cam_{name}.jpg  ->  <flat>/sample_NNNNNN__cam_{name}.jpg
  gather:  <flat-masks>/sample_NNNNNN__cam_{name}.png  ->  <ann>/sample_NNNNNN/cam_{name}.png

gather 는 파일명 어디에든 `sample_NNNNNN__cam_<name>` 이 들어있으면 인식(툴이 접미사를
붙여도 OK: 예 sample_000000__cam_front_png.rf.HASH.png). 마스크는 >0 을 drivable(255)로 이진화.
"""
import argparse
import glob
import os
import re
import shutil
import sys

import cv2

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
    g = sub.add_parser("gather", help="펼쳐진 마스크를 sample 구조로 되돌리기")
    g.add_argument("--flat-masks", required=True, help="sample_NNNNNN__cam_*.png 마스크 폴더")
    g.add_argument("--out", required=True, help="annotations/<bag> 루트(sample_NNNNNN/cam_*.png 생성)")
    args = ap.parse_args()
    if args.mode == "export":
        do_export(args.dataset_dir, args.out)
    else:
        do_gather(args.flat_masks, args.out)


if __name__ == "__main__":
    main()
