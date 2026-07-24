#!/usr/bin/env python3
"""수동 2D–3D 대응점 수집 → correspondences.json.

흐름(카메라 1대씩): (1) 이미지 창에서 물리 점 픽셀 클릭 → (2) open3d 창에서 같은 점 클릭.
open3d VisualizerWithEditing 은 [shift+좌클릭]으로 점 선택, 창 닫으면 인덱스 반환.

사용:
  python3 pick_correspondences.py --cloud cloud.npy --image extracted/frame_000800/cam2.jpg \
    --cam front --out correspondences.json --append
"""
import argparse
import json
import pathlib

import cv2
import numpy as np
import open3d as o3d


def pick_pixels(image_path):
    """이미지 창 클릭 → [(u,v), ...]. 아무키나 누르면 종료."""
    img = cv2.imread(image_path)
    pts = []

    def on_click(ev, x, y, flags, _):
        if ev == cv2.EVENT_LBUTTONDOWN:
            pts.append((float(x), float(y)))
            cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(img, str(len(pts)), (x + 6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow("image (click points, any key=done)", img)

    cv2.imshow("image (click points, any key=done)", img)
    cv2.setMouseCallback("image (click points, any key=done)", on_click)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return pts


def pick_points_3d(xyzi):
    """open3d 편집 뷰 → 선택 점 인덱스 리스트(shift+좌클릭, 창 닫기=종료)."""
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyzi[:, :3])
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window("cloud (shift+click same points, close=done)")
    vis.add_geometry(pc)
    vis.run()
    vis.destroy_window()
    return vis.get_picked_points()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cloud", required=True)
    ap.add_argument("--image", required=True)
    ap.add_argument("--cam", required=True, help="front/right/rear/left")
    ap.add_argument("--out", required=True)
    ap.add_argument("--append", action="store_true", help="기존 out에 이어붙임")
    args = ap.parse_args()

    xyzi = np.load(args.cloud)
    uv = pick_pixels(args.image)
    idx = pick_points_3d(xyzi)
    n = min(len(uv), len(idx))
    if len(uv) != len(idx):
        print(f"[경고] 2D {len(uv)}개 ≠ 3D {len(idx)}개 → 앞 {n}쌍만 사용")

    corrs = []
    if args.append and pathlib.Path(args.out).exists():
        corrs = json.loads(pathlib.Path(args.out).read_text())
    for i in range(n):
        corrs.append({"cam": args.cam, "uv": list(uv[i]),
                      "xyz": xyzi[idx[i], :3].tolist()})
    pathlib.Path(args.out).write_text(json.dumps(corrs, indent=1))
    print(f"{n}쌍 저장(누적 {len(corrs)}) → {args.out}")


if __name__ == "__main__":
    main()
