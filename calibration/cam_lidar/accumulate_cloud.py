#!/usr/bin/env python3
"""bag → 밀집 라이다 클라우드(.npy, x,y,z,intensity).

정지(1단계): --window 안의 프레임 병합. 이동(2단계): --trajectory 로 모션 보정.
대표 카메라 이미지는 별도로 bag_extract 로 뽑는다(중복 구현 안 함).

사용:
  source /opt/ros/humble/setup.bash && source <ws>/install/setup.bash
  # 1단계(정지)
  python3 accumulate_cloud.py <bag> -o cloud.npy --window 3
  # 2단계(이동)
  python3 accumulate_cloud.py <bag> -o cloud.npy --trajectory traj.tum --at 12.5 --window 2
"""
import argparse
import sys

import numpy as np

from cloud_io import (accumulate_motion, accumulate_static, load_tum,
                      read_clouds, voxel_downsample)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--topic", default="/unilidar/cloud")
    ap.add_argument("--window", type=float, default=3.0, help="누적 시간창(초)")
    ap.add_argument("--voxel", type=float, default=0.02, help="다운샘플 격자(m), 0이면 끔")
    ap.add_argument("--trajectory", help="TUM 궤적(주면 모션 보정 누적)")
    ap.add_argument("--at", type=float, help="기준 시각(초, bag 시작 기준). 미지정시 첫 스캔")
    args = ap.parse_args()

    clouds = read_clouds(args.bag, args.topic)
    if not clouds:
        print(f"[에러] {args.topic} 스캔 없음", file=sys.stderr)
        sys.exit(1)
    t0 = clouds[0][0]
    at_ns = t0 + int((args.at or 0.0) * 1e9) if args.at is not None else t0

    if args.trajectory:
        times, poses = load_tum(args.trajectory)
        xyzi = accumulate_motion(clouds, times, poses, at_ns, args.window)
        mode = "motion"
    else:
        xyzi = accumulate_static(clouds, at_ns, args.window)
        mode = "static"

    xyzi = voxel_downsample(xyzi, args.voxel)
    np.save(args.out, xyzi)
    lo, hi = xyzi[:, :3].min(0), xyzi[:, :3].max(0)
    print(f"[{mode}] {len(xyzi)} pts → {args.out}  ref_stamp_ns={at_ns}")
    print(f"  범위 x[{lo[0]:.1f},{hi[0]:.1f}] y[{lo[1]:.1f},{hi[1]:.1f}] z[{lo[2]:.1f},{hi[2]:.1f}]")
    print(f"  대표 이미지: ros2 run econ_camera_ros bag_extract {args.bag} -o extracted  (stamp≈{at_ns})")


if __name__ == "__main__":
    main()
