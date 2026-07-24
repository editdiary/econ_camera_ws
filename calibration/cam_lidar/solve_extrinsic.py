#!/usr/bin/env python3
"""대응점(correspondences.json) → T_front_lidar → calib.yaml 기록.

correspondences.json = [{"cam":"front","uv":[u,v],"xyz":[x,y,z]}, ...]
초기값: --init-rpy/--init-xyz(대략 장착, deg·m) 또는 calib.yaml 의 기존 T_front_lidar.

사용:
  python3 solve_extrinsic.py correspondences.json \
    --calib ../../data/calib_260723/calib.yaml --orientation orientation.json \
    --init-rpy -90 0 0 --init-xyz 0.05 0 0.1 --holdout 3 --stage 1
"""
import argparse
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "verify"))
from ds_model import load_rig  # noqa: E402

from calib_io import load_T_front_lidar, write_extrinsic  # noqa: E402
from chain import residuals, se3_from_rpy_xyz, se3_to_rvec_tvec, solve  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("corrs")
    ap.add_argument("--calib", required=True)
    ap.add_argument("--orientation", required=True)
    ap.add_argument("--init-rpy", type=float, nargs=3, metavar=("R", "P", "Y"))
    ap.add_argument("--init-xyz", type=float, nargs=3, metavar=("X", "Y", "Z"), default=[0, 0, 0])
    ap.add_argument("--holdout", type=int, default=0, help="검증용으로 제외할 대응점 수")
    ap.add_argument("--huber", type=float, default=2.0)
    ap.add_argument("--stage", type=int, default=1)
    ap.add_argument("--no-write", action="store_true", help="calib.yaml 기록 생략(미리보기)")
    args = ap.parse_args()

    corrs = json.loads(pathlib.Path(args.corrs).read_text())
    rig = load_rig(args.calib, args.orientation)

    if args.init_rpy is not None:
        init_T = se3_from_rpy_xyz(*args.init_rpy, *args.init_xyz)
    else:
        init_T = load_T_front_lidar(args.calib)
        if init_T is None:
            print("[에러] --init-rpy 또는 calib.yaml 의 기존 T_front_lidar 필요", file=sys.stderr)
            sys.exit(1)

    hold = corrs[len(corrs) - args.holdout:] if args.holdout else []
    train = corrs[:len(corrs) - args.holdout] if args.holdout else corrs
    T, rms, per = solve(train, rig, init_T, huber=args.huber)
    print(f"T_front_lidar =\n{np.array2string(T, precision=6)}")
    print(f"train RMS = {rms:.3f} px (n={len(train)}, max={per.max():.2f})")

    hold_rms = None
    if hold:
        # 홀드아웃은 재최적화 없이 학습된 T로 재투영만.
        params = np.concatenate(se3_to_rvec_tvec(T))
        r = residuals(params, hold, rig).reshape(-1, 2)
        hold_rms = float(np.sqrt(np.mean((r ** 2).sum(1))))
        print(f"holdout RMS = {hold_rms:.3f} px (n={len(hold)})")

    if not args.no_write:
        meta = {"method": "manual-pnp-ds", "reproj_rms_px": round(rms, 3), "stage": args.stage}
        if hold_rms is not None:
            meta["holdout_rms_px"] = round(hold_rms, 3)
        write_extrinsic(args.calib, T, meta)
        print(f"→ {args.calib} 에 T_front_lidar + verification.cam_lidar 기록")


if __name__ == "__main__":
    main()
