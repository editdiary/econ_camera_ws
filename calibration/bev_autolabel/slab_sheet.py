#!/usr/bin/env python3
"""검수용 컨택트시트 + reach 통계. 사람이 눈으로 볼 이미지를 폴더에 저장한다.

사용: python3 data/bev/slab_check/make_sheet.py data/bev/slab_check/raws3
산출: <dir>/_sheet_review.png  (궤적 전체에 걸친 review.png 격자)
      <dir>/_sheet_stack.png   (한 샘플의 occupancy/visibility/review 나란히)
      <dir>/_stats.txt         (샘플별 z_ref·obstacle·visible·reach)
"""
import json
import pathlib
import sys

import numpy as np
import cv2
from PIL import Image

COLS, ROWS = 5, 3          # 컨택트시트 격자
TILE = 300                 # 타일 한 변[px]


def load(sd):
    m = json.loads((sd / "meta.json").read_text())
    occ = np.array(Image.open(sd / "occupancy.png"))
    vis = np.array(Image.open(sd / "visibility.png"))
    rev = cv2.imread(str(sd / "review.png"))
    return m, occ, vis, rev


def center_reach(vis, b, half_cols=5):
    """중앙축(|y|<0.25m)에서 보이는 가장 먼 전방 거리[m]와 가시 시작 거리[m].

    'ego 인접행부터 끊김 없는 사슬' 로 재면 안 된다 — 카메라가 수평을 봐서 반경 0.5m 는
    아무 카메라도 못 보므로 그 지표는 항상 0 이 나온다.
    """
    R, C, RES = b["R_EGO"], b["C_EGO"], b["RES"]
    col = vis[:R, C - half_cols:C + half_cols + 1].astype(bool).any(1)
    idx = np.flatnonzero(col)
    if not len(idx):
        return 0.0, 0.0
    return (R - idx.min()) * RES, (R - idx.max()) * RES


def main():
    d = pathlib.Path(sys.argv[1])
    sds = sorted(d.glob("sample_*"))
    if not sds:
        sys.exit(f"sample_* 없음: {d}")

    rows = []
    for sd in sds:
        m, occ, vis, _ = load(sd)
        far, near = center_reach(vis, m["bev"])
        rows.append({"sample": sd.name, "z_ref": m["z_ref"],
                     "obs": m["stats"]["obstacle_pct"],
                     "vis": m["stats"]["visible_pct"],
                     "cam_ok": m["stats"]["camera_ok_pct"],
                     "far": far, "near": near})

    lines = [f"{d.name}: {len(rows)} samples",
             "sample        z_ref   obs%   vis%  cam_ok%  reach_far  vis_start"]
    for r in rows:
        lines.append(f"{r['sample']}  {r['z_ref']:+.3f} {r['obs']:5.1f} {r['vis']:5.1f} "
                     f"{r['cam_ok']:7.1f} {r['far']:9.2f} {r['near']:9.2f}")
    far = np.array([r["far"] for r in rows])
    lines += ["",
              f"reach_far  중앙 {np.median(far):.2f}m  최소 {far.min():.2f}m  최대 {far.max():.2f}m",
              f"  4.0m 도달 {int((far >= 3.999).sum())}/{len(far)} "
              f"({(far >= 3.999).mean()*100:.0f}%)",
              f"  3.0m 이상 {int((far >= 3.0).sum())}/{len(far)} "
              f"({(far >= 3.0).mean()*100:.0f}%)",
              f"  1.0m 미만 {int((far < 1.0).sum())}/{len(far)}"]
    (d / "_stats.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines[-5:]))

    # 궤적 전체에 균등 표본한 review 컨택트시트
    sel = [sds[int(round(t))] for t in np.linspace(0, len(sds) - 1, COLS * ROWS)]
    sheet = np.zeros((ROWS * TILE, COLS * TILE, 3), np.uint8)
    for k, sd in enumerate(sel):
        m, _, vis, rev = load(sd)
        t = cv2.resize(rev, (TILE, TILE), interpolation=cv2.INTER_AREA)
        far, _ = center_reach(vis, m["bev"])
        cv2.putText(t, f"{sd.name[-3:]} reach{far:.1f}m", (5, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        r, c = divmod(k, COLS)
        sheet[r * TILE:(r + 1) * TILE, c * TILE:(c + 1) * TILE] = t
    cv2.imwrite(str(d / "_sheet_review.png"), sheet)

    # 중간 구간 한 샘플의 세 채널 나란히
    mid = sds[len(sds) // 2]
    m, occ, vis, rev = load(mid)
    B = 600

    def up(img, gray=False):
        if gray:
            img = np.stack([img * 255] * 3, -1).astype(np.uint8)
        return cv2.resize(img, (B, B), interpolation=cv2.INTER_NEAREST)

    occ_rgb = np.zeros(occ.shape + (3,), np.uint8)
    occ_rgb[occ == 0] = (40, 40, 220)      # obstacle 빨강(BGR)
    occ_rgb[occ == 1] = (0, 190, 0)        # drivable 초록
    panels = [up(occ_rgb), up(vis, gray=True), up(rev)]
    labels = [f"occupancy  {mid.name}", "visibility", "review(4color)"]
    stack = np.hstack(panels)
    for i, lab in enumerate(labels):
        cv2.putText(stack, lab, (i * B + 8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 0), 2, cv2.LINE_AA)
    cv2.imwrite(str(d / "_sheet_stack.png"), stack)
    print(f"\n저장: {d}/_sheet_review.png, _sheet_stack.png, _stats.txt")


if __name__ == "__main__":
    main()
