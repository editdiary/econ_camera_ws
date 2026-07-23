"""카메라 번호(cam0~3) ↔ 물리 방향 매핑으로 Kalibr 토픽 순서를 링 순서로 정렬.

360° 서라운드 어안 4대는 인접(90° 이웃)만 겹치고 반대편(180°)은 안 겹친다.
Kalibr 다중 카메라 캘리브레이션은 --topics 순서상 연속된 쌍이 겹친다고 가정하므로,
토픽을 물리 인접 링 순서(front-right-rear-left)로 넘겨야 extrinsic 이 풀린다.

orientation.json = {"cam0":"left", "cam1":"right", "cam2":"front", "cam3":"rear"}
(모니터 커버 테스트로 각 cam 의 방향을 확인해 수동 작성).

순수 로직(order_from_json)만 단위 테스트하고, CLI 출력은 run_kalibr.sh 에서 수동 검증.

실행:
    python3 -m econ_camera_ros.cam_layout orientation.json --topics
    python3 -m econ_camera_ros.cam_layout orientation.json --map
"""

import argparse
import json

RING = ["front", "right", "rear", "left"]   # 인접 순서(반대편 미겹침), front = extrinsic 기준
_CAMS = ["cam0", "cam1", "cam2", "cam3"]


def order_from_json(orientation):
    """{"cam0":"left", ...} → 링 순서 [(cam_idx, direction), ...]. 검증 포함."""
    if sorted(orientation) != _CAMS:
        raise ValueError(f"orientation 키는 정확히 {_CAMS} 여야 함 (받음: {sorted(orientation)})")
    dirs = [orientation[c] for c in _CAMS]
    if sorted(dirs) != sorted(RING):
        raise ValueError(f"orientation 값은 {RING} 각 1개씩이어야 함 (받음: {dirs})")
    dir_to_idx = {orientation[c]: int(c[3:]) for c in _CAMS}
    return [(dir_to_idx[d], d) for d in RING]


def main():
    p = argparse.ArgumentParser(description="orientation.json → Kalibr 토픽 순서")
    p.add_argument("orientation", help="cam↔방향 매핑 json")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--topics", action="store_true", help="링 순서 /camN/image_raw 목록")
    g.add_argument("--map", action="store_true", help="Kalibr cam인덱스 ↔ 방향 ↔ 토픽 표")
    a = p.parse_args()

    with open(a.orientation) as f:
        order = order_from_json(json.load(f))
    if a.topics:
        print(" ".join(f"/cam{idx}/image_raw" for idx, _ in order))
    else:
        for i, (idx, d) in enumerate(order):
            print(f"Kalibr cam{i} = {d} (/cam{idx}/image_raw)")


if __name__ == "__main__":
    main()
