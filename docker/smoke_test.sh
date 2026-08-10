#!/bin/bash
# 후처리 Docker 환경 스모크 테스트 — 실데이터로 전 경로를 한 번씩 돌린다.
#   ./docker/smoke_test.sh 2>&1 | tee /tmp/smoke.log
# 이미지 빌드(./docker/build.sh) 후 실행. 산출물은 data/_archive/docker_smoke/ (검증용, 버려도 됨).
# 각 항목은 [PASS]/[FAIL] 로 표시되며, 실패해도 중단하지 않고 끝까지 돈다.
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" || exit 1
OUT=data/_archive/docker_smoke
mkdir -p "$OUT"
R() { echo; echo "======== $* ========"; }
ok() { if [ $? -eq 0 ]; then echo "[PASS] $1"; else echo "[FAIL] $1"; fi; }

R "0. 인터프리터/패키지 버전"
./docker/run.sh python3 -c '
import sys, importlib
print("python", sys.version.split()[0])
for m in ["numpy","cv2","scipy","open3d","yaml","PIL","mcap","rclpy","rosbag2_py"]:
    try:
        mod = importlib.import_module(m)
        v = getattr(mod, "__version__", "?")
        print(f"{m:12s} OK   {v}")
    except Exception as e:
        print(f"{m:12s} FAIL {type(e).__name__}: {e}")
'
ok "imports"

R "1. open3d 로 map.pcd 읽기"
./docker/run.sh python3 -c '
import open3d as o3d, numpy as np
p = o3d.io.read_point_cloud("data/sj_bags/260722/maps/raws1_mapping/map.pcd")
a = np.asarray(p.points)
print("points", a.shape, "bbox", a.min(0).round(2), a.max(0).round(2))
assert len(a) > 1000
'
ok "open3d read_point_cloud"

R "2. mapping/pcd_preview.py"
./docker/run.sh python3 mapping/pcd_preview.py data/sj_bags/260722/maps/raws1_mapping/map.pcd "$OUT/pcd_preview"
ok "pcd_preview"

R "3. mapping/bev_grid.py"
./docker/run.sh python3 mapping/bev_grid.py data/sj_bags/260722/maps/raws1_mapping/map.pcd "$OUT/bev_grid"
ok "bev_grid"

R "4. calib 시각 검증 (verify_undistort)"
./docker/run.sh python3 calibration/verify/verify_undistort.py \
    --images data/calib_260723/extracted --frames "800" --out "$OUT/verify_undistort"
ok "verify_undistort"

R "5. BEV auto-label 단계1 (verify_labels) — open3d+cv2+scipy+DS 전 경로"
./docker/run.sh bash -c 'cd calibration/bev_autolabel && python3 verify_labels.py \
    --map-dir ../../data/sj_bags/260722/maps/raws1_mapping \
    --extract-dir ../../data/extracted/raws1 \
    --calib ../../data/calib_260723/calib.yaml \
    --orient ../../data/calib_260723/orientation.json \
    --frames 900 2000 --out ../../data/_archive/docker_smoke/bev_review'
ok "verify_labels"

R "6. BEV auto-label 단계2 (generate --limit 2) — IPM 포함"
./docker/run.sh bash -c 'cd calibration/bev_autolabel && python3 generate.py \
    --map-dir ../../data/sj_bags/260722/maps/raws1_mapping \
    --extract-dir ../../data/extracted/raws1 \
    --calib ../../data/calib_260723/calib.yaml \
    --orient ../../data/calib_260723/orientation.json \
    --out ../../data/_archive/docker_smoke/bev_dataset --limit 2'
ok "generate"

R "7. BEV auto-label 단계3 (gather_annotations)"
./docker/run.sh python3 calibration/bev_autolabel/gather_annotations.py \
    --dataset "$OUT/bev_dataset" --out "$OUT/bev_annotations"
ok "gather_annotations"

R "8. bag 읽기 — bag_extract (mcap 플러그인)"
./docker/run.sh python3 src/econ_camera_ros/econ_camera_ros/bag_extract.py \
    data/sj_bags/260722/bags/record-all_with-sun_1 -o "$OUT/bag_extract" --limit 3
ok "bag_extract"

R "9. bag 읽기 — check_recording (mcap 파이썬 라이브러리)"
./docker/run.sh python3 tools/check_recording.py data/sj_bags/260722/bags/record-all_with-sun_1
ok "check_recording"

R "10. LiDAR bag 사전점검 (check_lidar_bag)"
./docker/run.sh python3 mapping/check_lidar_bag.py \
    data/sj_bags/260722/bags/record-all_with-sun_1 --out "$OUT/check_lidar" --accum-sec 2
ok "check_lidar_bag"

R "11. 순수 로직 테스트"
./docker/run.sh bash -c 'cd src/econ_camera_ros && python3 -m pytest test/ -q'
ok "pytest econ_camera_ros"
./docker/run.sh bash -c 'cd calibration/bev_autolabel && python3 -m pytest . -q'
ok "pytest bev_autolabel"
./docker/run.sh bash -c 'cd calibration/cam_lidar && python3 -m pytest . -q'
ok "pytest cam_lidar"
./docker/run.sh bash -c 'cd calibration/verify && python3 -m pytest . -q'
ok "pytest verify"
./docker/run.sh bash -c 'cd mapping && python3 -m pytest test/ -q'
ok "pytest mapping"

R "12. 산출물 소유권 (root 면 실패)"
find "$OUT" -newermt '-2 hours' \! -user "$(id -un)" -printf '%u %p\n' | head -20
echo "--- 소유권 샘플 ---"
ls -l "$OUT" | head -10

echo
echo "======== 스모크 테스트 종료 ========"
