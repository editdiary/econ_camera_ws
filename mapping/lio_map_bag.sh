#!/bin/bash
# lio_map_bag.sh — bag을 Point-LIO로 매핑해 {map.pcd, trajectory.tum, run_info, preview} 생성
# 사용법: ./mapping/lio_map_bag.sh <bag_path> <out_dir> [--no-preview]
set -uo pipefail

BAG="${1:?사용법: $0 <bag_path> <out_dir> [--no-preview]}"
OUT="${2:?사용법: $0 <bag_path> <out_dir> [--no-preview]}"
PREVIEW=1
[ "${3:-}" = "--no-preview" ] && PREVIEW=0

WS="$(cd "$(dirname "$0")/.." && pwd)"
MAP_DIR="$WS/mapping"
PCD_SRC="$WS/src/point_lio/PCD/scans.pcd"

mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"

set +u   # ROS2 ament setup 스크립트는 set -u(unbound variable)와 호환되지 않음
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

echo "[lio_map_bag] bag=$BAG  out=$OUT"
rm -f "$PCD_SRC"   # 이전 실행 잔여 맵 제거

# 1) 궤적 로거 (백그라운드)
python3 "$MAP_DIR/pose_logger.py" "$OUT/trajectory.tum" &
LOGGER_PID=$!

# 2) 매핑 노드 (프로세스 그룹 분리)
setsid ros2 launch point_lio mapping_unilidar_l2.launch.py rviz:=false \
    > "$OUT/point_lio.log" 2>&1 &
NODE_PGID=$!

sleep 6   # 노드 초기화 대기

# 3) bag 재생 (블로킹)
echo "[lio_map_bag] 재생 시작..."
ros2 bag play "$BAG"
echo "[lio_map_bag] 재생 종료"
sleep 3

# 4) 매핑 노드 SIGINT → scans.pcd 저장
kill -INT -"$NODE_PGID" 2>/dev/null || kill -INT "$NODE_PGID" 2>/dev/null || true
for _ in $(seq 1 40); do kill -0 "$NODE_PGID" 2>/dev/null || break; sleep 1; done

# 5) 로거 종료
kill -INT "$LOGGER_PID" 2>/dev/null || true
wait "$LOGGER_PID" 2>/dev/null || true

# 6) 산출물 수집
if [ -f "$PCD_SRC" ]; then
    mv "$PCD_SRC" "$OUT/map.pcd"
else
    echo "[lio_map_bag] 경고: scans.pcd 없음 (맵 미생성)" >&2
fi

# 7) run_info
{
    echo "bag: $BAG"
    echo "git: $(git -C "$WS" rev-parse HEAD 2>/dev/null || echo unknown)"
    echo "poses: $(wc -l < "$OUT/trajectory.tum" 2>/dev/null || echo 0)"
    [ -f "$OUT/map.pcd" ] && echo "map_points: $(grep -a -m1 '^POINTS' "$OUT/map.pcd" | awk '{print $2}')"
} > "$OUT/run_info.txt"

# 8) 미리보기
if [ "$PREVIEW" = 1 ] && [ -f "$OUT/map.pcd" ]; then
    mkdir -p "$OUT/preview"
    python3 "$MAP_DIR/pcd_preview.py" "$OUT/map.pcd" "$OUT/preview" || true
    python3 "$MAP_DIR/bev_grid.py"   "$OUT/map.pcd" "$OUT/preview" || true
fi

echo "[lio_map_bag] 완료 → $OUT"
ls -la "$OUT"
