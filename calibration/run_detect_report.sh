#!/usr/bin/env bash
# Kalibr 자체 검출기로 이미지별 코너 수 리포트(+필터). run_kalibr.sh 와 동일한 docker 우회.
# 어떤 프레임이 검출 부족으로 캘리브를 죽이는지(예: DLT 최소 6점 미달) 콕 집어낸다.
#
# 사용:
#   sudo bash calibration/run_detect_report.sh <DATA_DIR>                    # 리포트 + 기본 필터(<8코너 이동)
#   sudo bash calibration/run_detect_report.sh <DATA_DIR> --report-only      # 리포트만(파일 안 옮김)
#   sudo bash calibration/run_detect_report.sh <DATA_DIR> --min-corners 6    # 임계값 지정(크래시 경계=6)
#   → 필터 후 sudo bash calibration/run_kalibr.sh <DATA_DIR> 로 재캘리브
#
# 주의: run_kalibr.sh 와 동일 — docker sudo 필요, iptables raw 없어 --network=host,
#       shell-form ENTRYPOINT 우회 위해 --entrypoint bash.
set -euo pipefail

DATA_DIR="$(realpath "${1:?usage: run_detect_report.sh <DATA_DIR> [--min-corners N] [--report-only]}")"
shift || true
EXTRA=("$@")   # kalibr_detect_report.py 로 그대로 전달할 옵션들

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LAYOUT="$SCRIPT_DIR/../src/econ_camera_ros/econ_camera_ros/cam_layout.py"

# orientation.json 이 있으면 run_kalibr.sh 와 같은 링 순서 토픽을 사용(일관성).
if [ -f "$DATA_DIR/orientation.json" ]; then
  TOPICS="$(python3 "$LAYOUT" "$DATA_DIR/orientation.json" --topics)"
  echo "orientation.json 적용 → 토픽 순서: $TOPICS"
else
  TOPICS="/cam0/image_raw /cam1/image_raw /cam2/image_raw /cam3/image_raw"
  echo "orientation.json 없음 → 기본 순서 cam0~3"
fi

[ -d "$DATA_DIR/dataset" ]        || { echo "ERROR: $DATA_DIR/dataset 없음 (kalibr_bridge 먼저 실행)"; exit 1; }
[ -f "$DATA_DIR/aprilgrid.yaml" ] || { echo "ERROR: $DATA_DIR/aprilgrid.yaml 없음"; exit 1; }

docker run --rm --network=host -e KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 \
  -v "$DATA_DIR:/data" -v "$SCRIPT_DIR:/calib_scripts:ro" --entrypoint bash kalibr:arm64 -c "
  set -e
  source /catkin_ws/devel/setup.bash
  # 검출은 calib.bag 을 읽으므로 없으면 생성(있으면 재사용).
  [ -f /data/calib.bag ] || rosrun kalibr kalibr_bagcreater --folder /data/dataset --output-bag /data/calib.bag
  PY=\$(command -v python3 || command -v python)
  \$PY /calib_scripts/kalibr_detect_report.py \
    --bag /data/calib.bag --target /data/aprilgrid.yaml --dataset /data/dataset \
    --topics $TOPICS --out /data/detect_report.csv ${EXTRA[*]}
"
echo "DONE: $DATA_DIR/detect_report.csv (이미지별 코너 수)"
