#!/usr/bin/env bash
# dataset/ + aprilgrid.yaml 을 컨테이너에 마운트해 bag 생성 후 ds/eucm 2회 실행.
# 사용: sudo bash calibration/run_kalibr.sh <DATA_DIR>
#   DATA_DIR 안에 dataset/(cam0..3/<ns>.png) 와 aprilgrid.yaml 이 있어야 함.
# 주의:
#   - docker 는 현재 계정이 docker 그룹 밖이라 sudo 필요.
#   - 이 Jetson 커널엔 iptables 'raw' 테이블 모듈이 없어 기본 브리지 네트워크가 실패한다.
#     Kalibr 는 오프라인이라 네트워크가 불필요하므로 --network=host 로 우회.
#   - Kalibr 이미지의 ENTRYPOINT 는 shell-form 이라 넘긴 명령을 무시한다 → --entrypoint bash 로
#     덮어써야 아래 스크립트가 실행된다. 원 엔트리포인트가 주던 KALIBR_MANUAL_FOCAL_LENGTH_INIT
#     (어안 초점거리 자동 초기화 실패 대비)은 -e 로 직접 전달.
set -euo pipefail

DATA_DIR="$(realpath "${1:?usage: run_kalibr.sh <DATA_DIR>}")"
WS="/catkin_ws"   # 컨테이너 내 catkin workspace (빌드 로그로 확인됨)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LAYOUT="$SCRIPT_DIR/../src/econ_camera_ros/econ_camera_ros/cam_layout.py"

# orientation.json 이 있으면 링 순서(front-right-rear-left)로 토픽 정렬(포트 재연결로 cam
# 번호가 바뀌어도 인접 카메라가 --topics 순서상 연속되게). 없으면 기존 cam0~3 순서.
if [ -f "$DATA_DIR/orientation.json" ]; then
  TOPICS="$(python3 "$LAYOUT" "$DATA_DIR/orientation.json" --topics)"
  echo "orientation.json 적용 → Kalibr 토픽 순서: $TOPICS"
else
  TOPICS="/cam0/image_raw /cam1/image_raw /cam2/image_raw /cam3/image_raw"
  echo "orientation.json 없음 → 기본 순서 cam0~3 사용"
fi

# 사전 검증: 입력이 없으면 컨테이너 안에서 불투명하게 실패하므로 미리 거른다.
[ -d "$DATA_DIR/dataset" ]        || { echo "ERROR: $DATA_DIR/dataset 없음 (kalibr_bridge 먼저 실행)"; exit 1; }
[ -f "$DATA_DIR/aprilgrid.yaml" ] || { echo "ERROR: $DATA_DIR/aprilgrid.yaml 없음 (calibration/aprilgrid.yaml 복사)"; exit 1; }
# 이전 실행의 bag 이 남아 있으면 bagcreater 가 실패할 수 있어 제거.
rm -f "$DATA_DIR/calib.bag"

docker run --rm --network=host -e KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 \
  -v "$DATA_DIR:/data" --entrypoint bash kalibr:arm64 -c "
  set -e   # 컨테이너 내부에서도 실패 시 즉시 중단(부분 실패 후 DONE 오인 방지)
  source $WS/devel/setup.bash
  rosrun kalibr kalibr_bagcreater --folder /data/dataset --output-bag /data/calib.bag

  # Kalibr 모델명은 ds-none / eucm-none (도움말의 유효 목록). LABEL 은 파일 접미사용 축약.
  for M in ds-none eucm-none; do
    LABEL=\${M%-none}
    rosrun kalibr kalibr_calibrate_cameras \
      --bag /data/calib.bag \
      --target /data/aprilgrid.yaml \
      --models \$M \$M \$M \$M \
      --topics $TOPICS \
      --approx-sync 0.001 \
      --dont-show-report
    # 모델별 산출물이 덮이지 않게 접미사 부여. base 파일명만 정확히 지정
    # (와일드카드로 하면 앞 모델의 이미 접미사 붙은 파일까지 다시 잡혀 이중 접미사가 됨).
    for base in calib-camchain.yaml calib-results-cam.txt calib-report-cam.pdf; do
      [ -e \"/data/\$base\" ] && mv \"/data/\$base\" \"/data/\${base%.*}-\$LABEL.\${base##*.}\"
    done
  done
"
echo "DONE: /data 에 ds/eucm camchain·results·report 생성."
if [ -f "$DATA_DIR/orientation.json" ]; then
  echo "── Kalibr cam 인덱스 ↔ 방향 (results.txt 대조 / calib_convert --rms 입력용) ──"
  python3 "$LAYOUT" "$DATA_DIR/orientation.json" --map | sed 's/^/  /'
fi
