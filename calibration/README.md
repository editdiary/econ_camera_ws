# Kalibr 캘리브레이션 (arm64 Docker)

Jetson(aarch64)에서 Kalibr(ROS1)를 네이티브 Docker로 빌드·실행한다.

## 빌드
    sudo bash calibration/build_kalibr_arm64.sh [클론경로(기본 ~/Desktop/kalibr)]
- 결과 이미지: `kalibr:arm64` (vision 스택 + cv2 선로딩 포함)
- 컨테이너 내 catkin workspace: `/catkin_ws` (스모크 테스트로 확인된 값; 다르면 여기 수정)
- `sudo`: 계정이 docker 그룹 밖이면 필요.

## 스모크 테스트
    sudo docker run --rm --network=host -e KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 \
      --entrypoint bash kalibr:arm64 \
      -c 'source /catkin_ws/devel/setup.bash && rosrun kalibr kalibr_calibrate_cameras --help'
- `--network=host`(iptables raw 부재 우회) / `--entrypoint bash`(shell-form ENTRYPOINT 우회)는 필수.
  자세한 이유는 `docs/CALIBRATION.md` §8 문제해결 참조.

## 타깃
- `calibration/aprilgrid.yaml` (7x5, tagSize 0.04, tagSpacing 0.25)

## 실행
- 데이터 준비·실행 순서는 `docs/CALIBRATION.md` 참조.
