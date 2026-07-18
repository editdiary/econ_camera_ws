#!/usr/bin/env bash
# Kalibr(ROS1)를 arm64 네이티브로 빌드한다. Jetson(aarch64)에서 실행.
# 사용: bash calibration/build_kalibr_arm64.sh [KALIBR_CLONE_DIR]
set -euo pipefail

# sudo 실행 시 $HOME 이 /root 로 바뀌므로 원래 사용자 홈으로 클론(중복 클론 방지).
KALIBR_DIR="${1:-/home/${SUDO_USER:-$USER}/Desktop/kalibr}"

if [ ! -d "$KALIBR_DIR/.git" ]; then
  git clone https://github.com/ethz-asl/kalibr.git "$KALIBR_DIR"
fi
cd "$KALIBR_DIR"

# 베이스 이미지 교체: 실제 Dockerfile 은 osrf/ros:noetic-desktop-full(arm64 미제공) 사용.
# arm64 네이티브 ros-base(arm64v8/ros:noetic)로 바꿔 Jetson 네이티브 빌드.
sed -i 's|^FROM osrf/ros:noetic-desktop-full|FROM arm64v8/ros:noetic|' Dockerfile_ros1_20_04

# arm64 ros-base 에는 desktop-full 이 주던 vision 스택(cv_bridge 등)이 없다. Kalibr 가
# import cv_bridge 하므로 apt 목록에 보충($ 앵커로 멱등: 두 번 실행해도 중복 추가 안 됨).
sed -i 's|python3-osrf-pycommon$|python3-osrf-pycommon ros-noetic-cv-bridge ros-noetic-image-geometry ros-noetic-image-transport|' Dockerfile_ros1_20_04

# 빌드(컴파일 양 많음 — 수십 분 소요 가능). docker 그룹 밖이면 sudo 필요.
# --network=host: 이 Jetson 커널엔 iptables 'raw' 테이블 모듈이 없어 기본 브리지 네트워크
#   엔드포인트 생성이 실패한다. host 네트워크로 우회.
docker build --network=host -t kalibr:arm64 -f Dockerfile_ros1_20_04 .

# 패치 레이어 적용: cv2 선로딩 .pth (cv_bridge_boost import 순서 이슈 해결).
# 결과를 다시 kalibr:arm64 로 태깅 → 한 번의 스크립트로 바로 쓸 수 있는 이미지 완성.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
docker build --network=host -t kalibr:arm64 -f "$SCRIPT_DIR/Dockerfile.patch" "$SCRIPT_DIR"

echo "DONE: image 'kalibr:arm64' built (vision 스택 + cv2 선로딩 포함)."
