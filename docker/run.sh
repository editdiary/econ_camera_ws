#!/bin/bash
# run.sh — 후처리 컨테이너에서 명령 실행. 인자 없으면 대화형 bash.
#
#   ./docker/run.sh                                   # 셸
#   ./docker/run.sh python3 mapping/pcd_preview.py data/.../map.pcd
#   ./docker/run.sh bash -c 'cd calibration/bev_autolabel && python3 generate.py ...'
#
# 이미지 빌드(최초 1회): ./docker/build.sh
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${ECON_IMAGE:-econ-proc:humble}"

# --user 로 호스트 uid/gid 를 넘기는 게 핵심. sudo docker 로 실행하므로 이게 없으면
# data/ 아래 산출물이 전부 root 소유로 떨어져 호스트에서 손대지 못한다.
# 컨테이너에 해당 uid 의 계정이 없어 HOME 이 없으므로 /tmp 로 지정(캐시 기록용).
ARGS=(--rm --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$WS:/ws" -w /ws)

# TTY 가 없는 곳(스크립트·CI)에서 -it 를 주면 실패하므로 조건부.
[ -t 0 ] && [ -t 1 ] && ARGS+=(-it)

exec sudo docker run "${ARGS[@]}" "$IMAGE" "$@"
