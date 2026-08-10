#!/bin/bash
# build.sh — 후처리 이미지 빌드(최초 1회, Dockerfile 수정 시 재실행).
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${ECON_IMAGE:-econ-proc:humble}"

# 빌드 컨텍스트는 docker/ 만. 저장소 전체(data/ 66GB 포함)를 컨텍스트로 보내지 않기 위함.
exec sudo docker build -t "$IMAGE" "$WS/docker"
