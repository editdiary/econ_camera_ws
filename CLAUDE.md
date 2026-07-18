# CLAUDE.md

이 저장소에서 작업할 때 참고할 핵심 사항. 상세 설계는 아래 spec을, 사용법은 `docs/USAGE.md`를 참조.

## 현재 상태 (2026-07-18)
카메라 4대 연속 동기 수집은 **구현 완료**(실기 4대 수동 검증만 남음). 구성:
- `capture`(`capture_node.py`): 단일 파이프라인 캡처 → `CompressedImage` 발행. **워밍업**
  (`warmup_s` 기본 4s: 프레임 폐기로 4대 정상 확인 후 첫 클린 사이클부터 발행 → 시작 프레임 수 일치).
  발행 병목 회피: `msg.data = array.array("B", data)` (bytes 대입 대비 2200배).
- `monitor`(`web_monitor_node.py`): **브라우저 2×2 MJPEG**(stdlib http.server, 기본 포트 10010).
  구독 전용·JPEG 패스스루라 **cv2 미사용**. 헤드리스/SSH 대응.
- `bag_extract.py`: bag(mcap) → 동기 세트별 JPEG(`frame_NNNNNN/cam{0..3}.jpg` + `sets.csv`).
  기본 동기 허용오차 **1ms**(frame_sync 실측 sub-ms).
- `record.launch.py`: `capture` + `ros2 bag record -s mcap`(카메라 4토픽만 명시 기록) 동시 기동.
- **캘리브레이션**(어안 4대 intrinsic + 카메라 간 extrinsic): Kalibr(arm64 Docker)로 실기 관통 검증 완료.
  도구 = `calibration/`(빌드·실행 스크립트, `aprilgrid.yaml`) + `kalibr_bridge`(세트→Kalibr 데이터셋)
  + `calib_convert`(camchain→`calib.yaml`). 절차·판정 기준·문제해결은 `docs/CALIBRATION.md`.
- 순수 로직 테스트 18개 통과(`cd src/econ_camera_ros && python3 -m pytest test/`).

## 프로젝트
e-con AR0234 4-camera 모듈용 **ROS2 연속 수집 패키지**. 4대를 하드웨어 동기(`frame_sync`)가
맞춰진 상태로 끊김 없이 캡처하여 **ROS2 bag(mcap)** 으로 저장한다. 런치 하나로 촬영 시작 →
`1280x720@30`(frame_sync=1) 4대 이미지를 `sensor_msgs/CompressedImage`(HW JPEG)로 계속 기록.
수집 데이터는 이후 딥러닝 학습에 사용. 향후 LiDAR를 같은 ws에 추가 예정(본 범위 밖).

- **선행 프로젝트**: `../Multi-Cam_module_test` (Flask 촬영·캘리브레이션 도구). 그 `econ_cam`
  패키지의 순수 로직(`controls`, `stats`)을 재사용한다.

## 하드웨어 핵심 사실
- 카메라 4대: `/dev/video0`~`/dev/video3` (e-con AR0234, `tegra-video` CSI)
- 포맷: **UYVY** 4:2:2 (모듈 내부 디베이어링 완료 → **ISP/Argus 미경유**, 순수 V4L2 경로)
- **Argus(libargus/nvarguscamerasrc/Isaac ROS Argus Camera) 사용 불가** — UYVY 직출력이라
  Argus가 카메라를 인식 못 함("No cameras available"). 촬영은 `v4l2src` 경로로만.
- 목표 수집: `1280x720@30` (frame_sync=1)
- 동기화: V4L2 `frame_sync` (`0/1/2` = Disable/30Hz/60Hz), `v4l2-ctl -c frame_sync=1 -d /dev/videoN`
- 플랫폼: Jetson AGX Orin, JetPack 6.1 / L4T R36.4 (Ubuntu 22.04)

## 기술 결정 (확정)
- **ROS2 = 호스트 네이티브 설치(Humble)**. Docker 미사용.
- **캡처+인코딩 = GStreamer** (Python `gi` + `appsink`, HW `nvvidconv`/`nvjpegenc`). 모니터는
  웹 MJPEG(JPEG 패스스루)라 cv2 미사용.
- **다중 동기 = 단일 파이프라인(4개 v4l2src, 공유 클럭)**. valve/tee 없이 연속 스트림.
  타임스탬프 = appsink 버퍼 PTS. 카메라 간 stamp 직접 비교 가능.
- **저장 = `sensor_msgs/CompressedImage`(JPEG)**, rosbag2 스토리지 **mcap**.
- **환경 = ROS2 Humble + colcon**. `econ_cam` 재사용은 `pip install -e ../Multi-Cam_module_test`.

## 작업 규칙
- 선행 프로젝트의 `econ_cam.controls`/`econ_cam.stats`는 **재사용**(복붙 금지, import).
  연속 수집 파이프라인 문자열은 기존 valve/tee 구조와 달라 **신규 작성**(`gst_builder.py`).
- 순수 로직(파이프라인 문자열 빌더)은 하드웨어 없이 `pytest`/`colcon test`로 검증. 실제
  캡처·동기·녹화는 4대에서 수동 검증.
- 파일은 관심사별로 작게 유지: `econ_camera_ros/{gst_builder,capture_node,web_monitor_node,bag_extract}.py`.

## Git 작업 방식
- **새 기능 추가·테스트는 항상 새 브랜치**에서 진행한다.
- **브랜치 병합·푸시는 사용자가 직접** 한다 — Claude는 새 브랜치 생성과 **커밋까지만** 수행.

## 상세 문서
- **사용 가이드**: `docs/USAGE.md` (녹화·모니터·bag 추출·파라미터·문제해결)
- **캘리브레이션 가이드**: `docs/CALIBRATION.md` (촬영법·Kalibr 실행·결과 판정·calib.yaml·문제해결)
- 설계 스펙(캘리브·검증): `docs/superpowers/specs/2026-07-18-camera-calibration-and-verification-design.md`
- 구현 계획(캘리브): `docs/superpowers/plans/2026-07-18-camera-calibration-and-verification.md`
- 설계 스펙(카메라 수집): `docs/superpowers/specs/2026-07-16-econ-camera-ros2-capture-design.md`
- 설계 스펙(수집 시스템 & bag 녹화 주의사항, Camera+LiDAR→BEV):
  `docs/superpowers/specs/2026-07-18-data-collection-bag-and-fusion-design.md`
- 구현 계획: `docs/superpowers/plans/2026-07-16-econ-camera-ros2-capture.md`
