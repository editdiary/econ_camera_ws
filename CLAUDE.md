# CLAUDE.md

이 저장소에서 작업할 때 참고할 핵심 사항. 상세 설계는 아래 spec을, 사용법은 `docs/USAGE.md`를 참조.

## 현재 상태 (2026-08-10)
카메라 4대 연속 동기 수집은 **구현 완료**(실기 4대 수동 검증만 남음). 구성:
- `capture`(`capture_node.py`): 단일 파이프라인 캡처 → `CompressedImage` 발행. **워밍업**
  (`warmup_s` 기본 4s: 프레임 폐기로 4대 정상 확인 후 첫 클린 사이클부터 발행 → 시작 프레임 수 일치).
  발행 병목 회피: `msg.data = array.array("B", data)` (bytes 대입 대비 2200배).
- `monitor`(`web_monitor_node.py`): **브라우저 2×2 MJPEG**(stdlib http.server, 기본 포트 10010).
  구독 전용·JPEG 패스스루라 **cv2 미사용**. 헤드리스/SSH 대응.
- `bag_extract.py`: bag(mcap) → 동기 세트별 JPEG(`frame_NNNNNN/cam{0..3}.jpg` + `sets.csv`).
  기본 동기 허용오차 **1ms**(frame_sync 실측 sub-ms).
- `tools/check_recording.py`: 녹화 bag의 **프레임 타임스탬프 간격**으로 FPS·끊김·4대 정렬
  판정(ROS 불필요, `pip install mcap`). 녹화 성공 판정은 모니터 화면이 아니라 이 도구로 한다.
- `record.launch.py`: `capture` + `ros2 bag record -s mcap`(카메라 4토픽만 명시 기록) 동시 기동.
- `record_all.launch.py`: `capture` + **LiDAR**(`unitree_lidar_ros2`) + `ros2 bag record -s mcap`
  (카메라4 + `/unilidar/cloud`·`/unilidar/imu` + `/tf`·`/tf_static`, 8토픽) 동시 기동.
- `record_lidar.launch.py`: 카메라 없이 **LiDAR만** 녹화(`/unilidar/*`+`/tf`·`/tf_static`, 4토픽).
- **LiDAR**(Unitree 4D L2): `src/unitree_lidar_ros2`(벤더 패키지) + `third_party/unitree_lidar_sdk`
  (prebuilt SDK). 이더넷 UDP(호스트 `192.168.1.2/24`). 절차·검증은 `docs/LIDAR.md`. 실기 검증만 남음.
- **오프라인 매핑**(bag→궤적·맵): Point-LIO(ROS2) 벤더링 `src/point_lio` + `mapping/`
  (`lio_map_bag.sh`(`--max-secs`로 재생 구간 제한)·`pose_logger.py`·`pcd_preview.py`·`bev_grid.py`·
  `check_lidar_bag.py`(매핑 전 `/unilidar/cloud` 사전점검+BEV PNG)). `ros2 bag play` 기반
  후처리라 **실시간 수집·녹화 코드와 완전 분리**. 산출물 = `map.pcd`+`trajectory.tum`(ego-pose,
  BEV 전제)+미리보기. 절차는 `docs/MAPPING.md`.
  **뒷부분 잘림**: 재생은 realtime인데 노드가 못 따라가면 bag 뒤가 경고 없이 통째로 누락된다.
  지금은 궤적이 안 커질 때까지 기다린 뒤 노드를 내린다(`DRAIN_MAX`, 기본 180s). 판정은
  `run_info.txt`의 `traj_span_s` ≈ bag 길이.
  **고립 노이즈 제거**: `mapping/pcd_denoise.py <out>/map.pcd` → 같은 폴더에 `map_clean.pcd`
  (원본 보존). k-NN `d4>0.3m`인 **고립점만** 제거(실측 0.05~0.10%). 500점+ 대형 분리군집은
  실구조물(raws3의 24,158점=천장)이라 "최대 군집만 남기기" 금지. 바닥은 희소해 우선 삭제되므로
  `--protect-below p1` 아래는 임계를 3배 완화. PCD 바이너리 직접 read/write로 intensity 등 8필드
  보존(open3d 경유 시 소실). `lio_map_bag.sh`가 자동 실행하지 않으니 따로 돌린다. `docs/MAPPING.md §6.6`.
  **self mask**: `map.pcd`는 필터 없는 누적 원장(동적물체 제거·carving 전무)이라 카트를 끄는
  수집자가 그대로 적립되고, 사람이 카트 뒤 0.85m를 따라오므로 **지나간 경로가 사람으로 덧칠**된다.
  `pcd_save.self_mask_*`(unilidar_l2.yaml, 기본 ON: **축정렬 박스** x −1.5~−0.45·|y|<0.35·z<1.0)로
  제거(잔재 −62%). **정합(ikd-Tree)에는 남기고 `map.pcd` 누적에서만 뺀다** — 라이다가 바닥을 못 봐
  수직이 약한데 사람 점이 높이 기준 역할을 해서, 전처리에서 빼면 z 드리프트가 +0.19→+0.67m로
  악화된다(재현 확인). **부채꼴 금지** — 통로가 좁아 좌우 0.33~0.58m에 실제 구조물이 있어 벽을
  갉아먹는다. 같은 이유로 `blind` 키우기도 금지. 판정은 `mapping/check_self_points.py`
  (궤적 거리 아닌 pose 바디프레임 박스 카운트, `--png`로 y-z 단면 검수뷰). 근거는 `docs/MAPPING.md §6.5`.
  **260722 재매핑 완료(2026-08-10)** → `data/sj_bags/260722/maps_selfmask/`(7종, `map_clean.pcd` 포함).
  잘림 0/7, 궤적 40.9~44.0m로 수렴, 코어 잔재 −61~−96%, 구 maps/에서 붕괴했던 rawos1·rawos3 복구.
  **하류는 이 폴더를 쓴다.** 구 `data/sj_bags/260722/maps/`는 대조용 보관(`lidaronly_mapping`은 거기만 있음).
  잔여 이슈: rawos 4종의 선형 z 경사(−0.9~−1.9m). 대부분 강체 기울기라 ego-local BEV 크롭에는
  불일치분 ≈6cm만 남아 비치명적 — 라벨 생성 단계에서 처리. 상세 `docs/MAPPING.md §6.7`.
- **캘리브레이션**(어안 4대 intrinsic + 카메라 간 extrinsic): Kalibr(arm64 Docker)로 실기 관통 검증 완료.
  도구 = `calibration/`(빌드·실행 스크립트, `aprilgrid.yaml`) + `kalibr_bridge`(세트→Kalibr 데이터셋)
  + `calib_convert`(camchain→`calib.yaml`). 절차·판정 기준·문제해결은 `docs/CALIBRATION.md`.
  `calib.yaml` **시각 검증**(언디스토션·360° 파노라마·카메라 간 겹침, 검출 불필요·호스트 파이썬)은
  `calibration/verify/`(`docs/CALIBRATION.md §6.5`). calib 이미지·직접 수집 이미지 모두 적용.
- **Cam-LiDAR extrinsic**(`T_front_lidar`, 라이다→front): 도구 구현 완료(`calibration/cam_lidar/`,
  수동 2D-3D 대응점 클릭 + DS-PnP). 카메라 간 extrinsic 체인에 라이다를 한 단으로 붙인다.
  절차·판정 기준은 `docs/CAM_LIDAR_CALIBRATION.md`. 순수 로직 테스트 통과, 실기(정지 촬영·
  대응점 클릭·solve RMS·오버레이 검증)만 남음.
- **BEV auto-label**(3어안→BEV occupancy 학습 정답 자동생성): 파이프라인 CLI 구현 완료
  (`calibration/bev_autolabel/`: 수직성 obstacle+국소floor / 카메라 FoV∩가림 observed / 정밀 ego(0.28m) /
  corridor 무조건 drivable). 단계1 `verify_labels.py`(검수 PNG) + 단계2 `generate.py`(LiDAR 라벨 **+ 3어안 IPM
  지면투영 RGB 캔버스 + 라벨 오버레이 검수뷰** 일괄 생성, `ipm.py`). LiDAR가 바닥을 못 보므로 바닥 모습은 IPM으로 보완.
  **사람은 카메라 마스킹 없이 IPM 배경 위 라벨을 BEV에서 보정만** 함 → 옛 마스킹 경로(`ipm_review`·`dataset_flatten`) 폐기.
  각 sample에 `overlay.png`(ipm_rgb+라벨 오버레이, **네이티브 해상도·장식 없음**=CVAT 라벨링 base, resize 왕복 없음) 저장.
  단계3 `gather_annotations.py`(dataset→`data/bev/annotations/<name>/` 하나에 하위 `label/`(CVAT 업로드용, 해상도는 dataset `meta.json` 따름)·
  `review/`(참고용 확대 검수뷰, 원본 3어안+BEV, `--review-scale` 기본18)로 나눠 모음). 격자·ego 등 장식은 review로만(base엔 없음).
  raws3 + 타 bag 4종 검증. 실행법·변경사항은 `docs/BEV_AUTOLABEL.md §A`. 순수 테스트 34개.
  **BEV 범위는 CLI 옵션**(`--xf/--xr/--yh`, 기본 3.0/1.0/2.0 = 80×80; `RES`=0.05 고정) — 5m×5m는 `--xf 3.5 --xr 1.5 --yh 2.5`.
  국소 floor 윈도는 **ego 미터좌표에 고정**(격자 인덱스 기준이면 범위를 옮길 때 바닥 추정이 튀어 허위 obstacle 발생).
  IPM 다중카메라 합성은 기본 `nearest`(셀별 최근접 1대, 겹침 유령상 감소)·`--blend average` 선택 가능.
  **슬래브 라벨(LiDAR 라벨 현행판)**: `slab_label.py`+`slab_io.py`+`slab_render.py`+`generate_slab.py`
  +`gather_slab.py`(단계3 대응. **`gather_annotations.py`는 슬래브에 안 먹는다** — `label.png`를 요구하는데 없다.
  `overlay.png`=**obstacle만** 얹은 CVAT base 라 합성 없이 바이트 그대로 복사해 모은다. 보정 대상은 occupancy 하나뿐,
  visibility 는 보정본 raycast 로 재생성)
  → `data/bev/slab/<name>/sample_NNNNNN/{slab.pcd,occupancy.png,visibility.png,ipm_rgb.png,overlay.png,
  review.png,cam_{front,left,right}.jpg,meta.json}`. **기본 `--xf 4.0 --xr 2.0 --yh 3.0` = 120×120**
  — §A(`generate.py`)의 80×80과 **다른 그리드**이니 한 데이터셋에 섞지 말 것.
  map_clean.pcd 에서 body 프레임 3D crop → 하위1% z 부터 0.8m 슬래브 → 2D 기둥 count≥3
  occupancy + (2D raycast ∧ 카메라 관측가능성 ∧ ¬self박스) visibility. 옛 `label.png`(0/1/2) 대체.
  카메라가 수평을 봐서 실제 지면에서 반경 0.5m 완전 사각·1.0m 부분 사각이다. 카트 자기 가림은
  둘로 나눠 처리: **상판·받침판은 이미지 마스크**(`data/calib_260723/self_mask/`, 클래스 색 PNG,
  기본 `table` 만) + **손잡이·수집자는 body 프레임 self 박스**(기본 x −2.1~−0.4·|y|≤0.7, 896셀)
  — 사람의 이미지 위치가 프레임마다 달라 정적 마스크로는 못 맞히기 때문. 어안 원 바깥은 자동 검출.
  금지: 3D raycast·min_pts≥10·pct=5·corridor prior 부활·ground_offset=0·handle/human 을
  self-mask-classes 에 넣기. `docs/BEV_AUTOLABEL.md §B`.
- 순수 로직 테스트 25개 통과(`cd src/econ_camera_ros && python3 -m pytest test/`).
- **폴더**: 수집 bag·추출 이미지·캘리브/LIO 산출물 등 모든 데이터·산출물은 `data/`(gitignore)
  한 곳으로 모은다. 하위 구조:
  - `data/sj_bags/<날짜>/{bags,maps_selfmask}/` — 현장 원본 bag(`bags/`) + 그 bag의 Point-LIO 산출
    (`maps_selfmask/<name>_mapping/` = self mask·drain 적용 현행판, `map.pcd`+`map_clean.pcd`+`trajectory.tum`).
    260722에는 구버전 `maps/`도 대조용으로 남아 있다.
  - `data/extracted/<name>/` — bag별 추출 이미지(`frame_NNNNNN/cam{0..3}.jpg`+`sets.csv`). `<name>`: `raws{N}`=with-sun / `rawos{N}`=without-sun. bag↔map↔extracted를 같은 `<name>`으로 짝짓는다.
  - `data/calib_260723/`(cam-cam 캘리브)·`data/cam-lidar_calib_260724/`(cam-LiDAR 캘리브)·`data/bev/{review,dataset,annotations}/`(BEV 검수뷰·데이터셋·CVAT 업로드 묶음)·`data/_archive/`(폐기·임시 모음).
  `third_party/point_lio_unilidar`(upstream 원본 클론)는 빌드에 안 쓰이며
  (매핑은 `src/point_lio` 사용) gitignore 처리됨.

## 프로젝트
e-con AR0234 4-camera 모듈용 **ROS2 연속 수집 패키지**. 4대를 하드웨어 동기(`frame_sync`)가
맞춰진 상태로 끊김 없이 캡처하여 **ROS2 bag(mcap)** 으로 저장한다. 런치 하나로 촬영 시작 →
`1280x720@30`(frame_sync=1) 4대 이미지를 `sensor_msgs/CompressedImage`(HW JPEG)로 계속 기록.
수집 데이터는 이후 딥러닝 학습에 사용. LiDAR(Unitree L2, IMU 포함)도 같은 ws에 통합되어
카메라와 단일 bag으로 함께 수집한다(자세히는 `docs/LIDAR.md`).

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
- **전체 파이프라인(순차 따라하기)**: `docs/PIPELINE.md` (데이터 수집→매핑→캘리브→LiDAR/IPM auto-label→최종 검수까지 단계별 실행·옵션·산출물, 상세 문서 링크 허브)
- **서버 후처리 환경(Docker)**: `docs/DOCKER.md` (수집 완료 데이터를 서버에서 가공. 단일 이미지 `econ-proc:humble`, colcon 빌드 불필요, `docker/{build,run,smoke_test}.sh`)
- **사용 가이드**: `docs/USAGE.md` (녹화·모니터·bag 추출·파라미터·문제해결)
- **문제해결**: `docs/TROUBLESHOOTING.md` (실기 운영 중 겪은 문제 사례별 정리)
- **캘리브레이션 가이드**: `docs/CALIBRATION.md` (촬영법·Kalibr 실행·결과 판정·calib.yaml·문제해결)
- **Cam-LiDAR 캘리브 가이드**: `docs/CAM_LIDAR_CALIBRATION.md` (T_front_lidar, 수동 2D-3D 대응+DS-PnP, 정지 1단계·모션보정 2단계)
- **매핑 가이드**: `docs/MAPPING.md` (오프라인 LIO 실행·산출물·시각화·판정)
- **BEV 자동라벨 파이프라인**: `docs/BEV_AUTOLABEL.md` (3어안→BEV occupancy 학습용 auto-label; 규격·단계·특이사항·**§A 실행 가이드(CLI)**. PoC·CLI 구현 완료 `calibration/bev_autolabel/`, 다중 bag 검증)
- 설계 스펙(BEV auto-label 품질개선): `docs/superpowers/specs/2026-07-27-bev-autolabel-quality-improvement-design.md`
- 구현 계획(BEV auto-label): `docs/superpowers/plans/2026-07-27-bev-autolabel-quality-improvement.md`
- 설계 스펙(매핑): `docs/superpowers/specs/2026-07-20-lio-mapping-integration-design.md`
- 설계 스펙(캘리브·검증): `docs/superpowers/specs/2026-07-18-camera-calibration-and-verification-design.md`
- 구현 계획(캘리브): `docs/superpowers/plans/2026-07-18-camera-calibration-and-verification.md`
- 설계 스펙(카메라 수집): `docs/superpowers/specs/2026-07-16-econ-camera-ros2-capture-design.md`
- 설계 스펙(수집 시스템 & bag 녹화 주의사항, Camera+LiDAR→BEV):
  `docs/superpowers/specs/2026-07-18-data-collection-bag-and-fusion-design.md`
- 구현 계획: `docs/superpowers/plans/2026-07-16-econ-camera-ros2-capture.md`
