# LIO 오프라인 매핑 통합 설계 (Point-LIO ROS2 → econ_camera_ws)

- 작성일: 2026-07-20
- 브랜치: `feat/lio-mapping-integration`
- 상태: 설계 확정
- 관련 문서: `2026-07-18-data-collection-bag-and-fusion-design.md`(Camera+LiDAR→BEV),
  `2026-07-20-lidar-integration-design.md`(LiDAR 수집 통합)

## 1. 배경 / 목표

카메라 4대 + Unitree L2 LiDAR를 단일 bag(mcap)으로 수집하는 파이프라인은 구현 완료됐다.
다음 단계는 이 bag을 **오프라인 처리하여 ego 궤적(pose)과 3D 맵을 복원**하는 것이다. 이는
최종 목표인 **BEV 학습 데이터셋**의 전제다 — BEV 라벨(점유·높이·주행가능영역)은 LiDAR 점군을
ego-pose로 여러 프레임 누적·투영하여 자동 생성하며, 그 **pose가 핵심 산출물**이다.

`point_lio_ros2`(HKU Point-LIO의 ROS2 포트, dfloreaa/point_lio_ros2)를 이 ws에 통합하여,
**"bag 경로 → {맵 PCD + 궤적 TUM + 미리보기}"** 를 재현 가능하게 한 번에 뽑는 도구를 만든다.

### 이번 세션에서 검증된 전제 사실
- `point_lio_ros2`는 **ROS2 Humble에서 그대로 빌드·동작**한다(ROS1 변환 불필요).
  - 패키지명 `point_lio`, 실행파일 `pointlio_mapping`, `ament_cmake`.
  - livox 의존성은 CMakeLists·소스에서 **전부 주석 처리**되어 있어 livox 없이 빌드됨.
  - 빌드 선결 조건: `ros-humble-pcl-ros`, `ros-humble-pcl-conversions`(apt). 최초 빌드 ~90s.
- L2 설정(`config/unilidar_l2.yaml`)이 수집 bag과 정합:
  `lid_topic:/unilidar/cloud`, `imu_topic:/unilidar/imu`, `scan_line:18`,
  per-point `time` 필드 존재(README의 *"Failed to find match for field 'time'"* 문제 없음),
  `imu_time_inte:0.004`(250Hz IMU와 일치).
- 정지 벤치 테스트 bag(`rosbag2_2026_07_20-19_49_12`, 141s)으로 관통 확인:
  - `/cloud_registered` ~8.6Hz, 발행 토픽 `/aft_mapped_to_init`(ego pose, frame
    `camera_init`→`aft_mapped`), `/path`, `/Laser_map`, `/cloud_registered(_body)`, `/cloud_effected`.
  - SIGINT 시 `PCD/scans.pcd` 저장(1,914,879 pts, 61MB). 클린 종료.
- **한계(정상)**: 이 테스트 bag은 정지 상태라 맵은 정적 장면 재구성이다. 실제 BEV용으로는
  이동하며 녹화한 bag이 필요하다(도구 자체 검증에는 이 bag으로 충분).

## 2. 통합 방식 (확정 결정)

- **벤더링 = 선택적 복사**: `third_party/point_lio_ros2/`(중첩 clone)를 **`src/point_lio/`** 로
  복사하고 중첩 `.git`을 제거한다. ROS2 패키지이므로 `unitree_lidar_ros2`처럼 `src/`에 두어
  `colcon build`에 기본 포함시킨다. **벤더 소스(C++/config/launch)는 수정하지 않는다.**
- **래퍼 = `mapping/` 디렉터리**(기존 `calibration/` 툴링 패턴과 동일): bag→산출물 오케스트레이션
  스크립트 + 궤적 로거 + 시각화 도구.
- **궤적 포맷 = TUM** (`timestamp tx ty tz qx qy qz qw`, space-구분, timestamp 초 단위).
  SLAM 평가 표준이자 후속 BEV 가공에서 소비하기 쉬움.
- **출력 폴더 레이아웃**: 실행마다 지정한 `<out_dir>`에 산출물을 모은다(point_lio가 pcd를
  패키지 폴더에 저장하는 문제를 래퍼가 정리).

## 3. 디렉터리 구조

```
econ_camera_ws/
├── src/
│   ├── econ_camera_ros/         (기존)
│   ├── unitree_lidar_ros2/      (기존, LiDAR 드라이버)
│   └── point_lio/               ← 신규 벤더: third_party/point_lio_ros2 복사(.git 제거)
├── mapping/                     ← 신규 툴링 디렉터리
│   ├── lio_map_bag.sh           오케스트레이터 (bag → 맵·궤적·미리보기)
│   ├── pose_logger.py           /aft_mapped_to_init → trajectory.tum (rclpy)
│   ├── pcd_preview.py           raw scatter top-down/side PNG (numpy)
│   ├── bev_grid.py              셀 집계 height/density PNG (BEV 라벨 형식, numpy)
│   └── test/
│       └── test_pose_logger.py  TUM 포맷 순수 로직 pytest
└── docs/
    └── MAPPING.md               운영 가이드
```

- 벤더링 후 `third_party/point_lio_ros2/`(원본 clone)는 제거한다(내용이 `src/point_lio`로
  이관됨 · untracked `git clean` 위험 제거). ROS1짜리 `third_party/point_lio_unilidar/`는
  사용자 참고용이므로 **손대지 않는다**.
- `build/`, `install/`, `log/`, `src/point_lio/{PCD,Log}/` 산출물은 커밋하지 않는다(gitignore 확인).

## 4. 벤더링 세부

- `src/point_lio`는 벤더 clone을 그대로 복사한 것이며, **CMake 경로 수정이 불필요**하다
  (point_lio는 IKFoM 등 의존 헤더를 자체 `include/`에 포함하는 self-contained 패키지).
- 빌드: `colcon build`(ws 전체) 또는 `colcon build --packages-select point_lio`.
  선결: `sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions`.
- 기본 설정(`config/unilidar_l2.yaml`, `launch/mapping_unilidar_l2.launch.py`)을 그대로 사용:
  `pcd_save_en:true`, `interval:-1`(단일 파일), `publish_odometry_without_downsample:true`,
  `use_system_timestamp` 계열은 수집 bag이 이미 시스템 시간 도메인이라 정합됨.

## 5. `mapping/` 래퍼 구성

### 5.1 `lio_map_bag.sh <bag_path> <out_dir> [--no-preview]`
이번 세션에서 성공시킨 절차(`run_lio.sh`)를 정식화한 오케스트레이터. 순서:
1. `source /opt/ros/humble/setup.bash` + `source install/setup.bash`.
2. `pose_logger.py`를 백그라운드로 기동 → `<out_dir>/trajectory.tum` 기록 시작.
3. `ros2 launch point_lio mapping_unilidar_l2.launch.py rviz:=false`를 백그라운드(`setsid`)로 기동.
4. `ros2 bag play <bag_path>` (블로킹, 재생 완료까지 대기).
5. 재생 종료 후 point_lio 프로세스 그룹에 **SIGINT**(→ `scans.pcd` 저장), pose_logger 종료.
6. `src/point_lio/PCD/scans.pcd` → `<out_dir>/map.pcd` 이동.
7. `<out_dir>/run_info.txt` 작성: bag 경로·duration, 맵 point 수, 궤적 pose 수, 주요 파라미터,
   `git rev-parse HEAD`.
8. `--no-preview`가 아니면 `bev_grid.py`·`pcd_preview.py`로 `<out_dir>/preview/*.png` 생성.

프로세스 수명 관리는 검증된 `setsid` + `kill -INT -PGID` 방식을 따른다.

### 5.2 `pose_logger.py`
- rclpy 노드. `/aft_mapped_to_init`(`nav_msgs/Odometry`) 구독 → 매 메시지를 TUM 한 줄로 append.
- 순수 함수로 분리하여 테스트 가능:
  ```python
  def odom_to_tum_line(stamp_sec: float,
                       px, py, pz, qx, qy, qz, qw) -> str:
      # "t tx ty tz qx qy qz qw"  (소수 표기, 공백 구분)
  ```
- timestamp = 메시지 `header.stamp`(초). 카메라와 동일 시간 도메인이라 후속 크로스모달 매칭에 사용.
- 인자: 출력 파일 경로. 종료(SIGINT/SIGTERM) 시 flush·close.

### 5.3 `pcd_preview.py` / `bev_grid.py` (이미 작성·검증됨)
- `pcd_preview.py`: 원시 점 top-down/side scatter PNG(numpy만, 헤드리스).
- `bev_grid.py`: 셀 단위 집계(최대 높이 / 로그 밀도) PNG — **실제 BEV 라벨 래스터화와 동일 연산**.
  둘 다 `<pcd> [out_dir]` 인자를 받도록 정리하여 래퍼가 호출.

### 5.4 출력 폴더 레이아웃
```
<out_dir>/
├── map.pcd            # scans.pcd 이관 (world 프레임 누적 점군)
├── trajectory.tum     # ego pose 시퀀스 (t tx ty tz qx qy qz qw)
├── run_info.txt       # bag·duration·#points·#poses·params·git rev
└── preview/
    ├── bev_topdown.png
    ├── bev_heightmap.png
    └── bev_density.png
```

## 6. 실행 & 검증

### 빌드
```bash
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions
cd ~/Desktop/econ_camera_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select point_lio
source install/setup.bash
```

### 매핑 실행
```bash
./mapping/lio_map_bag.sh rosbag2_2026_07_20-19_49_12 out/run01
```

### 검증
- **자동(순수 로직)**: `mapping/test/test_pose_logger.py` — `odom_to_tum_line`이 필드 순서
  (position xyz → quaternion xyzw)·공백 구분·timestamp 초 단위를 정확히 내는지 pytest.
- **회귀**: 기존 카메라 pytest 18개(`cd src/econ_camera_ros && python3 -m pytest test/`) 무회귀.
- **수동(관통)**: 위 정지 bag으로 `lio_map_bag.sh` 실행 → `<out_dir>`에
  `map.pcd`(point > 0), `trajectory.tum`(라인 > 0), `run_info.txt`, `preview/*.png` 생성 확인.
  point_lio 자체가 prebuilt C++ SLAM이라 알고리즘 정확도의 자동 테스트는 없음(맵·궤적 생성 여부로 판정).

| # | 명령어 | 기대 결과 |
|---|--------|-----------|
| 1 | `colcon build --packages-select point_lio` | `Finished`, 에러 0 (경고 허용) |
| 2 | `./mapping/lio_map_bag.sh <bag> out/run01` | 정상 종료, `out/run01/` 생성 |
| 3 | `head -1 out/run01/trajectory.tum` | `t tx ty tz qx qy qz qw` 형태 1줄, 필드 8개 |
| 4 | `wc -l out/run01/trajectory.tum` | 라인 수 > 0 (pose 기록됨) |
| 5 | `python3 mapping/pcd_preview.py out/run01/map.pcd` | PNG 생성(맵 point > 0) |
| 6 | `python3 -m pytest mapping/test/` | pose_logger TUM 포맷 테스트 통과 |
| 7 | `cd src/econ_camera_ros && python3 -m pytest test/` | 18 passed (기존 카메라 테스트 회귀 없음) |

## 7. 문서 업데이트
- **`docs/MAPPING.md`(신규)**: 개요(오프라인 LIO의 역할·BEV와의 관계), 빌드 선결 조건,
  `lio_map_bag.sh` 사용법, 산출물 설명(map.pcd/trajectory.tum/run_info), 시각화 방법
  (헤드리스 `pcd_preview`/`bev_grid`, 디스플레이 `pcl_viewer`/RViz), 정지 vs 이동 촬영 주의,
  문제해결.
- `CLAUDE.md`: 현재 상태에 오프라인 매핑 도구(`src/point_lio`·`mapping/`·`docs/MAPPING.md`) 반영.
- `docs/USAGE.md`: 매핑 절차 요약 + `docs/MAPPING.md` 링크.

## 8. Git
- 본 작업은 `feat/lio-mapping-integration` 브랜치에서 진행.
- 커밋까지만 수행. 병합·푸시는 사용자가 직접(프로젝트 규칙).

## 9. 범위 밖 (YAGNI)
- BEV 데이터셋 생성 파이프라인 자체(누적·크로스모달 투영·라벨 포맷) — 후속 sub-project.
- cam↔LiDAR extrinsic 캘리브레이션 — 별도 진행 중(`feat/zed-lidar-extrinsic` 계열).
- 동적 객체 제거, 루프 클로저/포즈 그래프 최적화, 다중 bag 병합.
- point_lio 파라미터 튜닝(기본 L2 설정으로 충분; 실측 이동 bag 확보 후 필요 시 후속).
- 하드웨어 정밀 시간 동기(PTP 등).
```
