# 전체 파이프라인 가이드 (데이터 수집 → semi-auto BEV 라벨)

이 문서는 **처음 보는 사람이 데이터 수집부터 BEV occupancy 데이터셋 자동 라벨(semi-auto)까지
순서대로 따라 할 수 있게** 전 과정을 하나로 꿴다. 각 단계마다 **① 무엇을 하는가 ② 무엇을 실행
하는가 ③ 주요 옵션 ④ 결과물**을 적고, 깊은 내용은 해당 상세 문서로 링크한다.

- 최종 산출물: 어안 3대(front/left/right) 이미지 → **ego 중심 BEV occupancy/visibility 라벨(기본 120×120,
  occupancy 0=obstacle/1=drivable, visibility 0=unseen/1=visible)** 학습 데이터셋.
- "semi-auto"인 이유: LiDAR+맵과 IPM으로 **초안 라벨을 자동 생성**하되, 100%가 아니므로 **마지막은 사람이 검수·확정**한다.

---

## 0. 한눈에 보기

```
[리그당 1회] ── 캘리브레이션 ───────────────────────────────────────────────
  0. 환경/빌드
  1a. cam-cam (Kalibr)        AprilGrid 촬영 → calib.yaml (intrinsic + T_cam_front)
  1b. cam-lidar (수동 PnP)     정지 촬영     → calib.yaml 확장 (T_front_lidar)

[bag당 반복] ── 수집 & 라벨 ────────────────────────────────────────────────
  2. 데이터 수집        record_all → rosbag2_*/           (+QC: check_recording/check_lidar_bag)
  3. 이미지 추출        bag_extract → frame_*/cam{0..3}.jpg + sets.csv
  4. LIO 매핑          lio_map_bag → map.pcd + trajectory.tum → pcd_denoise → map_clean.pcd
  5. auto-label(라벨+IPM) generate_slab.py → slab/sample_*/{occupancy,visibility,ipm_rgb,overlay,review,cam_*,meta}
  6. 수동 보정 준비       gather_slab.py → annotations/<name>/{label,label_guided,review}/ → CVAT 보정
  7. 학습 라벨 확정       manual_labels.py → manual_labels/<name>/{rgb_images,occupancy_*,visibility_*,review_png,labels.csv}
```

| 단계 | 실행(대표) | 결과물 | 상세 문서 |
|---|---|---|---|
| 0. 환경/빌드 | `colcon build` | 실행 가능한 ws | [USAGE §2](USAGE.md), [LIDAR §2](LIDAR.md) |
| 1a. cam-cam | `run_kalibr.sh` → `calib_convert` | `calib.yaml`(intrinsic+`T_cam_front`) | [CALIBRATION.md](CALIBRATION.md) |
| 1b. cam-lidar | `pick_correspondences` → `solve_extrinsic` | `calib.yaml`에 `T_front_lidar` 추가 | [CAM_LIDAR_CALIBRATION.md](CAM_LIDAR_CALIBRATION.md) |
| 2. 수집 | `record_all.launch.py` | `rosbag2_*/`(mcap) | [USAGE §3·§9](USAGE.md), [LIDAR §3](LIDAR.md) |
| 3. 추출 | `bag_extract` | `frame_*/cam{0..3}.jpg` + `sets.csv` | [USAGE §6](USAGE.md) |
| 4. 매핑 | `lio_map_bag.sh` → `pcd_denoise.py` | `map.pcd`·`map_clean.pcd` + `trajectory.tum` | [MAPPING.md](MAPPING.md) |
| 5. auto-label(라벨+IPM) | `generate_slab.py` | `sample_*/{occupancy.png,visibility.png,ipm_rgb.png,overlay.png,review.png,cam_*.jpg,meta.json}` | [BEV_AUTOLABEL §B](BEV_AUTOLABEL.md) |
| 6. 수동 보정 준비 | `gather_slab.py` → (CVAT) | `annotations/<name>/{label,review}/` 또는 `label_guided/` 업로드·보정 → `manual_annotated/<name>_120x120_annotation/` export | [BEV_AUTOLABEL §B](BEV_AUTOLABEL.md) |
| 7. 학습 라벨 확정 | `manual_labels.py` | `manual_labels/<name>/{rgb_images,occupancy_npy,visibility_npy,occupancy_png,visibility_png,review_png,labels.csv}` | [BEV_AUTOLABEL §B](BEV_AUTOLABEL.md) |

> 5~7단계는 **슬래브 라벨(`generate_slab.py` + `gather_slab.py` + `manual_labels.py`, 기본 120×120)** 기준이다.
> 구판 경로(`generate.py` + `gather_annotations.py`, 단일 `label.png` 0/1/2, 80×80)는
> [BEV_AUTOLABEL 부록 A](BEV_AUTOLABEL.md) 로 보관돼 있다 — `data/bev/dataset/` 의 기존
> 산출물을 해석할 때만 참고하고, 새 데이터 생성에는 쓰지 않는다.

> **주기 구분**: 1a·1b(캘리브)는 **리그(카메라·라이다 장착)를 바꾸지 않는 한 1회**만 하고 이후
> 모든 bag이 그 `calib.yaml`을 공유한다. 2~7은 **수집한 bag마다** 반복한다.
>
> **폴더 규약**: 모든 산출물은 `data/`(gitignore) 아래로 모은다. bag별 3쌍을 같은 `<name>`으로 맞춘다 —
> `data/sj_bags/<날짜>/bags/<bag>` ↔ `.../maps_selfmask/<name>_mapping`(매핑 산출) ↔ `data/extracted/<name>`(추출 이미지).
> `<name>`: `raws{N}`=with-sun, `rawos{N}`=without-sun. BEV 산출은
> `data/bev/slab/<name>`(auto-label 초안), `data/bev/annotations/<name>`(CVAT 업로드용),
> `data/bev/manual_annotated/<name>_120x120_annotation`(수동 보정 export),
> `data/bev/manual_labels/<name>`(학습용 확정 라벨)로 둔다.
> 260722의 `.../maps/`(구버전, self mask·drain 수정 이전)는 대조용 보관본이다 — **하류는 `maps_selfmask/`를 쓴다**.

---

## 0단계. 환경 준비 & 빌드 (최초 1회)

**무엇**: ROS2 Humble 호스트 네이티브 + 이 ws 빌드 + 선행 프로젝트 순수 로직 재사용.

> **수집이 끝난 데이터를 서버에서 가공만 한다면** 이 절은 건너뛰고 [DOCKER.md](DOCKER.md)를 본다.
> 3·5·6단계와 calib 검증은 **colcon 빌드 없이** 단일 Docker 이미지로 돌아간다
> (후처리 스크립트는 `rosbag2_py`/`rclpy`를 지연 import 하는 평범한 파이썬이라 ROS 런타임만 있으면 된다).
> 아래 호스트 네이티브 빌드가 필요한 건 **촬영(2단계)과 LIO 매핑(4단계)** 뿐이다.

```bash
# 선행 프로젝트의 econ_cam(controls/stats) 재사용 — 복붙 아님, import
pip install -e ../Multi-Cam_module_test

# ws 빌드 (카메라 + LiDAR + Point-LIO 패키지)
cd ~/Desktop/econ_camera_ws
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions   # 매핑용, 최초 1회
rosdep install --from-paths src --ignore-src -r -y               # LiDAR 의존, 최초 1회
colcon build
source install/setup.bash        # ← 새 터미널 열 때마다 필요
```

**주의(환경 핀)**: 이 Jetson은 **시스템 scipy가 numpy<1.25 고정**이다. `pip --user`로 numpy≥2
(또는 opencv-python≥4.10)를 끌어오면 scipy가 깨져 라벨 파이프라인 전체가 멈춘다
(`ValueError: numpy.dtype size changed`). user-site에 numpy≥2/opencv-python≥4.10을 넣지 말 것.
PCD 읽기는 open3d 0.18(user-site, numpy1.x ABI 호환) 사용.

**결과**: `ros2 run/launch econ_camera_ros ...`가 동작. LiDAR 네트워크(`192.168.1.2/24`, `ping 192.168.1.62`)도 확인([LIDAR §1](LIDAR.md)).

---

## 1단계. 캘리브레이션 (리그당 1회)

라벨은 이미지 픽셀 ↔ 3D(라이다/맵)를 오가야 하므로 **캘리브가 모든 라벨의 기하 기반**이다.
카메라 간(cam-cam) → 카메라-라이다(cam-lidar) **순서**로 하며, 각각 **전용 촬영 bag**이 필요하다.

### 1a. 카메라 intrinsic + 카메라 간 extrinsic (Kalibr)

**무엇**: 어안 4대의 DS(Double Sphere) intrinsic과 카메라 간 상대자세(`T_cam_front`)를 구한다.

```bash
# (최초 1회) Kalibr arm64 Docker 이미지 빌드
sudo bash calibration/build_kalibr_arm64.sh

# 촬영 — 리그 고정, AprilGrid 판을 크게·다양한 포즈로. 인접 카메라 겹침 프레임 필수
ros2 launch econ_camera_ros record.launch.py            # 충분히 찍고 Ctrl-C

# 추출 → Kalibr 데이터셋(4Hz 다운샘플) → 방향 매핑
ros2 run econ_camera_ros bag_extract <bag_dir> -o extracted
ros2 run econ_camera_ros kalibr_bridge extracted -o dataset --rate 4.0
cp calibration/aprilgrid.yaml .
cp calibration/orientation.example.json orientation.json   # monitor로 카메라 방향 확인 후 편집

# Kalibr 실행(ds-none·eucm-none 둘 다) → 결과 비교 → calib.yaml
sudo bash calibration/run_kalibr.sh "$(pwd)"
ros2 run econ_camera_ros calib_convert calib-camchain-ds.yaml \
  --model ds -o calib.yaml --orientation orientation.json \
  --rms front=.. right=.. rear=.. left=..
```

**주요 옵션/판정**:
- **촬영이 품질을 좌우**: 보드가 화면 가로 절반 이상, 태그 한 변 ≥50–80px, 극단 가장자리까지, 인접 쌍 동시 관측(겹침 없으면 extrinsic 못 품).
- 채택 판정: `calib-results-cam-*.txt`의 카메라별 재투영 **std(px)** — ≲1 양호, ≳5 재촬영. ds vs eucm 중 작은 쪽.
- `--orientation`을 주면 결과 키가 `cam0~3` 대신 방향명(front/right/rear/left)이 된다.

**시각 검증**(검출 불필요·호스트 파이썬):
```bash
python3 calibration/verify/verify_undistort.py --images data/calib_260723/extracted --frames "800,1600"
python3 calibration/verify/verify_panorama.py  --images data/calib_260723/extracted --frames "800,1600"
python3 calibration/verify/verify_extrinsics.py --images data/calib_260723/extracted --frames "800,1600"
```

**결과**: `calib.yaml`(DS intrinsic 4대 + 카메라 간 `T_cam_front` + `verification.reproj_rms_px`). 상세: [CALIBRATION.md](CALIBRATION.md).

### 1b. 카메라–라이다 extrinsic (`T_front_lidar`)

**무엇**: 라이다 점 → front 카메라 프레임 변환 `T_front_lidar`를 수동 2D–3D 대응 + DS-PnP로 구해
1a의 카메라 체인에 라이다를 한 단으로 붙인다. **1a가 선행되어야 한다**(`calib.yaml` 필요).

```bash
# 정지 촬영 — 리그 완전 고정, 모서리 뚜렷한 물체를 라이다+여러 카메라가 동시에 보게
ros2 launch econ_camera_ros record_all.launch.py        # 정지 10–20초, Ctrl-C

# 클라우드 누적(정지라 시간창만) + 대표 이미지 추출
python3 calibration/cam_lidar/accumulate_cloud.py <bag_dir> -o cloud.npy --window 3
ros2 run econ_camera_ros bag_extract <bag_dir> -o extracted

# 대응점 클릭(디스플레이 필요) — 카메라 1대씩, 이미지→클라우드 같은 순서로. 4대 반복 --append
python3 calibration/cam_lidar/pick_correspondences.py \
  --cloud cloud.npy --image extracted/frame_XXXXXX/cam0.jpg --cam front --out corrs.json --append

# 풀기 + calib.yaml 기록
python3 calibration/cam_lidar/solve_extrinsic.py corrs.json \
  --calib data/calib_260723/calib.yaml --orientation data/calib_260723/orientation.json \
  --init-rpy <대략 roll pitch yaw(도)> --init-xyz <대략 x y z(m)> --holdout 3 --stage 1

# 시각 검증 — 4대 이미지에 라이다 점 오버레이
python3 calibration/cam_lidar/overlay_verify.py --cloud cloud.npy --frame extracted/frame_XXXXXX \
  --calib data/calib_260723/calib.yaml --orientation data/calib_260723/orientation.json --out overlay
```

**주요 옵션/판정**:
- 대응점은 카메라당 학습 ~8쌍 + 홀드아웃 2~3쌍, **가까운/먼·좌우·상하로 분산**(한 평면에 몰리면 회전 못 품).
- 판정: `solve_extrinsic`이 출력하는 **train/holdout RMS 둘 다 ≲2px** + 4대 오버레이에서 엣지 정합.
- 회전 드리프트만 보이면 **2단계**(현장 bag 모션 보정, [CAM_LIDAR §6](CAM_LIDAR_CALIBRATION.md)).

**결과**: `calib.yaml`에 `extrinsics.T_front_lidar` + `verification.cam_lidar` 추가. 상세: [CAM_LIDAR_CALIBRATION.md](CAM_LIDAR_CALIBRATION.md).

> 이후 `calib.yaml` + `orientation.json` 한 쌍이 **모든 bag의 라벨 생성에 공유**된다(예: `data/calib_260723/`).

---

## 2단계. 데이터 수집 (bag)

**무엇**: 카메라 4대 + LiDAR(점군·IMU·TF)를 **단일 bag(mcap)** 으로 동시 녹화한다. 공간을 이동하며 촬영해야 매핑 궤적이 생긴다.

```bash
# 사전조건: 호스트 NIC 192.168.1.2/24, ping 192.168.1.62 OK
ros2 launch econ_camera_ros record_all.launch.py        # 촬영 후 Ctrl-C
```

- 녹화 토픽 8개: `/camera{0..3}/image_raw/compressed`, `/unilidar/cloud`, `/unilidar/imu`, `/tf`, `/tf_static`.
- 실행한 **현재 디렉터리**에 `rosbag2_<timestamp>/` 생성. 옮길 땐 **디렉터리째** 이동.
- 실시간 확인(선택): 다른 터미널 `ros2 run econ_camera_ros monitor` → `http://<Orin-IP>:10010`.

**QC(성공 판정) — 모니터 화면이 아니라 도구로 판정**:
```bash
ros2 bag info rosbag2_<ts>                               # 8토픽·메시지 수(카메라4 동일해야)
python3 tools/check_recording.py rosbag2_<ts>            # 카메라 프레임 간격·FPS·끊김·4대 정렬 (pip install mcap)
python3 mapping/check_lidar_bag.py rosbag2_<ts>          # /unilidar/cloud 프레임수·rate·빈프레임 + BEV PNG
```
- `check_recording` ❌ 조건: 카메라 간 프레임 수 차 3장 초과 / FPS < 기대×0.95 / 끊김 검출.
- 여기서 이상하면 뒤 단계가 다 오염되므로 **여기서 거른다**.

**결과**: 검증 통과한 `rosbag2_<ts>/`. 상세: [USAGE §3·§5·§9](USAGE.md), [LIDAR §3·§6](LIDAR.md).

---

## 3단계. bag → 학습용 이미지 추출

**무엇**: bag의 카메라 4대를 **동기 세트**로 묶어 프레임별 JPEG로 떨군다. calib·BEV가 공통으로 쓰는 전제.

```bash
ros2 run econ_camera_ros bag_extract rosbag2_<ts> -o data/extracted/<name>
# ROS 없이도: python3 src/econ_camera_ros/econ_camera_ros/bag_extract.py <bag> -o <out>
```

**주요 옵션**:

| 옵션 | 기본 | 설명 |
|---|---|---|
| `-o, --out` | `extracted` | 출력 폴더 |
| `--tolerance` | `0.001` | 동기 허용오차(초). 이내로 묶인 세트만 저장(HW 동기가 sub-ms라 1ms면 충분) |
| `--limit` | (없음) | 앞 N 세트만(빠른 확인) |

**결과**: `data/extracted/<name>/frame_NNNNNN/cam{0..3}.jpg` + `sets.csv`(idx, stamp0..3, spread_ms). 완성 세트(4대)만 저장. 상세: [USAGE §6](USAGE.md).

---

## 4단계. LIO 매핑 (bag → 궤적·맵)

**무엇**: bag을 Point-LIO로 후처리해 **ego 궤적(pose)과 3D 맵**을 복원한다. BEV 라벨의 전제(ego-motion + 월드 클라우드).

```bash
# 매핑 전 사전점검(권장)
python3 mapping/check_lidar_bag.py rosbag2_<ts>

# 매핑
colcon build --packages-select point_lio && source install/setup.bash   # 최초 1회
MAPDIR=data/sj_bags/<날짜>/maps_selfmask/<name>_mapping
./mapping/lio_map_bag.sh rosbag2_<ts> $MAPDIR

# 고립 노이즈 제거(원본 보존, 옆에 map_clean.pcd 생성) — 하류는 이걸 쓴다
python3 mapping/pcd_denoise.py $MAPDIR/map.pcd
```

**주요 옵션**:
- `--max-secs N`: bag 재생을 N초에서 중단(LIO 발산/오염 구간 제외).
- `--no-preview`: 미리보기 PNG 생략.
- `pcd_denoise.py --dry-run`: 쓰지 않고 제거 통계만. `--dist 0.3`(기본)은 실측 p99.9라 건드릴 일이 드물다.

**결과**: `map.pcd`(월드 밀집 클라우드) + `map_clean.pcd`(고립점 제거, 실측 0.05~0.10%) +
`trajectory.tum`(pose `world_T_body`) + `run_info.txt` + `preview/`.

**self mask(기본 ON)**: 카트를 끄는 수집자가 맵에 통째로 적립되므로 `map.pcd` **저장 단계에서만** 잘라낸다
(정합에는 남긴다 — 빼면 z 드리프트가 악화). 남았는지는 `python3 mapping/check_self_points.py $MAPDIR --png`
로 확인. 상세·근거: [MAPPING.md §6.5](MAPPING.md).

**건강성 판정(중요)**: 두 가지를 본다.
1. **잘림**: `run_info.txt`의 `traj_span_s` ≈ bag 길이(1초 이내). 벌어지면 bag 뒷부분이 통째로 누락된 것.
2. **붕괴**: `trajectory.tum` **총 길이를 실제 공간과 대조**한다(2 Hz로 다운샘플 후 재라 — raw는 지터로 3배 부풀려짐).
   z가 안정해도 궤적이 붕괴하면 매핑 실패(온실 긴 복도가 정답). 붕괴한 맵으로 라벨을 만들면 오염된다.

260722 7종 실측치는 [MAPPING.md §6.7](MAPPING.md) 표 참조.

---

## 5단계. LiDAR auto-label 데이터셋 생성

**무엇**: 맵+궤적+calib으로 **BEV 라벨 2채널**(`occupancy` + `visibility`) 초안을 키프레임마다
자동 생성하고, **동시에 3어안을 지면 평면에 IPM 투영해 BEV RGB 캔버스**(위에서 본 주행면)를
만들어 라벨을 얹은 CVAT 보정 base·검수뷰까지 낸다. 핵심 아이디어 = "로봇이 통과해야 하는
높이 구간(슬래브)에 점이 서 있으면 obstacle, 보이는 그 외는 drivable, 못 본 곳은 visibility=0"
(LiDAR는 바닥을 못 보고 장애물만 잘 봄 → 바닥 모습은 IPM으로 보완).

```bash
cd calibration/bev_autolabel

python3 generate_slab.py \
  --map-dir ../../data/sj_bags/<날짜>/maps_selfmask/<name>_mapping \
  --extract-dir ../../data/extracted/<name> \
  --calib ../../data/calib_260723/calib.yaml --orient ../../data/calib_260723/orientation.json \
  --self-mask-dir ../../data/calib_260723/self_mask \
  --out ../../data/bev/slab/<name>

# 검수 시트(생성 CLI가 자동 실행하지 않는다) — 궤적 전체를 한 장으로 훑어본다
python3 slab_sheet.py ../../data/bev/slab/<name>
```

**주요 옵션**(전부 `generate_slab.py`, 괄호는 기본값):
- `--kf-step 0.4`: 키프레임 이동거리 간격(m). **구간별 스케줄**도 받는다 —
  `--kf-step 0:0.5,0.2:1.5,0.7:0.5` = 누적 이동거리 0~20%는 0.5m, 20~70%는 1.5m,
  70~100%는 0.5m. 단조로운 중반부를 성기게, 변화가 많은 앞뒤를 촘촘히 뽑을 때 쓴다.
  `sample_NNNNNN` 번호는 전 구간에 걸쳐 연속으로 매겨진다.
- `--xf 4.0 --xr 2.0 --yh 3.0`(=**120×120**): BEV 전/후/좌우 범위(m). `RES`(0.05 m/cell) 고정.
  **구판 `generate.py`의 80×80과 다른 그리드다** — 한 학습 데이터셋에 섞지 말 것.
- `--thick 0.8`: 슬래브 두께(m, 로봇이 통과할 높이 구간). `--pct 1.0`: `z_ref` 퍼센타일
  (하위 1%; **5로 올리지 말 것** — 슬래브 바닥이 0.25m 들뜬다). `--min-pts 3`: obstacle 판정
  셀당 최소 점수.
- `--cam-height 0.87`: IPM 지면 평면용 카메라 렌즈 높이(m). 마스트 LiDAR가 바닥을 못 봐
  못 주므로 **자로 실측**. IPM 정확도의 핵심. `--ground-offset 0.87`: 관측가능성 판정 지면.
- `--self-mask-dir` + `--self-mask-classes table`(기본): 카트 상판·받침판 이미지 마스크.
  **`handle`·`human`을 여기 넣지 말 것** — 사람 위치가 프레임마다 달라 정적 마스크로는 못 맞힌다.
  그쪽은 body 프레임 self 박스(`--self-box-near 0.4 --self-box-far 2.1 --self-box-yh 0.7`)가 처리한다.
- `--alpha 0.30`: `overlay.png`의 obstacle 오버레이 불투명도(IPM 바닥이 보여야 보정할 수 있어 낮게).
  `--blend nearest`(기본): IPM 다중카메라 합성=셀별 최근접 1대. `--no-ipm`: 라벨만 빠르게.
  `--limit N`: 스모크(앞 N개만 — bag 앞부분 넓은 입구만 뽑히므로 **판정 근거로 쓰지 말 것**).
- `T_front_lidar` 키가 없으면 CLI가 즉시 종료(1b 선행 필요).

**결과**: `data/bev/slab/<name>/sample_NNNNNN/` 마다
- `occupancy.png` — `0=obstacle` / `1=drivable` (**사람이 보정하는 유일한 채널**),
- `visibility.png` — `0=unseen` / `1=visible` (학습 loss 마스크. 보정본 occupancy로 raycast를
  다시 돌리면 재생성되므로 사람이 손댈 필요 없다),
- `ipm_rgb.png` — 3어안 IPM 투영 **BEV RGB 캔버스**(기본 120×120; 바닥 모습은 이것으로만 보인다),
- `overlay.png` — `ipm_rgb` + **obstacle만** 반투명 오버레이(**네이티브 해상도·장식 없음**)
  = **CVAT 업로드용 보정 base**(resize 왕복 없음),
- `review.png` — 상단 원본 3어안(좌·전·우) + 하단 `[4색(occ+vis) | ipm+occupancy]` 두 패널,
- `cam_{front,left,right}.jpg` — 원본 3이미지, `meta.json` — pose·stamp·BEV 규격·파라미터.
- 최상위 `dataset.csv`(sample↔frame↔stamp). `slab_sheet.py`는 `_sheet_review.png`·
  `_sheet_stack.png`·`_stats.txt`를 추가로 낸다.

> ego 주변 검은 직사각형은 **정상**이다 — 카메라가 수평 바깥을 봐서 생기는 근거리 사각
> (실측 가시 시작 0.65~0.80m)과 후방 self 박스가 합쳐진 것이다.

상세·규격·판정 기준·금지 사항·알려진 한계(visibility 수율, IPM↔라벨 정합 오차):
[BEV_AUTOLABEL §B](BEV_AUTOLABEL.md).

> **구판 경로**(`verify_labels.py` → `generate.py`, 단일 `label.png` 0/1/2, 80×80)는
> [BEV_AUTOLABEL 부록 A](BEV_AUTOLABEL.md)로 보관돼 있다. `data/bev/dataset/`의 기존
> 산출물을 해석할 때만 참고하고, 새 데이터 생성에는 쓰지 않는다.

---

## 6단계. 수동 보정 준비와 CVAT export

**무엇**: auto-label은 초안이다. 5단계가 이미 **IPM 배경(`ipm_rgb.png`) 위에 obstacle을 얹은
`overlay.png`** 를 네이티브 해상도로 만들어 두므로, **별도의 카메라 마스킹·IPM 투영 단계 없이**
사람이 BEV 위에서 occupancy 경계만 보정한다.

> **왜 마스킹이 사라졌나**: 예전엔 사람이 카메라 3장에 drivable 마스크를 그려 IPM 투영·융합했다
> (`dataset_flatten`+`ipm_review`). 이제 생성 CLI가 IPM 배경 위에 라벨을 미리 얹어 주므로,
> 사람은 처음부터 그리지 않고 **보정만** 하면 된다.

**작업 방법**:
- `review.png`(상단 3어안 + 하단 4색 라벨·IPM 오버레이)를 보고 각 sample 판단. 궤적 전체를
  한 장으로 훑는 건 `slab_sheet.py`의 `_sheet_review.png`.
- **`gather_slab.py`로 `overlay.png`를 한 폴더로 모아 CVAT 등에 업로드**해 그 위에서 경계를 보정:
  ```bash
  python3 calibration/bev_autolabel/gather_slab.py \
    --dataset data/bev/slab/<name> --out data/bev/annotations --guided-labels
  # → data/bev/annotations/<name>/label/  (네이티브 120×120) — 이 폴더 그대로 CVAT 업로드
  # → data/bev/annotations/<name>/review/ — 참고용 검수뷰(--review-scale, 기본 1=원본)
  # → data/bev/annotations/<name>/label_guided/ — obstacle 강조+0.5m grid 참고 base
  ```
  `label/`은 `overlay.png`를 **바이트 그대로 복사**한 것이다 — 재인코딩하면 annotation base의
  화소가 바뀐다. `label_guided/`는 같은 해상도에 obstacle을 더 진하게 보이고 0.5m grid를
  얹은 참고/대체 base다.
- CVAT export 는 아래 이름으로 둔다. 이 export 는 아직 학습 최종 라벨이 아니라
  `manual_labels.py` 입력이다.
  ```text
  data/bev/manual_annotated/<name>_120x120_annotation/
    labelmap.txt
    SegmentationClass/sample_NNNNNN.png
  ```
- **보정 대상은 `occupancy` 하나뿐이다.** `visibility`는 보정된 occupancy로
  `bev_label.raycast_visible`을 다시 돌리면 재생성되므로 사람이 손대지 않는다.
- **주된 보정**: obstacle 경계. `overlay.png`는 빨강을 obstacle에만 `--alpha 0.30`으로 얹고
  바닥은 IPM 원본을 남기므로, IPM 질감을 근거로 경계를 옮긴다. 셀 1개 안팎의 어긋남은
  IPM↔라벨 정합 오차이지 라벨 오류가 아니다([BEV_AUTOLABEL §B 알려진 한계](BEV_AUTOLABEL.md)).

> **구판의 `gather_annotations.py`를 쓰지 말 것** — 그 스크립트는 sample마다 `label.png`
> (0/1/2 인덱스)를 요구하는데 슬래브 산출물엔 없어(`occupancy.png`+`visibility.png`로 분리)
> 전 sample이 `skip (missing ipm_rgb/label)`로 건너뛰어지고 `done: 0 images`로 끝난다.
> **`occupancy.png`를 `render.blend_label`에 넣지도 말 것** — 0/1 값을 전부 칠해 캔버스 전
> 셀을 덮으므로 보정 근거인 IPM 바닥이 사라진다.

- 남은 한계: IPM 평면 가정(수직물체 번짐·원거리 부정확), visibility 저수율(694샘플 중앙값
  5.0%), 전방 동적 물체 미처리, 어안→모델 입력 언디스토션 필요, 데이터 규모(일반화는 여러
  bag/환경 확충 전제). 상세: [BEV_AUTOLABEL §B·§7](BEV_AUTOLABEL.md).

**결과**: 수동 보정된 occupancy export(`manual_annotated/<name>_120x120_annotation`).

---

## 7단계. 학습용 BEV 라벨 생성 (`manual_labels.py`)

**무엇**: 수동 보정 export 에서 학습에 바로 쓸 입력 이미지와 `.npy` 라벨을 만든다. occupancy 는
CVAT에서 보정한 `occupancy` 색을 읽어 `0=obstacle, 1=drivable` 이진 배열로 변환하고,
visibility 는 그 occupancy 를 기준으로 `raycast_visible` 을 다시 돌려 `0=unseen, 1=visible` 로
재생성한다. map 기반 occupancy 를 다시 신뢰하지 않는다.

**실행**:

```bash
python3 calibration/bev_autolabel/manual_labels.py \
  --manual-dir data/bev/manual_annotated/<name>_120x120_annotation

# 원본 sample 이 data/bev/slab/<name> 에 있을 때:
python3 calibration/bev_autolabel/manual_labels.py \
  --manual-dir data/bev/manual_annotated/<name>_120x120_annotation \
  --dataset-root data/bev/slab
```

폴더명에서 `<name>` 을 자동 추론하고 원본 sample/meta/IPM 은 `data/bev/dataset/<name>` 에서
찾는다. 원본 sample 을 `data/bev/slab/<name>` 등에 만들었다면 `--dataset-root` 에 실제 sample
폴더들의 상위 경로를 넘긴다. 출력 위치나 이름을 바꾸려면 `--out`, `--name` 을 명시한다.

**결과**:

```text
data/bev/manual_labels/<name>/
  rgb_images/sample_NNNNNN/
    cam_front.jpg
    cam_left.jpg
    cam_right.jpg
  occupancy_npy/*.npy      # uint8 (120,120), 0=obstacle, 1=drivable
  visibility_npy/*.npy     # uint8 (120,120), 0=unseen, 1=visible
  occupancy_png/*.png      # class id 보존 preview
  visibility_png/*.png     # class id 보존 preview
  review_png/*.png         # 상단 3어안 + 하단 4색 occ/vis + IPM overlay 검수용
  labels.csv
  README.md
```

`labels.csv` 는 `sample`, `rgb_dir`, `occupancy_npy`, `visibility_npy`, `occupancy_png`,
`visibility_png`, `review_png`, 통계 컬럼을 가진다. `rgb_images/` 의 3카메라 JPG는 원본
sample 폴더에서 재인코딩 없이 복사된다.

`visibility` 는 **수동 occupancy에서 재계산한 순수 2D raycast 결과**다. self-mask와 rear-mask는
visibility 에 곱하지 않는다. 대신 `review_png` 하단 BEV 패널에 확인용으로만 overlay 한다:
`data/calib_260723/self_mask/bev_self_mask.png` 는 노란색, `bev_rear_self_box_03.png` 는 magenta.
두 고정 BEV 마스크는 `meta.json` 의 `self_mask` 경로에서 읽는다.

**최종 산출물**: 학습에 바로 쓰는 확정 BEV occupancy/visibility 데이터셋과 원본 3카메라 입력.

---

## 부록

### A. 순수 로직 테스트 (하드웨어 불필요)
```bash
cd src/econ_camera_ros && python3 -m pytest test/ -q          # 수집·추출 로직 25
cd calibration/cam_lidar && python3 -m pytest -q              # cam-lidar 18
cd calibration/bev_autolabel && python3 -m pytest -q          # BEV auto-label(§A+§B) 84
cd calibration/verify && python3 -m pytest -q                 # calib 시각검증 7
```

### B. 자주 겪는 문제 (요약)
| 증상 | 조치 |
|---|---|
| `ros2 run`이 패키지 못 찾음 | `source install/setup.bash`(새 터미널마다) |
| `No cameras available`/Argus 오류 | 정상(UYVY 직출력). `v4l2src` 경로만 사용 |
| 카메라 프레임 수 불일치 | `check_recording.py`로 판정, `capture` 로그의 `frames={..}`·`동기 spread` 확인 |
| 매핑 궤적 붕괴 | 초기 정지 구간 확보, `--max-secs`로 오염 구간 제외, 궤적 길이 실제와 대조 |
| `T_front_lidar` 없음 종료 | 1b(cam-lidar) 선행 |
| scipy 깨짐(`numpy.dtype size changed`) | user-site에 numpy≥2/opencv-python≥4.10 넣지 말 것(0단계 주의) |

실기 운영 중 겪은 사례별 정리: [TROUBLESHOOTING.md](TROUBLESHOOTING.md).

### C. 상세 문서 색인
- 수집·bag·모니터·추출: [USAGE.md](USAGE.md) / LiDAR: [LIDAR.md](LIDAR.md)
- 캘리브: [CALIBRATION.md](CALIBRATION.md)(cam-cam) / [CAM_LIDAR_CALIBRATION.md](CAM_LIDAR_CALIBRATION.md)(cam-lidar)
- 매핑: [MAPPING.md](MAPPING.md)
- BEV 자동 라벨(규격·단계·주의·**§B 실행**, 구판은 부록 A): [BEV_AUTOLABEL.md](BEV_AUTOLABEL.md)
- 문제해결 사례: [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
