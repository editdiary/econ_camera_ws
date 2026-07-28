# 전체 파이프라인 가이드 (데이터 수집 → semi-auto BEV 라벨)

이 문서는 **처음 보는 사람이 데이터 수집부터 BEV occupancy 데이터셋 자동 라벨(semi-auto)까지
순서대로 따라 할 수 있게** 전 과정을 하나로 꿴다. 각 단계마다 **① 무엇을 하는가 ② 무엇을 실행
하는가 ③ 주요 옵션 ④ 결과물**을 적고, 깊은 내용은 해당 상세 문서로 링크한다.

- 최종 산출물: 어안 3대(front/left/right) 이미지 → **ego 중심 BEV occupancy 라벨(80×80, 0=obstacle/1=drivable/2=ignore)** 학습 데이터셋.
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
  4. LIO 매핑          lio_map_bag → map.pcd + trajectory.tum   (+궤적 건강성 확인)
  5. LiDAR auto-label  generate.py → dataset/sample_*/{label,review,cam_*,meta}
  6. 자체 annotation   export → CVAT(drivable 마스크) → gather-cvat → annotations/*/sample_*/cam_*.png
  7. IPM 융합 검수      ipm_review.py → sample_*/{review_combined, label_fused, meta_review}
  8. 최종 확정          사람이 label tool로 검수·재라벨 → 최종 BEV 데이터셋
```

| 단계 | 실행(대표) | 결과물 | 상세 문서 |
|---|---|---|---|
| 0. 환경/빌드 | `colcon build` | 실행 가능한 ws | [USAGE §2](USAGE.md), [LIDAR §2](LIDAR.md) |
| 1a. cam-cam | `run_kalibr.sh` → `calib_convert` | `calib.yaml`(intrinsic+`T_cam_front`) | [CALIBRATION.md](CALIBRATION.md) |
| 1b. cam-lidar | `pick_correspondences` → `solve_extrinsic` | `calib.yaml`에 `T_front_lidar` 추가 | [CAM_LIDAR_CALIBRATION.md](CAM_LIDAR_CALIBRATION.md) |
| 2. 수집 | `record_all.launch.py` | `rosbag2_*/`(mcap) | [USAGE §3·§9](USAGE.md), [LIDAR §3](LIDAR.md) |
| 3. 추출 | `bag_extract` | `frame_*/cam{0..3}.jpg` + `sets.csv` | [USAGE §6](USAGE.md) |
| 4. 매핑 | `lio_map_bag.sh` | `map.pcd` + `trajectory.tum` | [MAPPING.md](MAPPING.md) |
| 5. LiDAR 라벨 | `generate.py` | `sample_*/{label.png,review.png,cam_*.jpg,meta.json}` | [BEV_AUTOLABEL §A.4](BEV_AUTOLABEL.md) |
| 6. annotation | `dataset_flatten` + CVAT | `annotations/*/sample_*/cam_*.png` | [BEV_AUTOLABEL §A.6](BEV_AUTOLABEL.md) |
| 7. IPM 융합 | `ipm_review.py` | `sample_*/{review_combined.png,label_fused.png}` | [BEV_AUTOLABEL §A.6](BEV_AUTOLABEL.md) |
| 8. 최종 확정 | (label tool) | 최종 데이터셋 | [BEV_AUTOLABEL §10](BEV_AUTOLABEL.md) |

> **주기 구분**: 1a·1b(캘리브)는 **리그(카메라·라이다 장착)를 바꾸지 않는 한 1회**만 하고 이후
> 모든 bag이 그 `calib.yaml`을 공유한다. 2~8은 **수집한 bag마다** 반복한다.
>
> **폴더 규약**: 모든 산출물은 `data/`(gitignore) 아래로 모은다. bag별 3쌍을 같은 `<name>`으로 맞춘다 —
> `data/sj_bags/<날짜>/bags/<bag>` ↔ `.../maps/<name>_mapping`(매핑 산출) ↔ `data/extracted/<name>`(추출 이미지).
> `<name>`: `raws{N}`=with-sun, `rawos{N}`=without-sun. BEV 산출은 `data/bev/{dataset,review,annotations}/<name>`.

---

## 0단계. 환경 준비 & 빌드 (최초 1회)

**무엇**: ROS2 Humble 호스트 네이티브 + 이 ws 빌드 + 선행 프로젝트 순수 로직 재사용.

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
./mapping/lio_map_bag.sh rosbag2_<ts> data/sj_bags/<날짜>/maps/<name>_mapping
```

**주요 옵션**:
- `--max-secs N`: bag 재생을 N초에서 중단(LIO 발산/오염 구간 제외).
- `--no-preview`: 미리보기 PNG 생략.

**결과**: `map.pcd`(월드 밀집 클라우드) + `trajectory.tum`(pose `world_T_body`) + `run_info.txt` + `preview/`.

**건강성 판정(중요)**: `trajectory.tum` **총 길이를 실제 공간과 대조**한다. z가 안정해도 궤적이 붕괴하면 매핑 실패
(온실 긴 복도가 정답). 붕괴한 맵으로 라벨을 만들면 오염된다. 상세: [MAPPING.md](MAPPING.md).

---

## 5단계. LiDAR auto-label 데이터셋 생성

**무엇**: 맵+궤적+calib으로 **LiDAR 기반 BEV 라벨(0/1/2)** 초안을 키프레임마다 자동 생성한다.
핵심 아이디어 = "장애물(수직 구조)=obstacle, 보이는 그 외=drivable, 가려짐=ignore"
(LiDAR는 바닥을 못 보고 장애물만 잘 봄).

```bash
cd calibration/bev_autolabel

# (선택) 단계1 — 몇 프레임만 검증뷰로 육안 확인
mkdir -p ../../data/bev/review/<name>
python3 verify_labels.py \
  --map-dir ../../data/sj_bags/<날짜>/maps/<name>_mapping \
  --extract-dir ../../data/extracted/<name> \
  --calib ../../data/calib_260723/calib.yaml --orient ../../data/calib_260723/orientation.json \
  --frames 900 2000 2500 4850 --out ../../data/bev/review/<name>

# 단계2 — 키프레임 전체를 데이터셋으로 일괄 생성
python3 generate.py \
  --map-dir ../../data/sj_bags/<날짜>/maps/<name>_mapping \
  --extract-dir ../../data/extracted/<name> \
  --calib ../../data/calib_260723/calib.yaml --orient ../../data/calib_260723/orientation.json \
  --out ../../data/bev/dataset/<name> --kf-step 0.4
```

**주요 옵션**:
- `--kf-step 0.4`: 키프레임 이동거리 간격(m).
- `--z-gate 0.3`: obstacle 바닥근접 여유(**유일한 실질 레버**; 0.15면 통로 더 개방, 0.6은 캐노피 오검).
- `--limit N`: 스모크(앞 N개만).
- `T_front_lidar` 키가 없으면 CLI가 즉시 종료(1b 선행 필요).

**결과**: `data/bev/dataset/<name>/sample_NNNNNN/` 마다
- `label.png` — 순수 class(0/1/2) **인덱스 팔레트**(재라벨 원본), `review.png` — 검수뷰(ego 박스·미터축),
- `cam_{front,left,right}.jpg` — 원본 3이미지, `meta.json` — pose·stamp·BEV 규격·파라미터.
- 최상위 `dataset.csv`(sample↔frame↔stamp). 상세·규격·주의: [BEV_AUTOLABEL §A·§4·§7](BEV_AUTOLABEL.md).

---

## 6단계. 자체 annotation (drivable 마스크)

**무엇**: 사람이 각 sample의 **이미지 3장에 drivable 영역 마스크**를 직접 그린다(7단계 IPM의 입력).
sample마다 `cam_front/left/right` 이름이 겹쳐 한 폴더로 못 모으므로 **펼치고(export) → 작업 → 되돌린다(gather)**.

```bash
cd calibration/bev_autolabel

# 1) 펼치기(업로드용): 파일명에 sample 인코딩 → sample_NNNNNN__cam_{name}.jpg (이름 유일)
python3 dataset_flatten.py export --dataset-dir ../../data/bev/dataset/<name> --out <flat_imgs>

# 2) CVAT 등 annotation tool에서 drivable 마스크 작업 → export
#    CVAT는 "Segmentation mask 1.1"로 내보냄(labelmap.txt + SegmentationClass/, 클래스별 RGB 컬러)

# 3) 되돌리기 — CVAT: labelmap의 drivable 색만 정확히 255로 (설정 무관)
python3 dataset_flatten.py gather-cvat \
  --cvat-dir ../../data/bev/annotations/<name>/cvat_label \
  --out      ../../data/bev/annotations/<name>            # --class 기본 drivable
```

**주요 옵션/변형**:
- **CVAT면 `gather-cvat`** 를 쓴다: `SegmentationClass/*.png`는 0/1 이진이 아니라 클래스별 RGB 컬러이고
  색이 프로젝트 설정마다 달라진다 → `labelmap.txt`에서 클래스명(기본 `drivable`)의 색을 읽어 정확히 매칭(다른 클래스 오염 없음).
- **이미 0/255 이진 마스크**를 주는 툴이면 `gather`(파일명에 `sample_NNNNNN__cam_<name>`만 있으면 접미사 붙어도 인식, `>0`을 drivable로 이진화).

**결과**: `data/bev/annotations/<name>/sample_NNNNNN/cam_{front,left,right}.png`(흰=drivable). 마스크 그린 sample만 있어도 됨(부분 라벨 OK). 상세: [BEV_AUTOLABEL §A.6](BEV_AUTOLABEL.md).

---

## 7단계. IPM 투영 + LiDAR 융합 검수

**무엇**: 6단계의 이미지 마스크를 **바닥 평면(z=−H)에 역투영(IPM)** 해 BEV로 올리고, 같은 sample의 `meta.json`으로
**LiDAR 라벨을 재구성**해 겹쳐 본다. 두 정보의 오차 성격이 상보적(IPM=바닥엔 강하나 수직물체 번짐, LiDAR=장애물엔 강하나 바닥 못 봄)이라 겹쳐 놓고 검수한다.

> **IPM은 pose·map을 쓰지 않는다** — 오직 calib(DS intrinsic + `T_cam_lidar`) + 실측 카메라 높이 H + 마스크 픽셀만으로,
> "바닥은 평평하다" 가정 위에 각 프레임을 독립 투영한다. pose·map은 겹쳐 볼 LiDAR 라벨 재구성에만 쓰인다.

```bash
cd calibration/bev_autolabel
python3 ipm_review.py \
  --dataset-dir ../../data/bev/dataset/<name> \
  --map-dir     ../../data/sj_bags/<날짜>/maps/<name>_mapping \
  --mask-dir    ../../data/bev/annotations/<name> \
  --cam-height  0.87
```

**주요 옵션**:
- `--cam-height`: 렌즈의 바닥 위 높이(m). 마스트 LiDAR가 바닥을 못 봐 못 주므로 **자로 실측**(예 0.87). IPM 정확도의 핵심.
- `--near 2.0`: 하늘 후보 바닥 승격 반경. `--out` 미지정 시 각 sample 폴더에 기록.
- calib·orient·z_gate는 sample `meta.json`에서 자동으로 읽음(플래그로 덮어쓰기 가능).

**결과**: 각 `sample_NNNNNN/`에 추가 —
- `review_combined.png`(상단 3이미지 + 하단 LiDAR/IPM/융합 3패널; 격자·범례),
- `label_fused.png`(장식 없는 80×80 다색 카테고리 맵 — 겹침/불일치를 색으로 구분, **label tool에 바로 로드**),
- `meta_review.json`(카테고리 셀 수).
- 검수뷰 색: 밝은초록=둘 다 drivable / 빨강=LiDAR 장애물 / 주황=이미지바닥∩LiDAR장애물(→obstacle) / 하늘=이미지 후보바닥 / 어두운초록=LiDAR만 drivable / 회색=미확정. 상세: [BEV_AUTOLABEL §A.6](BEV_AUTOLABEL.md).

---

## 8단계. 최종 BEV 데이터셋 자체 annotation (사람 확정)

**무엇**: auto-label은 초안이다. 자동 라벨(5단계 `label.png` / 7단계 `label_fused.png`)이 **100%가 아니므로**,
사람이 BEV 위에서 검수·보정해 **최종 데이터셋을 확정**한다.

- **일치 셀은 자동 확정, 불일치(융합 검수뷰의 주황/하늘/회색)만 사람이 판단**하면 작업량이 준다.
- 재라벨 원본: 5단계 `label.png`(순수 class 인덱스 팔레트) 또는 7단계 `label_fused.png`를 label tool에 로드해 수정.
- 남은 한계(문서화된 것): IPM 평면 가정(원거리·수직물체 번짐), 전방 동적 물체 미처리, 어안→모델 입력 언디스토션 필요, 데이터 규모(일반화는 여러 bag/환경 확충 전제). 상세: [BEV_AUTOLABEL §7·§10](BEV_AUTOLABEL.md).

**결과**: 학습에 바로 쓰는 확정 BEV occupancy 데이터셋(이미지 3장 + 최종 라벨 + meta).

---

## 부록

### A. 순수 로직 테스트 (하드웨어 불필요)
```bash
cd src/econ_camera_ros && python3 -m pytest test/ -q          # 수집·추출 로직 18
cd calibration/cam_lidar && python3 -m pytest -q              # cam-lidar 18
cd calibration/bev_autolabel && python3 -m pytest -q          # bev_label 21 + ipm_review
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
- BEV 자동 라벨(규격·단계·주의·§A 실행): [BEV_AUTOLABEL.md](BEV_AUTOLABEL.md)
- 문제해결 사례: [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
