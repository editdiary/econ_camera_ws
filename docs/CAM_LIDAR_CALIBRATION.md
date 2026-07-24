# Cam–LiDAR Extrinsic 캘리브레이션 가이드

어안 카메라 4대(front/right/rear/left, DS 모델)와 Unitree L2 라이다 사이의 상대 자세
**`T_front_lidar`**(라이다 점 → front 카메라 프레임)를 수동 2D–3D 대응점 + DS-PnP(`scipy.optimize.
least_squares`, Huber loss)로 구해 `calib.yaml`에 기록하는 실전 절차. `docs/CALIBRATION.md`로
구한 카메라 간 extrinsic(카메라↔카메라)에 라이다를 체인으로 붙이는 마지막 한 조각이다.

- 설계 근거: [`superpowers/specs/2026-07-23-cam-lidar-extrinsic-calibration-design.md`](superpowers/specs/2026-07-23-cam-lidar-extrinsic-calibration-design.md)
- 구현 계획: [`superpowers/plans/2026-07-23-cam-lidar-extrinsic-calibration.md`](superpowers/plans/2026-07-23-cam-lidar-extrinsic-calibration.md)
- 도구 위치: `calibration/cam_lidar/`(`chain.py`·`cloud_io.py`·`calib_io.py` + 4개 CLI).
  순수 로직 18개 테스트: `cd calibration/cam_lidar && python3 -m pytest -q`.

**규약**: `T_<target>_<source>` = source 점 → target 점. 투영 체인은
`pixel = cam.project(T_cam_front · T_front_lidar · P_lidar)`이며, `T_cam_front`는 기존
`calib.yaml`의 카메라 간 extrinsic(`orientation.json`으로 이름 매핑)에서 그대로 읽는다. front
카메라 자신은 `T_cam_front = I`이므로 라이다→front 는 `T_front_lidar` 단독으로 검증 가능하다.

---

## 0. 사전 준비

```bash
pip install open3d scipy opencv-python
```

- open3d는 이 Jetson AGX Orin(arm64) 호스트에서 **설치 성공(0.18.0)** 이 확인됨 — 상세는
  `calibration/cam_lidar/README.md`. 단 호스트가 헤드리스(DISPLAY 미설정)라 실제 3D 픽킹 창은
  디스플레이 또는 X-forwarding 이 붙은 상태에서만 뜬다(SSH -X 등).
- `docs/CALIBRATION.md`로 만든 카메라 캘리브 산출물이 미리 있어야 한다: `calib.yaml`(intrinsic +
  카메라 간 extrinsic) + `orientation.json`(예: `data/calib_260723/{calib.yaml,orientation.json}`,
  `cam0..3 = front/right/rear/left`).
- LiDAR가 이미 `docs/LIDAR.md` 절차로 카메라와 같은 bag에 녹화 가능한 상태여야 한다
  (`/unilidar/cloud` 토픽).

---

## 1. 촬영 (1단계, 정지)

리그를 **완전히 고정**한 상태로, 모서리·경계가 뚜렷한 물체(박스, 벽-바닥 경계, 기둥 등)를
라이다와 카메라 여러 대가 **동시에** 보도록 배치한다. 리그가 움직이면 안 된다(모션 보정 없이
단순 시간창 누적만 하기 때문).

```bash
ros2 launch econ_camera_ros record_all.launch.py    # 정지 상태 10-20초, Ctrl-C 로 종료
```

→ `rosbag2_<timestamp>/`에 카메라 4토픽 + `/unilidar/cloud`·`/unilidar/imu` + `/tf`·`/tf_static`
가 함께 담긴다.

---

## 2. 클라우드 누적 + 대표 이미지

```bash
# 정지 촬영이므로 --trajectory 없이 시간창만 지정(기본 3초, 첫 스캔 기준)
python3 calibration/cam_lidar/accumulate_cloud.py <bag_dir> -o cloud.npy --window 3

# 대응점 클릭에 쓸 대표 프레임(카메라 4장) 추출
ros2 run econ_camera_ros bag_extract <bag_dir> -o extracted
```

`accumulate_cloud.py`는 `/unilidar/cloud`의 PointCloud2를 파싱해 `--window`초 구간을 병합하고
`--voxel`(기본 0.02m) 격자로 다운샘플한 뒤 `(N,4)` = `x,y,z,intensity` 배열을 `.npy`로 저장한다.
출력에 찍히는 `ref_stamp_ns`와 점 범위(x/y/z min-max)로 클라우드가 비정상(빈 스캔·범위 이상)이
아닌지 먼저 확인한다. `extracted/frame_NNNNNN/`중 `ref_stamp_ns`에 가까운 시각의 프레임을
3절 대응점 클릭에 사용한다.

---

## 3. 대응점 수집 (디스플레이 필요)

카메라 1대씩, 같은 물리적 점을 이미지에서 먼저 클릭하고 이어서 클라우드에서 같은 순서로 클릭한다.

```bash
python3 calibration/cam_lidar/pick_correspondences.py \
  --cloud cloud.npy --image extracted/frame_XXXXXX/cam2.jpg \
  --cam front --out corrs.json --append
```

- `--cam`은 `orientation.json`의 이름(front/right/rear/left)과 일치해야 한다.
  `cam{0..3}.jpg` 중 어느 파일이 어느 이름인지는 `orientation.json`을 따른다
  (예: `cam0=front, cam1=right, cam2=rear, cam3=left`이면 front 대응점은 `cam0.jpg`).
- 흐름: 이미지 창(OpenCV, 좌클릭=점 추가, 아무 키나 누르면 종료) → open3d 창
  (`VisualizerWithEditing`, shift+좌클릭=점 선택, 창을 닫으면 종료). 2D 클릭 개수와 3D 클릭
  개수가 다르면 도구가 경고하고 앞쪽 짧은 쪽 개수만큼만 짝을 짓는다 — **순서가 어긋나면 잘못된
  쌍이 조용히 저장되므로 반드시 같은 순서로 클릭**한다.
- 카메라 4대 모두 반복하며 매번 `--append`로 같은 `corrs.json`에 이어붙인다.
- 카메라 1대당 학습용 ~8쌍 + 검증(홀드아웃)용 2~3쌍을 목표로 하고, 점들을
  **가까운/먼 거리·좌우·상하로 분산**시켜야 회전이 관측된다(한 평면·한 거리에 몰리면 PnP가
  회전 축을 못 구분).

---

## 4. 풀기 + calib.yaml 기록

```bash
python3 calibration/cam_lidar/solve_extrinsic.py corrs.json \
  --calib data/calib_260723/calib.yaml --orientation data/calib_260723/orientation.json \
  --init-rpy <대략 장착 roll pitch yaw(도)> --init-xyz <대략 장착 x y z(m)> \
  --holdout 3 --stage 1
```

- `--init-rpy`/`--init-xyz`는 라이다가 front 카메라 프레임 기준으로 대략 어느 방향·위치에
  있는지 자로 잰 근사치(도/미터)면 충분하다 — 처음 실행이라 `calib.yaml`에 기존
  `T_front_lidar`가 없으면 **필수**. 이미 값이 있으면(2단계 재보정 등) 생략 가능하고 그 값을
  초기값으로 쓴다.
- `--holdout N`으로 지정한 개수만큼 `corrs.json`의 **뒤쪽 N개**를 학습에서 빼고, 학습된 T로
  재투영만 해서 별도 RMS를 낸다(과적합 확인용). 홀드아웃 대응점이 카메라별로 골고루 섞여
  있도록 `corrs.json` 순서에 유의한다.
- **정량 판정은 이 도구가 출력하는 숫자로 한다**: `train RMS = ... px`와(홀드아웃을 줬다면)
  `holdout RMS = ... px`. **판정 기준: train/holdout RMS 둘 다 ≲ 2px.**
- `--no-write`를 주면 `calib.yaml`을 건드리지 않고 값만 미리 본다. 기본(옵션 없음)은
  `calib.yaml`의 `extrinsics.T_front_lidar`(4×4)와 `verification.cam_lidar`
  (`method`, `reproj_rms_px`, `stage`, 있으면 `holdout_rms_px`)를 기록한다 — 카메라 블록은
  건드리지 않는다.

---

## 5. 검증 (정성, 시각)

```bash
python3 calibration/cam_lidar/overlay_verify.py \
  --cloud cloud.npy --frame extracted/frame_XXXXXX \
  --calib data/calib_260723/calib.yaml --orientation data/calib_260723/orientation.json \
  --out overlay
```

→ `overlay/overlay_{front,right,rear,left}.png` 4장. 라이다 점을 깊이(turbo 색상, `--dmax`
기본 10m)로 칠해 4대 이미지에 투영한 오버레이다.

**주의: `overlay_verify.py`는 정성(시각) 검증 전용이며 RMS를 계산하지 않는다.** 정량 RMS는
반드시 4절의 `solve_extrinsic.py`가 출력한 train/holdout 값으로 판정한다. 이 절에서는 눈으로
**4대 모두** 기둥·박스 엣지·바닥-벽 경계가 라이다 점과 맞물리는지만 본다(4대 동시에 맞아야
카메라 간 extrinsic 체인 전체와 라이다 회전이 함께 검증된다). 특정 카메라만 어긋나면 해당
카메라의 대응점을, 전체가 통째로 어긋나면 라이다 자체 정렬(6절)을 의심한다.

---

## 6. 2단계 (현장 bag 모션 보정, 조건부)

5절 오버레이에서 **회전 드리프트**(엣지가 일관되게 한쪽으로 밀림)가 보이면, 실제 수집에
쓸 양호한 LIO bag(`docs/MAPPING.md`의 `DATASET.md` 기준으로 궤적이 정상인 bag)에서 이동 중
스캔을 모션 보정해 다시 누적한다.

```bash
python3 calibration/cam_lidar/accumulate_cloud.py <field_bag> -o fcloud.npy \
  --trajectory traj.tum --at <기준 시각(초)> --window 2
```

- `--trajectory`를 주면 `accumulate_motion`(TUM 궤적 기반)이 각 스캔을 `--at` 기준 시각의
  포즈로 되돌려 누적한다(정적 `accumulate_static` 대신). `traj.tum`은 `mapping/`
  (`docs/MAPPING.md`)의 LIO 산출물을 그대로 쓴다.
- 이어서 3–5절을 반복한다. 단 `solve_extrinsic.py`는 `--init-rpy`/`--init-xyz`를 **주지 말고**
  실행해 `calib.yaml`에 이미 있는 1단계 `T_front_lidar`를 초기값으로 미세 보정한다.

---

## 7. 결과 판정 / 문제해결

| 증상 | 원인 | 대응 |
|---|---|---|
| train/holdout RMS ≲ 2px, 4대 오버레이 엣지 정합 | 정상 | `calib.yaml`의 `extrinsics.T_front_lidar`(+ `verification.cam_lidar`) 확정 |
| RMS 크게 나옴(수~수십 px) | 대응점 기하 다양성 부족(한 평면/한 거리에 몰림) 또는 오클릭(2D·3D 순서 어긋남) | 가까운/먼·좌우/상하로 분산해 재수집, 클릭 순서 재확인 |
| 특정 카메라만 재투영 어긋남 | 그 카메라의 대응점이 부족하거나 오클릭 | 해당 카메라만 대응점 추가 후 재-solve |
| 투영이 4대 전부 통째로 크게 어긋남(방향 자체가 틀림) | `--init-rpy` 부호/축 실수(라이다가 실제로 보는 방향과 반대로 줌) | 라이다 정면 방향을 다시 확인해 `--init-rpy` 부호·축 재확인 후 재-solve |
| 5절 정성 검증에서 회전 드리프트만 보임 | 정지 촬영 당시 장착 각도와 실제 사용 시 미세 차이(라이다를 뗐다 붙인 경우 등) | 6절 2단계(현장 bag 모션 보정)로 미세 보정 |
| `pick_correspondences.py`에서 open3d 3D 창이 안 뜸 | 호스트 헤드리스(DISPLAY 미설정) | X-forwarding(`ssh -X`) 붙여서 실행하거나 디스플레이가 있는 세션에서 실행 |

---

## 참고: calib.yaml 확장 필드

```yaml
extrinsics:
  T_front_lidar:                # 신규: lidar 점 → front 프레임 (4x4)
    - [ .., .., .., .. ]
    - [ .., .., .., .. ]
    - [ .., .., .., .. ]
    - [0, 0, 0, 1]
verification:
  cam_lidar:
    method: manual-pnp-ds
    reproj_rms_px: <train RMS>
    holdout_rms_px: <holdout RMS>   # --holdout 을 줬을 때만
    stage: 1                       # 1=정지, 2=현장 bag 모션 보정
```

카메라 블록(`cameras`, 카메라 간 `extrinsics`)은 그대로 유지되며 `write_extrinsic`
(`calibration/cam_lidar/calib_io.py`)은 이 두 키만 병합해 쓴다. 다른 카메라의
`T_camC_lidar`가 필요하면 `T_camC_front · T_front_lidar`로 소비 측에서 계산한다
(별도 저장하지 않음).
