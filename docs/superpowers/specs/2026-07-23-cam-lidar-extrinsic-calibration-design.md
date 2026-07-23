# Cam–LiDAR Extrinsic 캘리브레이션 설계

어안 4대(DS 모델, intrinsic·카메라 간 extrinsic 완료)와 Unitree 4D LiDAR L2 사이의
**6DoF extrinsic**을 구해 `calib.yaml`에 추가하는 설계. 최종 용도는 **BEV 융합/학습용**이라
픽셀 수준(재투영 ~1–2px) 정렬을 목표로 한다.

- 선행: 카메라 캘리브(`docs/CALIBRATION.md`, `calib.yaml`), LiDAR 통합(`docs/LIDAR.md`),
  오프라인 매핑(`docs/MAPPING.md`, LIO 궤적).
- 브레인스토밍 경위·의사결정은 본 문서 §9에 기록.

---

## 1. 목표와 범위

- **구하는 것:** 라이다 프레임(`unilidar_lidar`) → 카메라 기준 프레임(`front`)의 강체 변환
  **`T_front_lidar`** (4×4) 하나. 나머지 3대는 `calib.yaml`의 카메라 간 extrinsic으로 체인 연결.
- **방법(확정):** 접근 **B — 경량 자체구현**. 정지 상태 라이다 누적 → **수동 2D–3D 대응점** →
  DS 기반 PnP(비선형 최소제곱)로 6DoF 해. 성숙한 오픈소스(A, `direct_visual_lidar_calibration`)는
  DS 모델 미지원 + arm64/헤드리스 부담으로 제외(§9).
- **2트랙(단계):**
  - **1단계(주):** 지금 통제 촬영(정지)으로 현재 장착에 대한 신뢰도 높은 extrinsic.
  - **2단계(조건부):** 이미 수집된 현장 bag에 1단계 결과를 투영·검증하고, 장착 드리프트가
    눈에 띄면 그 bag에서 미세보정. 같은 툴킷 재사용(누적만 모션 보정으로 교체).
- **범위 밖:** 자동 엣지/광도 정렬(향후 선택, §8), 시간 오프셋 추정(하드웨어 동기·공통 시간축
  전제, `DATASET.md`에서 확인됨), 카메라 intrinsic 재추정(고정 입력).

## 2. 좌표 규약과 수학

`calib.yaml`의 extrinsic은 `T_<name>_front` = **front 프레임 점 → name 프레임 점** 변환
(`ds_model.py`의 `CameraRig.T_cam_front`가 이 규약으로 로드). 투영 체인:

```
p_front = T_front_lidar · p_lidar            # 미지수 (우리가 푸는 것)
p_camC  = T_camC_front  · p_front            # calib.yaml (기존)
(u,v), valid = DS_project(p_camC, K_C)       # ds_model.DoubleSphereCamera.project (기존)
```

- 미지수 6DoF는 회전(축각 `rvec` 3) + 평행이동(`tvec` 3)으로 매개변수화, `T_front_lidar`로 합성.
- L2가 360° 스캔이라 **어느 카메라에서 보이는 대응점이든** 그 카메라의 `T_camC_front`+`K_C`로
  체인을 태워 잔차를 만들 수 있다. → **4대 대응점을 한 번에 넣어 `T_front_lidar` 하나를 공동
  최적화**(360° 이점 활용).
- **잔차:** 대응점 `i`(3D 라이다 점 `Pi`, 관측 픽셀 `(ui,vi)`, 관측 카메라 `Ci`)에 대해
  `r_i = DS_project(T_{Ci}_front · T_front_lidar · Pi) − (ui, vi)`.
  전체 목적함수 `Σ ρ(‖r_i‖²)` (`ρ`=Huber 등 robust loss), `scipy.optimize.least_squares`로 최소화.
- **회전 민감도:** 회전 1°는 10m에서 ≈17cm(수십 px). 회전이 지배 파라미터이므로 대응점은
  **거리·방향이 다양**하게(가까운/먼, 좌우/상하) 분포시켜야 회전이 잘 관측된다.

## 3. 아키텍처 (신규 도구)

모두 **호스트 파이썬**(numpy/scipy/opencv + open3d). 기존 관심사-분리 관례대로 작게 유지.
DS 투영은 `calibration/verify/ds_model.py`(`DoubleSphereCamera`, `CameraRig`, `load_rig`)를 재사용.

```
calibration/cam_lidar/
  accumulate_cloud.py     # bag → 밀집 라이다 클라우드(+대표 카메라 이미지)
  pick_correspondences.py # 이미지 2D 클릭 + 클라우드 3D 클릭 → correspondences.json
  solve_extrinsic.py      # DS-PnP(least_squares) → T_front_lidar + 재투영 RMS
  overlay_verify.py       # 라이다 점을 4대 이미지에 투영 오버레이 + RMS
  (shared)                # DS 체인 잔차 등 공용 로직(테스트 대상)
docs/CAM_LIDAR_CALIBRATION.md   # 운영 가이드
```

### 3.1 `accumulate_cloud.py`
- 입력: bag 경로, `--window <sec>`, (2단계) `--trajectory <tum>` `--at <sec>`.
- **1단계(정지):** `/unilidar/cloud`를 시간창만큼 **단순 병합** + voxel 다운샘플 → 밀집 클라우드
  (`unilidar_lidar` 프레임). 이동이 없으므로 프레임 간 변환 불필요.
- **2단계(이동):** 각 클라우드 프레임을 해당 시각의 LIO 자세로 **모션 보정** 후 병합
  (누적 기준 시각 `--at`의 라이다 프레임으로 되돌림). 궤적은 `mapping/` 산출물(`trajectory.tum`)
  또는 bag의 `/tf` 사용.
- 대표 시각의 카메라 4장은 기존 `bag_extract`로 추출(중복 구현 금지).
- 출력: 클라우드(`.pcd`/`.npy`, `x,y,z,intensity`) + 사용한 대표 프레임 인덱스/시각 기록.

### 3.2 `pick_correspondences.py`
- 입력: 밀집 클라우드, 카메라 이미지(들), `--cam <name>`(orientation 매핑).
- 상호작용: (a) 이미지 창에서 픽셀 클릭 `(u,v)`, (b) open3d 창에서 **같은 물리 점** 3D 클릭
  `(x,y,z)`. ~8쌍(권장 홀드아웃 2~3쌍은 검증용). 여러 카메라 혼용 가능(각 쌍에 `cam` 라벨).
- 출력: `correspondences.json` = `[{cam, uv:[u,v], xyz:[x,y,z]}...]`.
- **디스플레이 필요**(일회성 ~10분). 나머지 단계는 헤드리스.

### 3.3 `solve_extrinsic.py`
- 입력: `correspondences.json`, `calib.yaml`, `orientation.json`, (선택) `--init`(2단계 초기값).
- 처리: §2 잔차로 `least_squares`(robust loss). 초기값 없으면 대략 장착값 또는 OpenCV
  `solvePnP`(카메라별 언프로젝트한 방향 기반) 시드. → `T_front_lidar`.
- 출력: `T_front_lidar`(4×4) + **대응점별·전체 재투영 RMS(px)**, (있으면) 홀드아웃 RMS.

### 3.4 `overlay_verify.py`
- 입력: 클라우드, 카메라 이미지 4장, `calib.yaml`(+새 `T_front_lidar`), `orientation.json`.
- 처리: 체인으로 라이다 점을 **4대 이미지에 투영**, 깊이/강도 색으로 오버레이 PNG 저장 +
  (대응점 있으면) 재투영 RMS 재계산. `--out`으로 분리 저장(현장 이미지에도 적용).
- 판정: 기둥·박스 엣지·바닥–벽 경계가 라이다 점과 맞물리는지(4대 동시 = 체인+360° 검증).

## 4. 데이터 흐름

### 1단계 (통제 촬영, 현재 장착)
```
[배치]  리그 정지, 앞에 모서리 뚜렷한 물체(박스·구조물)를 라이다·카메라가 함께 보게
[녹화]  ros2 launch econ_camera_ros record_all.launch.py  (정지 ~10–20s) → bag  (신규 코드 0)
[추출]  bag_extract → 카메라 동기 세트 (기존 재사용)
[누적]  accumulate_cloud.py <bag> --window T → 밀집 클라우드
[대응]  pick_correspondences.py <cloud> <image...> → correspondences.json  (~8쌍, 디스플레이)
[풀기]  solve_extrinsic.py … → T_front_lidar + RMS
[검증]  overlay_verify.py … → 4대 오버레이 + RMS
[기록]  calib.yaml 에 T_front_lidar + verification 추가
```

### 2단계 (현장 bag 복원, 조건부)
1단계 오버레이 검증에서 회전 드리프트가 눈에 띄는 경우에만.
```
[선택]  양호 LIO bag (with-sun 계열·without-sun_2/4 — DATASET.md)
[누적]  accumulate_cloud.py <bag> --trajectory traj.tum --at t --window T  (모션 보정)
[대응]  pick_correspondences.py … (1단계 값을 참고 초기값으로)
[풀기]  solve_extrinsic.py … --init T_front_lidar(1단계)
[검증]  overlay_verify.py
```

## 5. `calib.yaml` 확장

`extrinsics`에 라이다 항목 추가(규약 일치: `T_<target>_<source>` = source→target):
```yaml
extrinsics:
  T_front_front: [...]        # 기존
  T_right_front: [...]        # 기존
  ...
  T_front_lidar:              # 신규: lidar 점 → front 프레임
  - [r11, r12, r13, tx]
  - [r21, r22, r23, ty]
  - [r31, r32, r33, tz]
  - [0, 0, 0, 1]
verification:
  reproj_rms_px: {...}        # 기존(카메라 intrinsic)
  cam_lidar:                  # 신규
    method: manual-pnp-ds
    date: 2026-07-23
    reproj_rms_px: <값>
    holdout_rms_px: <값>
    stage: 1                  # 1=통제촬영 / 2=현장bag별
```
- 파생 `T_camC_lidar = T_camC_front · T_front_lidar`는 필요 시 소비 측에서 계산(저장 선택).
- 2단계 bag별 결과는 해당 bag 산출물 폴더에 별도 `calib_camlidar.yaml`로 남길 수 있다
  (원본 `calib.yaml`은 1단계=현재 장착 기준 유지).

## 6. 의존성과 리스크

| 항목 | 내용 | 대응 |
|---|---|---|
| open3d (arm64) | 클라우드 IO·3D 점 클릭. Jetson aarch64 휠 가용성 불확실 | **최우선 스파이크**: 설치·픽킹 동작 확인. 실패 시 PCD 파서 + 경량 대체 피커(matplotlib 3D 등) |
| 3D 클릭 디스플레이 | 픽킹 단계만 GUI 필요 | 일회성 ~10분 모니터/X-forwarding. 나머지 헤드리스 |
| 2단계 LIO 품질 | 붕괴·드리프트 bag은 모션 보정 누적 부정확(DATASET.md) | 양호 bag만 사용. 2단계는 조건부 |
| 대응점 기하 다양성 부족 | 회전 관측 약화 → 정확도 저하 | 가까운/먼·좌우/상하 분산 + 홀드아웃 RMS로 감시 |

## 7. 검증 / 성공 기준

- **정량:** 대응점 재투영 RMS ≲ 2px, 가능하면 **홀드아웃 대응점**(솔브에 미사용)으로도 ≲ 2–3px.
- **정성:** 4대 오버레이에서 엣지(기둥·박스·바닥–벽)가 라이다 점과 맞물림 — 4대 동시 확인으로
  extrinsic 체인 + 360° 정합을 함께 검증.
- **교차 확인:** front 단독 해 vs 4대 공동 해가 근접(수 mm/0.x° 이내)하면 일관성 양호.
- **순수 로직 테스트:** 잔차·체인·PnP 수렴을 합성 데이터(알려진 `T`로 대응점 생성 → 복원)로
  `pytest` 검증(하드웨어 불필요). 실제 정합은 오버레이로 수동 확인.

## 8. 향후(범위 밖, 선택)

- **자동 엣지/광도 정렬:** 초기값(1단계 또는 장착값)에서 시작해 이미지 Canny 엣지 ↔ 클라우드
  깊이 불연속 엣지를 chamfer/상호정보량으로 정렬 → 수동 클릭 제거·2단계 자동화. 데이터가
  희소·반복적일 때 취약하므로 MVP 이후 필요성 확인 후 도입.
- intensity 기반 정렬(L2 intensity 품질 확인 후).

## 9. 의사결정 기록 (브레인스토밍)

- **비반복 L2 특성 확인:** 매 프레임 패턴이 달라지는 비반복 스캐너 → 정지 수 초 누적으로
  밀도 확보 필요(사용자 이해가 정확). intensity 필드 존재 확인(`x,y,z,intensity,ring,time`).
- **접근 A 제외:** `direct_visual_lidar_calibration`은 pinhole/fisheye/omni만 지원, **DS 미지원**.
  DS 재적합/언디스토션 마찰 + Iridescence GUI의 OpenGL(헤드리스 불리) + arm64 빌드 부담.
- **접근 B 채택:** DS 코드(`verify/ds_model.py`) 재사용, 호스트 파이썬(arm64 Docker 회피),
  코드베이스 관례 일치, 수동 대응 PnP의 신뢰성·이해도.
- **2단계 전략:** 장착을 뗐다 붙여 **회전이 살짝 틀어졌을 수 있음**(위치는 고정). 회전은
  거리 비례로 픽셀 오차가 커지는 지배 파라미터라, 현장 bag은 1단계 값으로 검증 후 필요 시
  그 bag에서 미세보정. 통제 촬영의 신뢰성 + 현장 데이터의 정합성을 분리 취득.
- **대응점 방식:** 수동 3D 클릭 채택(보드 자동검출은 어안 주변부 검출·평면분할 리스크로 보류).
- **최적화 범위:** 360° 활용 위해 4대 대응점 공동 최적화(front 단독 대비 관측성·검증성 우수).
