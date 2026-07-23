# Cam–LiDAR 캘리브 — 다음 세션 재개용 (RESUME)

> 이 파일 하나만 열면 어디서 멈췄고, 무엇을 시키고, 무엇을 확인할지 다 나온다.

## 지금 상태 (2026-07-23 기준)

- **브레인스토밍·설계·계획까지 완료. 구현 코드는 아직 0줄.**
- 브랜치 **`feat/cam-lidar-calib`** 에 2개 커밋:
  - 스펙: `docs/superpowers/specs/2026-07-23-cam-lidar-extrinsic-calibration-design.md`
  - 계획: `docs/superpowers/plans/2026-07-23-cam-lidar-extrinsic-calibration.md` ← **작업 대본**
- 다음 할 일 = 계획의 **Task 0**(open3d 스파이크)부터 순서대로.

## 다음 세션에서 이렇게 시작하면 됨 (복붙용 프롬프트)

```
feat/cam-lidar-calib 브랜치에서 이어서 진행할게.
docs/superpowers/plans/2026-07-23-cam-lidar-extrinsic-calibration.md 계획을
subagent-driven-development 로 Task 0부터 실행해줘.
```

- 계획의 각 Task는 **체크박스(`- [ ]`)** 로 되어 있어 진행 추적이 된다. 완료한 Task는 체크된다.
- 실행 스킬: **superpowers:subagent-driven-development**(권장, 태스크마다 리뷰) 또는
  **superpowers:executing-plans**(인라인, 체크포인트).
- 먼저 `git switch feat/cam-lidar-calib` 확인.

## Claude가 할 일 (코드) — 순수 로직은 하드웨어 없이 끝까지 가능

| Task | 내용 | 하드웨어/디스플레이 필요? |
|---|---|---|
| 1–5 | `chain.py`(SE3·투영·PnP)·`cloud_io.py`(파싱·누적·모션보정)·`calib_io.py` + **pytest** | ❌ 불필요 (합성 데이터로 검증) |
| 3 | `verify/ds_model.load_rig` 하드닝(`T_front_lidar` 오파싱 버그 수정) | ❌ |
| 6,7,9 | CLI(`accumulate_cloud`·`solve_extrinsic`·`overlay_verify`) 작성 | 작성은 ❌, **스모크는 bag 필요** |
| 8 | `pick_correspondences`(2D+open3d 3D 클릭) 작성 | 작성은 ❌, **스모크는 디스플레이 필요** |
| 10 | 문서·인덱스 | ❌ |

→ **Task 1–5, 3, 10과 6/7/9의 코드 작성까지는 다음 세션에 사람 없이 완주 가능.**
   실기 검증이 필요한 지점에서 멈춰 아래 "사용자가 할 일"을 안내하게 된다.

## 사용자(당신)가 할 일 — Claude가 대신 못 하는 실기

돌아온 뒤, 코드가 준비되면 아래를 준비/수행하면 된다:

1. **Task 0 확인**: 디스플레이(또는 X-forwarding) 붙은 상태에서 open3d 3D 픽킹이 뜨는지.
   안 되면 계획의 "대체 경로"(matplotlib 피커)로 자동 전환된다.
2. **1단계 통제 촬영**: 리그 완전 고정, 앞에 **모서리 뚜렷한 물체(박스·구조물)** 를 라이다·카메라가
   함께 보게 배치 → `ros2 launch econ_camera_ros record_all.launch.py` 로 **정지 10–20초** 녹화.
3. **대략 장착값 준비**: 라이다가 카메라 대비 대략 어느 방향/위치인지 rpy(도)·xyz(m).
   → `solve_extrinsic --init-rpy R P Y --init-xyz X Y Z` 의 초기값. (자로 잰 근사치면 충분)
4. **대응점 클릭**(~10분, 디스플레이): `pick_correspondences` 로 이미지·클라우드에서 같은 점
   ~8쌍(+홀드아웃 2–3). 가까운/먼·좌우/상하로 분산(회전 관측).

## 완료 후 무엇을 확인하면 되나 (성공 판정)

- [ ] **코드 검증(자동)**: `cd calibration/cam_lidar && python3 -m pytest -q` 전부 통과 +
      기존 `calibration/verify`·`src/econ_camera_ros` 테스트 회귀 없음.
- [ ] **핵심 정확성**: 합성 대응점으로 `solve` 가 알려진 extrinsic 을 **<1e-3 px** 복원(Task 2 테스트).
- [ ] **실기 정량**: `solve_extrinsic` 의 **train/holdout RMS ≲ 2px**.
- [ ] **실기 정성**: `overlay_verify` 산출 **4대 오버레이 PNG**에서 기둥·박스 엣지·바닥-벽 경계가
      라이다 점과 맞물림(4대 동시 = 체인+360° 검증).
- [ ] **산출물**: `calib.yaml` 에 `extrinsics.T_front_lidar` + `verification.cam_lidar` 기록됨.
- [ ] (조건부) 오버레이에서 회전 드리프트가 보이면 → 계획 **§6/Task 재사용**으로 현장 bag별
      **2단계 미세보정**(모션 보정 누적 후 재대응). 양호 LIO bag만(DATASET.md: with-sun 계열·without-sun_2/4).

## 주의/미해결

- **open3d arm64**: 최대 리스크. Task 0에서 먼저 판정, 실패 시 대체 피커로 전환(대응점 형식 동일).
- **장착 회전 드리프트**: 라이다를 뗐다 붙여 회전이 살짝 틀어졌을 수 있음 → 1단계 값은 "현재 장착"
  기준. 현장 bag 복원은 검증 후 필요시 2단계.
- **병합·푸시는 직접**: Claude는 브랜치 커밋까지만. 작업 끝나면 당신이 리뷰 후 병합/푸시.
