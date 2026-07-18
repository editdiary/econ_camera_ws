# 카메라 캘리브레이션 & 검증 설계 (어안 4대, LiDAR는 나중)

- 작성일: 2026-07-18
- 상태: 설계 (카메라 수집은 구현 완료, 캘리브 절차·검증은 미구현)
- 목적: 4대 어안 카메라의 Intrinsic/Extrinsic 캘리브레이션을 **어떻게 수행하고 어떻게 검증**하는지
  확정한다. cam↔LiDAR는 L2 도착 후로 미루되 설계상 자리를 남긴다.
- 관련 문서: `2026-07-18-data-collection-bag-and-fusion-design.md` (정적/동적 데이터 원칙,
  캘리브는 정적 상수라 사후 계산 무손실), `2026-07-16-econ-camera-ros2-capture-design.md`

---

## 1. 배경과 핵심 제약

BEV 학습 데이터셋을 위해 4대 카메라 + (나중) LiDAR의 캘리브레이션이 필요하다. 캘리브레이션은
**정적 상수**(리그 고정 시 불변)라 촬영을 끝낸 뒤 계산해도 무손실이며, `calib.yaml` 사이드카로
둔다(별도 topic/bag 불필요). 이 문서는 그 "계산"을 실제로 어떻게 하고 검증하는지를 다룬다.

두 가지 하드웨어 사실이 설계를 좌우한다:

1. **카메라 4대가 4방향을 향하지만 인접 카메라 간 시야(FOV) 겹침이 크다.**
   → 카메라 간 extrinsic을 **지금(카메라만으로)** Kalibr multi-cam으로 풀 수 있다.
2. **각 렌즈가 ~180° 어안이다.**
   → 핀홀+radtan 모델은 주변부에서 무너진다. **전용 광각 모델(Double Sphere / eUCM)** 을 써야 한다.
   → 어안은 **주변부 커버리지**가 정확도를 좌우한다(가장자리 왜곡이 가장 크고 제약이 가장 적음).

---

## 2. 범위와 순서

| 단계 | 대상 | 시점 | 방법 |
|---|---|---|---|
| **A. Intrinsic** | 4× 어안 K·왜곡계수 | **지금 (카메라만)** | Kalibr, 카메라별 |
| **B. 카메라 간 Extrinsic** | 4대 상대 자세(링) | **지금** | Kalibr multi-cam (겹침 이용) |
| **C. cam↔LiDAR Extrinsic** | LiDAR 공통 프레임 | **L2 도착 후** | 별도(본 문서 범위 밖, 자리만 남김) |

- **A→B 순서**: 카메라별 intrinsic을 먼저 확정·검증한 뒤 multi-cam extrinsic으로 넘어간다.
  하드웨어 `frame_sync`로 4대가 동기돼 있어 multi-cam의 동시 관측 프레임 정합에 유리하다.
- **B의 가치는 나중에도 이어진다**: C(각 카메라→LiDAR)를 구하면 LiDAR 프레임을 거쳐 유도한
  카메라 간 자세가 B와 일치해야 하므로, B는 **C의 교차검증 기준**이 된다.

---

## 3. 도구·환경

### 3.1 Kalibr는 Docker로 실행 (ROS1 도구 ↔ ROS2 환경 브릿지)

Kalibr는 ROS1(Noetic) 기반이라 ROS2 Humble + `CompressedImage` + mcap 환경에 그대로 안 물린다.
경로:

```
[수집]  기존 capture 파이프라인으로 캘리브 시퀀스 녹화 (mcap, CompressedImage 4토픽, 하드웨어 동기)
   │
   ├─ bag_extract 로 동기 세트 JPEG 추출 (frame_NNNNNN/cam{0..3}.jpg + sets.csv)
   │
[브릿지]  추출 JPEG → ROS1 bag 변환   ← 신규 (kalibr_bagcreater 활용)
   │        cam0..3/<timestamp_ns>.png 폴더 구조 → .bag
   │
[Kalibr]  Docker(arm64) 컨테이너에서 offline 실행 → intrinsic/extrinsic + 리포트(PDF/yaml)
```

- **신규 작업물**: 추출 동기 세트 → ROS1 bag 브릿지. 나머지(capture, bag_extract)는 재사용.
- Kalibr는 **ROS1 `.bag` + `sensor_msgs/Image`(비압축)** 를 먹는다. JPEG 디코드는 브릿지 단계에서 처리.
- 브릿지는 Kalibr 내장 유틸 **`kalibr_bagcreater`** 를 활용한다. 추출 세트를 아래 폴더 구조
  (타임스탬프 파일명)로 정리하면 bag이 생성된다:
  ```
  dataset/cam0/<timestamp_ns>.png   dataset/cam1/...   dataset/cam2/...   dataset/cam3/...
  ```
- multi-cam 캘리브는 4대 프레임이 **같은 타임스탬프로 묶여** 입력돼야 한다(하드웨어 동기 + `sets.csv`
  기반 동기 세트가 이 요건을 충족).

**arm64(aarch64) 대응 — 네이티브 빌드로 확정.** 플랫폼이 Jetson AGX Orin(`aarch64`)이라 흔한 x86
Kalibr 이미지가 안 돈다. Kalibr를 clone해 Dockerfile의 베이스를 `arm64v8/ros:noetic`으로 지정하고
소스 빌드한다(qemu 에뮬레이션은 대안이나 채택 안 함):
```
# Kalibr repo에서
docker build -t kalibr:arm64 -f Dockerfile_ros1_20_04 .
```
- Docker는 이미 설치돼 있음(v29.5.2). 현재 계정은 `docker` 그룹 밖 → `sudo` 필요하거나
  `sudo usermod -aG docker $USER` 후 재로그인.
- 컨테이너 내 catkin workspace 경로(`source .../devel/setup.bash`)는 clone한 Dockerfile에서
  실제 빌드 위치를 확인해 사용한다(공식 `Dockerfile_ros1_20_04`는 통상 `/catkin_ws`).

### 3.2 카메라 모델 — DS와 eUCM 둘 다 비교

어안 전용 모델 후보 중 **Double Sphere(`ds`)** 와 **eUCM(`eucm`)** 두 가지를 채택 후보로 둔다.

- **추가 촬영 불필요**: 같은 녹화 데이터에 Kalibr를 모델만 바꿔 **offline 2회** 실행.
  - `--models ds-none ds-none ds-none ds-none`
  - `--models eucm-none eucm-none eucm-none eucm-none`
- 두 리포트의 **재투영 오차 RMS·잔차 산포**를 비교해 더 나은 쪽을 채택.
- 둘 다 만족스럽지 않으면 KB(equidistant, `pinhole-equi`)로 확장 검토(모델은 재계산으로 교체 가능).

### 3.3 타깃

보유 중인 **튼튼한 판 + AprilGrid** 사용(양면 체커/AprilGrid 장비). AprilGrid는 **부분적으로 잘려도
검출**되므로 어안 주변부 커버리지에 유리 → AprilGrid 면을 사용.

Kalibr용 `aprilgrid.yaml` — 실측으로 확정됨(태그 7×5, 태그 4cm, 태그 사이 검은 사각형 1cm):

```yaml
target_type: 'aprilgrid'
tagCols: 7          # 가로 태그 개수
tagRows: 5          # 세로 태그 개수
tagSize: 0.04       # 태그 한 변 [m] = 4cm
tagSpacing: 0.25    # (태그 사이 간격 1cm) / (태그 4cm) = 0.25
```

### 3.4 Kalibr 실행 커맨드 (4대·어안·헤드리스)

컨테이너 안에서 workspace를 source한 뒤, **같은 bag에 모델만 바꿔 2회** 실행한다.

```bash
# (컨테이너 내부) 예: source /catkin_ws/devel/setup.bash

# Double Sphere
rosrun kalibr kalibr_calibrate_cameras \
  --bag /data/calib.bag \
  --target /data/aprilgrid.yaml \
  --models ds-none ds-none ds-none ds-none \
  --topics /cam0/image_raw /cam1/image_raw /cam2/image_raw /cam3/image_raw \
  --approx-sync 0.001 \
  --dont-show-report

# eUCM 비교 실행 (--models 만 교체)
#   --models eucm-none eucm-none eucm-none eucm-none
```

- **`--models`** 는 카메라 수(4)만큼 나열. DS/eUCM 리포트를 비교해 채택(§3.2).
- **`--topics`** 는 브릿지가 bag에 넣은 실제 토픽명과 일치해야 함.
- **`--approx-sync 0.001`** — 하드웨어 동기 sub-ms를 감안해 4대 프레임을 묶음.
- **`--dont-show-report`** — 헤드리스/SSH 대응. GUI 없이 `report-*.pdf` + `*-camchain.yaml`을
  `/data`에 저장(X11(`xhost`/`DISPLAY`) 없이 동작).

---

## 4. 촬영 절차 (랩/현장)

캘리브 시퀀스는 기존 `capture` 파이프라인으로 녹화한다(별도 캡처 코드 불필요).

- **주변부까지 꽉 채운 커버리지** — 타깃을 화면 극단 모서리·코너까지 이동. 어안은 여기가 생명.
- **천천히·부드럽게** — 모션블러 최소화(어안+블러는 검출 실패의 주원인).
- **Kalibr 입력은 ~4Hz로 다운샘플** — 30fps 원본에서 중복/블러 프레임을 줄여 최적화 안정화.
  (다운샘플은 브릿지 단계에서 프레임 솎기로 처리.)
- **세션 앞뒤로 캘리브 시퀀스 1회씩** — 사족보행 리그의 진동 드리프트를 감지(앞/뒤 결과가
  일치하면 그 세션 데이터 신뢰, 어긋나면 드리프트 발생으로 판단).
- **학습용/검증용 시퀀스 분리 촬영** — 캘리브를 푸는 데이터와 검증하는 데이터를 분리(§6).
  같은 데이터로 잰 재투영 오차는 낙관적이므로 반드시 held-out 세트로 검증한다.

---

## 5. 커버리지/품질 체크 — Kalibr 검출 기반

Kalibr는 **완전 offline 배치** 도구다: 녹화 중에는 피드백이 없다. 촬영 품질(커버리지·검출)은
**Kalibr 실행 결과로 판단**한다.

**결정 이력(2026-07-18 실기):** 초기에는 별도 경량 게이트(`calib_coverage`, cv2.aruco AprilGrid
검출 + 커버리지 히트맵)를 두려 했으나, **실기 어안 프레임에서 cv2.aruco도 pupil-apriltags도
검출에 실패**함을 확인했다(어안 왜곡으로 태그 사각형이 휘고, 회색 배경 저대비, 태그 픽셀 과소).
독립 검출기는 신뢰할 수 없어 **은퇴**시키고, **Kalibr 자체 검출을 권위 소스로 사용**한다.

- 커버리지/검출 판단 = Kalibr 실행 시 나오는 **카메라별 코너/태그 검출 수** + report PDF의
  **관측 코너 분포(coverage) 플롯**. 검출 수가 적거나 주변부가 비면 → **재촬영**.
- 촬영 품질 요건(§4)을 지키는 게 1차 방어선: 보드를 **화면의 큰 비중으로(가까이)**, 태그가
  충분한 픽셀(권장 한 변 ≥ ~50–80px)로 보이게, 선명하게, 여러 포즈로.

> 즉 "찍은 직후" 체크는 현장에서 Kalibr를 한 번 돌려 검출 수/커버리지 플롯을 확인하는 것으로
> 갈음한다(Jetson+Docker가 현장에 있으므로 가능). 별도 cv2.aruco 게이트는 두지 않는다.

---

## 6. 검증 프로토콜

**원칙: 캘리브를 푼 데이터가 아니라 별도 held-out 시퀀스로 검증한다.**

**구현 단계(phasing):** Kalibr는 재투영 RMS·잔차 플롯·**쌍별(pairwise) 재투영 오차**를 리포트로
직접 출력하며, 이것이 실무 표준 검증 신호다. 따라서 **(A) 지금**은 Kalibr 내장 리포트 해석 +
정성 육안(6.1-3)으로 검증하고, calib.yaml에 RMS를 기록한다. **(B) 나중**에 필요성이 확인되면
독립 검증 도구(6.2의 루프 일관성·cross-projection)를 추가한다 — 이들은 Kalibr가 안 주는 루프
닫기 재실행·프레임별 보드 자세 추출이 필요해 별도 작업이다(§9).

### 6.1 Intrinsic 검증 (카메라별)

1. **재투영 오차** — Kalibr 리포트의 카메라별 RMS(어안이라도 sub-pixel 지향).
2. **잔차 산포의 무작위성** — 잔차 플롯이 무작위 구름이어야 한다. **방사형/체계적 패턴이 보이면
   모델 부적합 신호**(DS↔eUCM 교체 또는 KB 검토).
3. **왜곡 보정 직선성** — 직선 구조(문틀·타일 등)를 보정한 뒤 **직선이 직선으로** 펴지는지,
   특히 **주변부**에서 확인.

### 6.2 카메라 간 Extrinsic 검증

1. **Cross-projection(겹침 영역)** — held-out 세트에서, A 카메라가 본 타깃 코너의 3D 추정을
   `T_AB`로 B 카메라에 투영 → 실제 관측 위치와의 **픽셀 오차** 측정.
2. **루프 일관성** — front→right→back→left→front 상대자세 합성이 **항등(identity)에 근접**해야
   하며, 그 잔차(회전·병진)를 드리프트 지표로 본다.
3. **정성 스티칭** — 4뷰를 공통 지면/파노라마로 워프해 겹침에서 **이중상(double vision)** 이
   없으면 OK(정성적이지만 링 구조 정합을 직관적으로 드러냄).

### 6.3 cam↔LiDAR 검증 (L2 도착 후, 범위 밖)

- 점군을 이미지에 투영해 **물체 경계/깊이 불연속이 영상 엣지와 정합**하는지.
- 점군을 이미지 색으로 **컬러라이즈**해 육안 검사.

---

## 7. 산출물 — `calib.yaml` 사이드카

데이터셋 옆에 두는 정적 캘리브 파일.

```
카메라 4대 intrinsic       # 채택 모델(ds 또는 eucm)과 파라미터, 왜곡계수
카메라 간 extrinsic        # 링 구조 상대 자세(기준 카메라 대비 변환)
cam↔LiDAR extrinsic        # (나중) L2 도착 후 채움
```

- 채택한 카메라 모델명과 파라미터, 검증 결과(재투영 RMS·루프 잔차)를 함께 기록해 추적성을 남긴다.

---

## 8. 신규 구현 항목 요약

1. **arm64 Kalibr Docker 이미지** — Kalibr clone + `arm64v8/ros:noetic` 베이스로 소스 빌드(§3.1).
2. **JPEG → ROS1 bag 브릿지** — 추출 동기 세트를 `cam{0..3}/<ts_ns>.png` 폴더로 정리 →
   `kalibr_bagcreater`로 bag 생성(+ ~4Hz 다운샘플). `bag_extract` 출력 재사용.
3. **현장 커버리지 체크 스크립트** — 검출률 + 커버리지 히트맵, `bag_extract` 출력 재사용.
4. **Kalibr 실행 절차** — DS/eUCM 2회 실행(§3.4) + 리포트 비교 워크플로우.
5. **검증 스크립트/절차** — cross-projection·루프 일관성·스티칭 육안 확인.
6. **`calib.yaml` 스키마** — 모델·파라미터·extrinsic·검증 결과 기록.
7. **운영 가이드 문서** — 촬영 방법(무엇을 어떻게 흔들며 녹화) + 캘리브 실행 순서(녹화→커버리지
   체크→브릿지→Kalibr→검증)를 단계별 how-to로 정리(`docs/` 하위). 구현 후 산출.

(재사용: `capture` 파이프라인 녹화, `bag_extract` 동기 세트 추출.)

---

## 9. 미해결/추후

- 실시간 검출 오버레이(옵션 B) — 현장 체크(A)로 부족하면 monitor 노드 확장으로 추가.
- 독립 검증 도구(§6.2 B) — 루프 일관성·cross-projection. Kalibr 리포트(A)로 부족하면 추가.
- cam↔LiDAR extrinsic(C) — L2 도착 후 별도 설계. targetless/target 기반 방식 선택 포함.
- 컨테이너 내 catkin workspace 경로 — clone한 Dockerfile에서 실제 값 확인(§3.1).
