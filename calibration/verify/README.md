# calib.yaml 시각 검증 도구 (어안 DS 4대)

`calib.yaml`(Double Sphere) 이 실제로 잘 맞는지 **눈으로** 확인하는 도구 모음.
Kalibr 리포트의 수치(재투영 오차)를 넘어, 실제 장면이 얼마나 잘 펴지고 카메라끼리
얼마나 잘 이어지는지를 본다. **검출 불필요**(DS 투영/역투영만 사용)라 호스트 파이썬만으로
돌아가고, calib 이미지든 직접 수집한 일반 이미지든 동일하게 적용된다.

의존성: `numpy opencv-python pyyaml` (이미 설치됨). ROS·Docker 불필요.

## 구성

| 파일 | 역할 |
|---|---|
| `ds_model.py` | DS 투영(3D→픽셀)·역투영(픽셀→방향) + `calib.yaml`/`orientation.json` 로더 |
| `rectify.py` | 핀홀·원통 ray 생성과 리매핑 |
| `common.py` | 프레임 로딩·선택·라벨·회전 렌더 등 공용 |
| `verify_undistort.py` | **언디스토션**: RAW \| 핀홀 \| 원통 나란히 (카메라별) |
| `verify_panorama.py` | **360° 파노라마 스티칭**: extrinsic 으로 4대 합성 |
| `verify_extrinsics.py` | **카메라 간 겹침**: 인접쌍 checkerboard/blend |
| `test_verify.py` | 순수 로직 pytest |

입력 이미지 레이아웃은 `bag_extract.py` 산출물과 동일: `<루트>/frame_XXXXXX/cam{0..3}.jpg`.

## 실행 (저장소 루트에서)

```bash
# 1) 언디스토션 — 세상 직선이 곧게 펴지는지 (calib 세션 이미지)
python3 calibration/verify/verify_undistort.py \
    --images data/calib_260723/extracted --frames "0,800,1600,2400"

# 2) 360° 파노라마 — intrinsic+extrinsic 동시 검증
python3 calibration/verify/verify_panorama.py \
    --images data/calib_260723/extracted --frames "800,1600"

# 3) 카메라 간 겹침 — extrinsic 검증
python3 calibration/verify/verify_extrinsics.py \
    --images data/calib_260723/extracted --frames "800,1600"

# 4) 최종: 직접 수집한 이미지에 적용 (--out 로 분리 저장)
python3 calibration/verify/verify_undistort.py \
    --images data/cam_out/raws1_out-images --frames "0,250" \
    --out data/cam_out/verify/undistort
python3 calibration/verify/verify_panorama.py \
    --images data/cam_out/raws1_out-images --frames "250" \
    --out data/cam_out/verify/panorama
```

공통 인자: `--calib`(기본 `data/calib_260723/calib.yaml`), `--orientation`,
`--images`(필수), `--frames`(`"0,10,20"` 또는 `"0-499:50"`; 생략 시 등간격 6장), `--out`.
개별 인자: undistort `--pin-fov/--cyl-fov`, panorama `--width/--height/--vfov`,
extrinsics `--fov/--width/--height`.

## 무엇을 봐야 하나 (판정 포인트)

**언디스토션 (`verify_undistort`)** — 가장 직관적.
- **핀홀**: 세상에서 곧은 것(보드 격자 행/열, 창틀, 기둥 모서리, 천장선)이 **곧은 직선**으로
  나오면 intrinsic 양호. 중앙이 휘거나 격자가 배럴/핀쿠션으로 굽으면 intrinsic 문제.
- **원통**: 넓은 화각에서 **수직선이 수직**, 수평선/지평선이 매끄러운지. 어안 주변부까지
  왜곡이 잘 제거됐는지 본다.
- 보드 프레임이면 격자가 얼마나 규칙적인 직선망이 되는지가 핵심.

**360° 파노라마 (`verify_panorama`)** — intrinsic+extrinsic 동시.
- `*_pano_blend.png`: **먼 구조물**(벽·창·천장선·복도)이 카메라 경계를 넘어 **끊김 없이 이어지는지**.
  잘 이어지면 상대자세(extrinsic)와 각 카메라 펴짐(intrinsic)이 모두 양호.
- `*_pano_seam.png`: 노란 선 = 카메라 경계. **경계에서 선/모서리가 어긋나지 않는지**.
- ⚠️ **가까운 물체의 이음새 어긋남/유령현상은 정상**: 파노라마는 병진을 무시한 회전-전용
  근사라 시차(parallax)로 가까운 것은 어긋난다. **멀리 있는 구조물의 정렬**로 판정한다.

**카메라 간 겹침 (`verify_extrinsics`)** — extrinsic 집중.
- `checkerboard`(위): 두 카메라를 바둑판으로 교차. **타일 경계에서 선/무늬가 매끄럽게
  이어지면** 상대자세 양호. 경계에서 뚝뚝 끊기면 extrinsic 오차(단, 가까운 물체는 시차).
- `50/50 blend`(아래): 겹침 영역 반투명 합성. **유령현상(겹쳐 보임)이 적을수록** 양호.
- 판정엔 **멀리 있는 배경**이 잘 보이는 프레임이 유리(가까운 보드만 있으면 시차가 지배).

**직접 수집 이미지**: 타깃이 없어도 언디스토션·파노라마는 그대로 유효. 실제 사용할 장면에서
직선이 곧게 펴지고 4대가 잘 이어지면, 그 calib 을 다운스트림(BEV·학습)에 써도 좋다는 실증.

## 한계 / 참고
- **재투영 코너 오버레이(정량)는 없음**: 이 촘촘한 AprilGrid 는 `cv2.aruco`·`pupil-apriltags`
  로 검출이 안 된다(`docs/CALIBRATION.md §7`, `kalibr_detect_report.py` 참조 — 알려진 한계).
  정량 재투영 오차는 Kalibr 리포트(`calib-report-cam-ds.pdf`, `calib-results-cam-ds.txt`)를 본다.
- 파노라마/겹침은 회전-전용 근사(병진 무시). 가까운 물체 시차는 오차가 아니다.

## 테스트
```bash
cd calibration/verify && python3 -m pytest test_verify.py -q
```
