"""gather_annotations 순수 로직 테스트(파일 IO 는 tmp_path). 하드웨어·dataset 불필요."""
import json
import os
import sys

BA = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BA)

from bev_label import BevSpec                         # noqa: E402
import gather_annotations as ga                        # noqa: E402


def test_spec_from_meta_reads_recorded_range(tmp_path):
    """dataset 이 어떤 범위로 만들어졌는지는 meta.json 에 있다 — 기본값을 쓰면 review 의
    미터축·ego 박스가 엉뚱한 자리에 그려진다."""
    (tmp_path / "meta.json").write_text(json.dumps(
        {"bev": {"XF": 3.5, "XR": 1.5, "YH": 2.5, "RES": 0.05}}))
    spec = ga.spec_from_meta(tmp_path)
    assert (spec.XF, spec.XR, spec.YH, spec.RES) == (3.5, 1.5, 2.5, 0.05)
    assert (spec.NX, spec.NY) == (100, 100)


def test_spec_from_meta_falls_back_to_default(tmp_path):
    """meta.json 없는 예전 dataset 도 그대로 동작해야 한다(기본 범위 가정)."""
    assert ga.spec_from_meta(tmp_path) == BevSpec()
