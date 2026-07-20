import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from tum import odom_to_tum_line


def test_field_order_and_count():
    line = odom_to_tum_line(1.5, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    parts = line.split()
    assert len(parts) == 8
    # position(xyz) 다음 quaternion(xyzw)
    assert parts[1:4] == [f"{1.0:.6f}", f"{2.0:.6f}", f"{3.0:.6f}"]
    assert parts[4:8] == [f"{0.0:.6f}", f"{0.0:.6f}", f"{0.0:.6f}", f"{1.0:.6f}"]


def test_timestamp_seconds_precision():
    t = 1784544578.963209
    line = odom_to_tum_line(t, 0, 0, 0, 0, 0, 0, 1)
    assert line.split()[0] == f"{t:.9f}"


def test_no_trailing_newline():
    line = odom_to_tum_line(0.0, 0, 0, 0, 0, 0, 0, 1)
    assert not line.endswith("\n")
