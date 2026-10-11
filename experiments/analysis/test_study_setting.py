#!/usr/bin/env python3
"""Synthetic tests for study_setting.extract_setting."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from study_setting import extract_setting  # noqa: E402


def test_a8_speed_grid():
    meta = {
        "scenario": "A8",
        "params": {"dz": 0.5},
        "realised": {"peak_speed": 0.3646},
        "height": 0.5,
    }
    s = extract_setting(meta)
    assert s["speed_mps_nominal"] == 0.4
    assert not s["speed_off_grid"]


def test_a1_dz_off_grid():
    meta = {"scenario": "A1", "params": {"dz": 0.30}}
    s = extract_setting(meta)
    assert s["dz_m_nominal"] == 0.25
    assert s["dz_off_grid"]


if __name__ == "__main__":
    test_a8_speed_grid()
    test_a1_dz_off_grid()
    print("PASS study_setting tests")
