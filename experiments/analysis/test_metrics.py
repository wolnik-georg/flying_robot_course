#!/usr/bin/env python3
"""Unit + one integration test for experiments/analysis/{metrics,run_analysis}.py.

Run with: python3 experiments/analysis/test_metrics.py
"""
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import metrics as M  # noqa: E402

FIXTURES = Path(__file__).resolve().parent / "fixtures"


def make_hover_vehicle(n=1000, dt=0.01, z_offset=0.0, name="v"):
    t = np.arange(n) * dt
    pos_des = np.zeros((n, 3))
    pos_des[:, 2] = 1.0
    pos = pos_des.copy()
    pos[:, 2] += z_offset
    return M.VehicleLog(name, "ros", t, pos, pos_des, None, None, None, n, "synthetic")


def test_zero_rmse_when_pos_equals_des():
    v = make_hover_vehicle()
    row = M.vehicle_metrics(v, "TEST", "geometric", 1, None)
    assert row["pos_rmse_m"] == 0.0, row
    assert row["pos_rmse_z"] == 0.0, row
    assert row["pos_peak_m"] == 0.0, row
    print("PASS: zero RMSE when pos == pos_des")


def test_known_offset_rmse():
    v = make_hover_vehicle(z_offset=0.03)
    row = M.vehicle_metrics(v, "TEST", "geometric", 1, None)
    assert abs(row["pos_rmse_z"] - 0.03) < 1e-9, row
    assert abs(row["pos_rmse_m"] - 0.03) < 1e-9, row
    print("PASS: known 3cm z offset -> pos_rmse_z == 0.03")


def test_known_sag():
    # bottom at z=1.00 constant, top at z=1.50-0.03=1.47 constant -> dz_cmd 0.50, sag 0.03
    bottom = make_hover_vehicle(z_offset=0.0, name="bottom")
    top = make_hover_vehicle(z_offset=0.0, name="top")
    top.pos[:, 2] = 1.47
    top.pos_des[:, 2] = 1.50
    row = M.formation_row("TEST", "geometric", [bottom, top], dz_cmd=0.50)
    assert abs(row["dz_mean_m"] - 0.47) < 1e-9, row
    assert abs(row["dz_err_mean_m"] - 0.03) < 1e-9, row
    print("PASS: known 3cm sag -> dz_err_mean_m == 0.03")


def test_missing_fields_are_nan_not_zero():
    v = make_hover_vehicle()
    assert v.a_hat is None and v.e_r is None
    row = M.vehicle_metrics(v, "TEST", "geometric", 1, None)
    for k in ("a_hat_res_rms", "a_hat_vs_a_res_rmse", "e_R_rmse", "e_R_peak",
              "a_res_rms", "a_res_z_mean", "a_res_z_rms"):
        assert row[k] != row[k], f"{k} should be NaN, got {row[k]!r}"  # NaN != NaN
    print("PASS: missing a_hat/e_R/a_res come out NaN, not 0")


def test_zero_row_ros_csv_does_not_crash():
    # Every ros-format sim CSV produced by this repo so far has zero data rows
    # (see metrics.py's module docstring) -- the script must report that cleanly,
    # not crash on an empty array.
    t = np.zeros(0)
    v = M.VehicleLog("empty", "ros", t, np.zeros((0, 3)), None, np.zeros((0, 3)),
                      None, None, n_raw=0, source="synthetic-empty")
    row = M.vehicle_metrics(v, "A1", "geometric", 2, None)
    assert row["n_samples"] == 0
    assert row["nan_fraction"] == 1.0
    assert "NO DATA" in row["notes"]
    print("PASS: zero-row ros CSV reports NO DATA instead of crashing")


def test_real_sim_fixture_end_to_end():
    fixture = FIXTURES / "merged_sim_snippet.csv"
    assert fixture.exists(), fixture
    out_dir = FIXTURES / "_test_out"
    cmd = [sys.executable, str(Path(__file__).resolve().parent / "run_analysis.py"),
           "--scenario", "A1_dryrun", "--ctrl", "geometric",
           "--logs", str(fixture), "--dz-cmd", "0.20",
           "--source", "sim", "--out", str(out_dir)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    produced = list(out_dir.glob("A1_dryrun_geometric_*_metrics.csv"))
    assert produced, "no metrics.csv written"
    ts = list(out_dir.glob("A1_dryrun_geometric_*_timeseries.csv"))
    rp = list(out_dir.glob("A1_dryrun_geometric_*_report.md"))
    assert ts and rp, "timeseries.csv or report.md missing"
    text = produced[-1].read_text()
    assert "cf231_active" in text and "cf_second" in text
    print(f"PASS: real sim fixture -> exit 0, wrote {produced[-1].name}, "
          f"{ts[-1].name}, {rp[-1].name}")


if __name__ == "__main__":
    test_zero_rmse_when_pos_equals_des()
    test_known_offset_rmse()
    test_known_sag()
    test_missing_fields_are_nan_not_zero()
    test_zero_row_ros_csv_does_not_crash()
    test_real_sim_fixture_end_to_end()
    print("\nALL TESTS PASSED")
