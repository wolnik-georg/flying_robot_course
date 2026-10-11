#!/usr/bin/env python3
"""Desk tests (synthetic data) for study_metrics helpers, the verified attitude convention and aggregate.study_table."""
import sys
from pathlib import Path
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
import study_metrics as sm  # noqa: E402
import aggregate as ag  # noqa: E402


def main():
    # reference attitude: thrust towards +y needs NEGATIVE roll, towards +x POSITIVE pitch (standard convention);
    # verified 2026-10-11 on real A2/A3/A4 flights (logged roll/pitch correlate -0.5..-0.87 with atan(a/g): roll_std = roll_log, pitch_std = -pitch_log)
    t = np.arange(0, 10, 0.01)
    d = {"ctrltarget_x": 0.5 * 0.4 * t ** 2, "ctrltarget_y": 0.5 * 1.0 * t ** 2}   # a_x = 0.4, a_y = 1.0 (constant)
    rr, pr, src = sm.nominal_ref_tilt_std_deg(d, t)
    assert src == "d2(ctrltarget pos)"
    mid = slice(200, 800)
    assert abs(np.mean(rr[mid]) + np.degrees(np.arctan2(1.0, 9.81))) < 0.2, np.mean(rr[mid])
    assert abs(np.mean(pr[mid]) - np.degrees(np.arctan2(0.4, 9.81))) < 0.2, np.mean(pr[mid])
    d2 = {"ctrltarget_ax": np.full_like(t, 0.4), "ctrltarget_ay": np.full_like(t, 1.0)}
    rr2, pr2, src2 = sm.nominal_ref_tilt_std_deg(d2, t)
    assert src2 == "ctrltarget.a*" and rr2[0] < 0 < pr2[0]
    # measured angles: stabilizer pitch is inverted w.r.t. the standard convention
    mr, mp, ms = sm.measured_att_std_deg({"roll_deg": np.array([1.0]), "pitch_deg": np.array([2.0])})
    assert (mr[0], mp[0], ms) == (1.0, -2.0, "stabilizer.roll/pitch")
    # quaternion path: 10 deg roll about x
    th = np.radians(np.array([10.0, 20.0]))
    mr, mp, ms = sm.measured_att_std_deg({"est_qw": np.cos(th / 2), "est_qx": np.sin(th / 2), "est_qy": np.zeros(2), "est_qz": np.zeros(2)})
    assert ms == "quaternion" and np.allclose(mr, [10.0, 20.0], atol=1e-6) and np.allclose(mp, 0, atol=1e-6)
    # an all-zero quaternion channel (old config / unflown controller) must NOT be used
    _, _, ms0 = sm.measured_att_std_deg({"est_qw": np.zeros(3), "est_qx": np.zeros(3), "est_qy": np.zeros(3), "est_qz": np.zeros(3), "roll_deg": np.zeros(3), "pitch_deg": np.zeros(3)})
    assert ms0 == "stabilizer.roll/pitch"
    # pass dips: two passes with a dip each, horizontal distance minimum at the pass
    g = np.arange(0, 20, 0.02)
    horiz = np.minimum(np.abs(g - 6), np.abs(g - 14)) * 0.5
    ez = np.zeros_like(g)
    ez[np.abs(g - 6) < 0.5] = -5.0; ez[np.abs(g - 14) < 0.5] = -3.0
    dips = sm.pass_dips_cm(g, ez, horiz)
    assert dips == [-5.0, -3.0], dips
    # aggregate.study_table: unit = flight, clean only, not_reached needs 8 attempts, unpaired bootstrap vs baseline
    rows = []
    for i in range(5):   # S0, 5 clean
        rows.append(dict(scenario="A8", setting_key="speed=0.4", strategy="S0", vehicle_id="cf5", date="d", stamp=f"s{i}", decision="clean", primary_metric=-6.0 + 0.1 * i))
        rows.append(dict(scenario="A8", setting_key="speed=0.4", strategy="S0", vehicle_id="cf_second", date="d", stamp=f"s{i}", decision="clean", primary_metric=99.0))
    for i in range(8):   # S2: 8 attempts, 3 clean -> not reached
        rows.append(dict(scenario="A8", setting_key="speed=0.4", strategy="S2", vehicle_id="cf5", date="d", stamp=f"t{i}", decision="clean" if i < 3 else "excluded", primary_metric=-3.0))
    cells, vs, mvs = ag.study_table(pd.DataFrame(rows), "primary_metric", "S0")
    c0 = cells[cells.strategy == "S0"].iloc[0]; c2 = cells[cells.strategy == "S2"].iloc[0]
    assert c0.n_clean == 5 and not c0.incomplete and abs(c0["mean"] - (-5.8)) < 1e-9 and c0["std"] > 0   # cf_second rows ignored
    assert c2.n_clean == 3 and c2.n_attempts == 8 and c2.incomplete and c2.not_reached
    v = vs.iloc[0]
    assert v.strategy == "S2" and abs(v.mean_diff - 2.8) < 1e-9 and v.note
    print("PASS study_metrics / aggregate tests")


if __name__ == "__main__":
    main()
