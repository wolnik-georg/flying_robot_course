#!/usr/bin/env python3
"""Independent review (Topic 2): strategy of every flight read from the cf5 uSD log, with the margins of the decision.
INDI flag: ratio of the residual variance without / with the INDI correction (>1 = INDI explains the logged thrust better).
NS2 flag: coefficient of the NN prediction in the thrust model (about -1 applied, about 0 not).
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/review_topic2_strategy.py <bank> [...]"""
import json, sys
from pathlib import Path
import numpy as np
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "experiments/analysis")); sys.path.insert(0, str(REPO / "flying_drone_stack/tools"))
import ns2_applied_check as N, decode_usd_log as D

def detail(p):
    f = N.fit(str(p)); d = D.load(str(p)); air = np.asarray(d["z"]) > 0.3
    pred = np.asarray(d["rnn_pred_z"])[air]; est = np.asarray(d["omar_a_rpm_fz"])[air] - np.asarray(d["omar_a_imu_fz"])[air]
    return f, float(np.abs(pred).max()), float(np.abs(est).mean())

for bank in sys.argv[1:]:
    man = json.load(open(REPO / "experiments/logs" / bank / "training_manifest.json"))
    for fl in man["flights"]:
        f, pmax, est = detail(REPO / "experiments/logs/usd_raw" / fl["cf5"])
        s = ("S3" if f["coef_pred"] < -0.6 else "S1") if f["indi"] else ("S2" if f["coef_pred"] < -0.6 else "S0")
        print(f"{bank[12:]:8s} {fl['folder']:24s} {s}  indi={int(f['indi'])} coef_pred={f['coef_pred']:+.2f} coef_J={f['coef_J']:+.2f} r2={f['r2']:.2f} r2_nopred={f['r2_without_pred']:.2f} |pred|max={pmax:.2f} |est|mean={est:.2f}")
