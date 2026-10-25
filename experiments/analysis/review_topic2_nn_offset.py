#!/usr/bin/env python3
"""Independent review (Topic 2): NN mean level vs the INDI label, in and out of the gate, seen (pilot 10-19/20) vs unseen (10-23) flights, NN not applied.
Label = (a_imu_raw - a_rpm_raw)_z from the nn_vs_indi cache, zero-delay Butterworth 2nd order 5 Hz (as nn_vs_indi_check.py). Gate as nn_vs_indi_check.py,
evaluated directly on the bin time base from the logged own position and the neighbour's position in the merged CSV (time-aligned via bottom z).
Out of gate = airborne, steady (t > 3 s after takeoff), neighbour |dx|,|dy| > 0.4 m or neighbour not above: should be ~0 for an unbiased rotor model.
Also: onboard-logged estimate (estlog image) vs offline rebuild, all four strategies.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/review_topic2_nn_offset.py <scratch out dir>"""
import json, glob, sys
from pathlib import Path
import numpy as np, pandas as pd
from scipy.signal import butter, filtfilt
REPO = Path(__file__).resolve().parents[2]; CACHE = REPO / "experiments/results/nn_vs_indi_2026-10-22/cache"
man = {}
for b in glob.glob(str(REPO / "experiments/logs/bank_*2026-10-19")) + glob.glob(str(REPO / "experiments/logs/bank_*2026-10-2[03]")):
    for fl in json.load(open(Path(b) / "training_manifest.json"))["flights"]: man[fl["folder"]] = REPO / fl["path"]
def lp(x, fc=5.0, fs=500.0):
    b, a = butter(2, fc / (fs / 2)); return filtfilt(b, a, x, axis=0)
def neighbour(name, tb, zb):
    df = pd.read_csv(man[name], comment="#"); pb = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf5_")})[0]; pt = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf_second_")})[0]
    g = lambda p, k: df[f"{p}.{k}"].values; tm = df.t.values
    grid = np.arange(0, tm[-1], 0.01); a = np.interp(grid, tm, g(pb, "z")); a = a - a.mean(); tg = np.arange(tb[0], tb[-1], 0.01); bb = np.interp(tg, tb, zb); bb = bb - bb.mean(); n = len(a); best = (-2, 0)
    for s in range(0, len(bb) - n // 2):
        m = min(n, len(bb) - s)
        c = (a[:m] * bb[s:s + m]).sum() / (np.sqrt((a[:m] ** 2).sum() * (bb[s:s + m] ** 2).sum()) + 1e-12)
        if c > best[0]: best = (c, s)
    t0 = tg[best[1]]; q = lambda k: np.interp(tb - t0, tm, g(pt, k) - g(pb, k))
    return q("x"), q("y"), q("z"), q("vx")
rows = []
for f in sorted(CACHE.glob("*.npz")):
    z = np.load(f, allow_pickle=True); name = str(z["folder"])
    if name not in man: continue
    day = name.split("_")[1]; seen = day != "2026-10-23"; t = z["t"]; air = z["z"] > 0.2
    if air.sum() < 1500: continue
    i0 = np.argmax(air); w = air & (t > t[i0] + 1.0) & (t < t[np.where(air)[0][-1]] - 0.5)
    L = lp(z["a_imu_raw"][:, 2] - z["a_rpm_raw"][:, 2]); P = lp(z["pred"][:, 2]); dx, dy, dz, dvx = neighbour(name, t, z["z"])
    gate = (np.abs(dx) < 0.2) & (np.abs(dy) < 0.2) & (dz > 0) & (np.abs(dvx) < 1.5); far = ((np.abs(dx) > 0.4) | (np.abs(dy) > 0.4) | (dz < 0)) & (t > t[i0] + 3.0)
    lg = (z["logged_imu_f"] - z["logged_rpm_f"])[:, 2]; rb = (z["a_imu_f"] - z["a_rpm_f"])[:, 2]
    applied = float(z["coef_pred"]) < -0.6; strat = ("S3" if applied else "S1") if bool(z["indi"]) else ("S2" if applied else "S0")
    r = dict(folder=name, seen=seen, scenario=str(z["scenario"]), strat=strat, n_gate=int((w & gate).sum()), n_far=int((w & far).sum()),
             log_vs_rebuild_corr=float(np.corrcoef(lg[w], rb[w])[0, 1]) if np.abs(lg[w]).mean() > 1e-6 else np.nan, log_vs_rebuild_rms=float(np.sqrt(((lg - rb)[w] ** 2).mean())))
    for tag, sel in (("gate", w & gate), ("far", w & far)):
        if sel.sum() > 200: r.update({f"nn_{tag}": float(P[sel].mean()), f"lab_{tag}": float(L[sel].mean())})
    rows.append(r)
R = pd.DataFrame(rows); out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("."); out.mkdir(parents=True, exist_ok=True); R.to_csv(out / "nn_offset_per_flight.csv", index=False)
N = R[R.strat.isin(["S0", "S1"])].copy(); N["off_gate"] = N.nn_gate - N.lab_gate; N["lab_gate_corr"] = N.lab_gate - N.lab_far; N["off_gate_corr"] = N.nn_gate - N.lab_gate_corr
pd.set_option("display.width", 220)
print(N.groupby(["scenario", "strat", "seen"]).agg(n=("folder", "size"), nn_gate=("nn_gate", "median"), lab_gate=("lab_gate", "median"), lab_far=("lab_far", "median"), nn_far=("nn_far", "median"),
      offset=("off_gate", "median"), offset_after_far_bias=("off_gate_corr", "median"), ratio=("nn_gate", lambda s: np.nan)).drop(columns="ratio").round(2).to_string())
N["ratio"] = N.nn_gate / N.lab_gate; N["ratio_corr"] = N.nn_gate / N.lab_gate_corr
print("\nNN / label mean ratio in gate (median over flights):"); print(N.groupby(["seen", "strat"]).agg(ratio=("ratio", "median"), ratio_after_far_bias=("ratio_corr", "median"), n=("folder", "size")).round(2).to_string())
U = R[~R.seen]; print("\nestlog image: onboard log vs offline rebuild per strategy (unseen flights):"); print(U.groupby("strat").agg(n=("folder", "size"), corr_min=("log_vs_rebuild_corr", "min"), corr_med=("log_vs_rebuild_corr", "median"), rms_max=("log_vs_rebuild_rms", "max")).round(3).to_string())
