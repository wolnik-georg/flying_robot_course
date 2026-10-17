#!/usr/bin/env python3
"""Per-section results of the merged A9 flights (the nine A4 offset steps in one flight, scenario A9 in crazyswarm2) and the flight-level safety checks.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/ns2_a9_sections.py --bank experiments/logs/bank_ns2train_2026-10-17 --out experiments/results/ns2_collection_2026-10-17
Sections follow A9_SEQUENCE (dz 0.75/0.5/0.25 x offset 0/0.1/0.15, 2 passes of 1.2 m, 5 s transitions)."""
import argparse, json
from pathlib import Path
import numpy as np, pandas as pd
ROOT = Path(__file__).resolve().parents[2]
SEQ = ((0.75, 0.0), (0.75, 0.1), (0.75, 0.15), (0.5, 0.15), (0.5, 0.1), (0.5, 0.0), (0.25, 0.0), (0.25, 0.1), (0.25, 0.15))

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--bank", required=True); ap.add_argument("--out", required=True); a = ap.parse_args()
    ms = json.load(open(ROOT / a.bank / "training_manifest.json"))["flights"]; out = ROOT / a.out; out.mkdir(parents=True, exist_ok=True)
    flights, secs = [], []
    for f in ms:
        if f["scenario"] != "A9": continue
        m = json.load(open(ROOT / f"experiments/logs/A9_{f['date']}_{f['stamp']}.meta.json")); du = m["duration"]; sp = m["params"]["speed"]
        hdr = pd.read_csv(ROOT / f["path"], comment="#", nrows=0).columns
        B = [c for c in hdr if c.startswith("cf5_")][0].split(".")[0] + "."; T = B.replace("cf5_", "cf_second_")
        d = pd.read_csv(ROOT / f["path"], comment="#"); w = d[(d.t >= 0) & (d.t <= du)]; t = w.t.values; dt = float(np.median(np.diff(t))); g = lambda p, c: w[p + c].values
        dx, dy, dz = g(T, "locsrv_x") - g(B, "locsrv_x"), g(T, "locsrv_y") - g(B, "locsrv_y"), g(T, "locsrv_z") - g(B, "locsrv_z")
        h = np.hypot(dx, dy); r = np.sqrt(h ** 2 + dz ** 2)
        k = max(1, int(0.1 / dt)); sm = lambda x: np.convolve(x, np.ones(k) / k, "same")
        gate = (abs(dx) < .2) & (abs(dy) < .2) & (abs(np.gradient(sm(g(T, "locsrv_x")), t) - np.gradient(sm(g(B, "locsrv_x")), t)) < 1.5) & (dz > 0)
        ez, ar, bz = 100 * g(B, "omar_err_pos_z"), g(B, "a_res_z"), g(B, "locsrv_z")
        flights.append(dict(stamp=f["stamp"], speed=sp, plan_s=round(du, 1), min_3d_m=round(float(r.min()), 3), t_min_3d_s=round(float(t[r.argmin()]), 1),
                            bottom_above_top_s=round(dt * float((dz < 0).sum()), 1), gap_lt_0p2_in_lateral_gate_s=round(dt * float(((dz < .2) & (h < .2)).sum()), 1),
                            z_err_rms_cm=round(float(np.sqrt((ez ** 2).mean())), 1), gate_s=round(float(gate.sum() * dt), 1), a_res_z_in_gate=round(float(ar[gate].mean()), 2),
                            a_res_z_min=round(float(ar.min()), 2), a_res_z_max=round(float(ar.max()), 2)))
        leg = 1.2 * 35 / 16 / sp; sec = 2 * leg
        for i, (dzn, off) in enumerate(SEQ):
            t0 = i * (sec + 5.0); mk = (t >= t0) & (t <= t0 + sec); gk = gate & mk
            secs.append(dict(stamp=f["stamp"], speed=sp, section=i + 1, dz_nominal=dzn, offset=off, gate_s=round(float(gk.sum() * dt), 1),
                             real_gap_median=round(float(np.median(dz[mk])), 2), bottom_z_median=round(float(np.median(bz[mk])), 2),
                             z_err_rms_cm=round(float(np.sqrt((ez[mk] ** 2).mean())), 1), a_res_z_in_gate=round(float(ar[gk].mean()), 2) if gk.any() else None, min_3d_m=round(float(r[mk].min()), 2)))
    pd.DataFrame(flights).to_csv(out / "a9_flights.csv", index=False); S = pd.DataFrame(secs); S.to_csv(out / "a9_sections.csv", index=False)
    print(pd.DataFrame(flights).to_string(index=False)); print()
    print("a_res_z in the gate by lateral offset (all A9 flights, all dz):"); print(S.groupby("offset").a_res_z_in_gate.agg(["mean", "min", "max", "count"]).round(2).to_string())
    print("by nominal dz:"); print(S.groupby("dz_nominal").agg(real_gap=("real_gap_median", "mean"), a_res=("a_res_z_in_gate", "mean"), ez=("z_err_rms_cm", "mean")).round(2).to_string())
if __name__ == "__main__": main()
