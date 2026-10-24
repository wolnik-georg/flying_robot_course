#!/usr/bin/env python3
"""FBL (S4) flights compared with S0-S3 of the unseen-settings block (2026-10-23): same metrics (A8 crossing dip, A1 mean z error 2-15 s, A4 z RMS) and the FBL health channels
(ctrlFbl.active / tripped / faults / steps, tilt, thrust, motor rpm). The strategy of an FBL flight is read from ctrlFbl.steps (the Omar thrust model does not describe FBL).
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/fbl_unseen_compare.py --banks bank_fbl2_2026-10-24 --out experiments/results/fbl_unseen_2026-10-24"""
import argparse, json, sys
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "experiments/analysis")); sys.path.insert(0, str(REPO / "flying_drone_stack/tools"))
import decode_usd_log as D                                       # noqa: E402
from unseen_block_analysis import flight_row                     # noqa: E402

ap = argparse.ArgumentParser(); ap.add_argument("--banks", nargs="+", required=True); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--ref", type=Path, default=REPO / "experiments/results/unseen_block_2026-10-23"); a = ap.parse_args()
out = a.out if a.out.is_absolute() else REPO / a.out; out.mkdir(parents=True, exist_ok=True); rows = []; diag = []
for bank in a.banks:
    for fl in json.load(open(REPO / "experiments/logs" / bank / "training_manifest.json"))["flights"]:
        if fl["scenario"] not in ("A8", "A1", "A4"): continue
        meta = json.load(open(REPO / "experiments/logs" / f"{fl['scenario']}_{fl['date']}_{fl['stamp']}.meta.json")); meta["scenario"] = fl["scenario"]
        d = D.load(str(REPO / "experiments/logs/usd_raw" / fl["cf5"])); t = np.asarray(d["t"], float); t = (t - t[0]) / (1e6 if t[-1] - t[0] > 1e5 else 1)
        air = np.asarray(d["z"]) > 0.3; st = np.asarray(d["fbl_steps"]); ran = int(st[air][-1] - st[air][0]) > 0 if air.any() else False
        strat = "S4" if ran else "S0?"
        r = flight_row(REPO / fl["path"], meta, strat, bank); rows.append(r)
        tr = np.asarray(d["fbl_tripped"]); tilt = np.maximum(np.abs(d["roll_deg"]), np.abs(d["pitch_deg"]))
        k = int(np.argmax(tr > 0)) if (tr > 0).any() else None
        rpm = np.max([np.asarray(d[f"rpm_m{i}"]) for i in (1, 2, 3, 4)], 0)
        diag.append(dict(folder=fl["folder"], cell=r["cell"], fbl_active_pct=100 * float(np.mean(np.asarray(d["fbl_active"])[air])), tripped=int(tr.max()), faults=int(np.asarray(d["fbl_faults"]).max()),
                         trip_t_s=float(t[k]) if k is not None else None, trip_z=float(np.asarray(d["z"])[k]) if k is not None else None, tilt_max_deg=float(tilt[air].max()), tilt_at_trip=float(tilt[k]) if k is not None else None,
                         thrust_mean_N=float(np.asarray(d["fbl_thrust"])[air].mean()), rpm_max=float(rpm[air].max()), vbat0=r["vbat_start"]))
R = pd.DataFrame(rows); G = pd.DataFrame(diag); R.to_csv(out / "per_flight_fbl.csv", index=False); G.to_csv(out / "fbl_health.csv", index=False)
pd.set_option("display.width", 220); print(G.round(2).to_string(index=False))
C = pd.read_csv(a.ref / "cells.csv"); F = R.groupby("cell").agg(n=("metric", "size"), mean=("metric", "mean")).reset_index()
tab = []
for cell, g in C.groupby("cell"):
    row = {"cell": cell}; row.update({s: g[g.strategy == s]["mean"].iloc[0] if (g.strategy == s).any() else np.nan for s in ("S0", "S1", "S2", "S3")})
    f = F[F.cell == cell]; row["S4_FBL"] = f["mean"].iloc[0] if len(f) else np.nan; row["n_S4"] = int(f.n.iloc[0]) if len(f) else 0; tab.append(row)
T = pd.DataFrame(tab); T.to_csv(out / "comparison.csv", index=False); print(); print(T.round(1).to_string(index=False))
fig, axs = plt.subplots(1, len(T), figsize=(3.6 * len(T), 3.6), squeeze=False); cols = ["#7f7f7f", "#1f77b4", "#2ca02c", "#d62728", "#9467bd"]
for ax, (_, r) in zip(axs[0], T.iterrows()):
    v = [abs(r[s]) for s in ("S0", "S1", "S2", "S3", "S4_FBL")]; ax.bar(range(5), v, color=cols, alpha=0.85)
    ax.set_xticks(range(5)); ax.set_xticklabels(["S0", "S1", "S2", "S3", "S4\nFBL"]); ax.set_title(r["cell"], fontsize=8); ax.set_ylabel("|metric| [cm]")
fig.suptitle("FBL (S4) against S0-S3 at the unseen settings (bottom drone, uSD logs)", fontsize=9); fig.tight_layout(); fig.savefig(out / "fbl_vs_s0s3.png", dpi=130)
