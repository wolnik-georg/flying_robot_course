#!/usr/bin/env python3
"""Full per-flight and per-strategy overview table of the 2026-10-19 A8 validation (all merged flights). Writes experiments/results/v2val_2026-10-19/a8_overview.md
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/v2val_overview.py"""
import glob, json
from pathlib import Path
import numpy as np, pandas as pd
REPO = Path(__file__).resolve().parents[2]; BANK = REPO / "experiments/logs/bank_v2val_2026-10-19"; OUT = REPO / "experiments/results/v2val_2026-10-19"
S = json.load(open(BANK / "v2val_summary.json"))
FID = {"17-39-47": 1, "17-41-56": 2, "17-43-47": 3, "17-45-31": 4, "17-55-03": 5, "17-56-35": 6, "17-59-16": 7, "17-59-16x": 0, "18-00-40": 8, "18-03-26": 9, "18-04-46": 10,
       "18-07-12": 11, "18-08-34": 12, "18-18-35": 13, "18-21-50": 15, "18-23-19": 16}
rows = []
for r in S:
    f = glob.glob(f"{BANK}/{r['file']}/*_merged_usd.csv")[0]; d = pd.read_csv(f, comment="#")
    pre = sorted({c.split(".")[0] for c in d.columns if c.startswith("cf5_")})[0]
    b = d[[c for c in d.columns if c.startswith(pre + ".")]].rename(columns=lambda c: c.split(".", 1)[1]); t = d.t.values
    air = (b.z > 0.3).values & (t > 3); ez = (b.z - b.ctrltarget_z).values
    r["id"] = FID[r["file"].split("_")[-1]]; r["z_mean_cm"] = 100 * float(ez[air].mean()); r["z_abs_mean_cm"] = 100 * float(np.abs(ez[air]).mean())
    r["vbat_mean"] = float(b.vbat[air].mean()); r["vbat_max"] = float(b.vbat[air].max()); r["time"] = r["file"].split("_")[-1]
    rows.append(r)
rows.sort(key=lambda r: (r["name"], r["id"]))
L = ["# A8 validation 2026-10-19 — all merged flights (bottom vehicle; dz 0.5 m, peak 0.5 m/s, 19.5 s, 4 crossings; z error against the commanded z)", "",
     "Battery = loaded cell voltage logged on the bottom drone while airborne (resting voltage was not recorded). Dip = worst z error in ±0.75 s around a crossing relative to the steady offset. z mean = signed mean z error while airborne (t > 3 s).", "",
     "| strat | flight | time | note | dips at the 4 crossings [cm] | mean dip | worst | z mean [cm] | z abs mean | z RMS | vbat start | mean | min [V] | motor max | tilt max |", "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
for r in rows:
    note = "draining battery (3 back-to-back)" if r["id"] in (5, 6) else ("fresh battery per flight" if r["id"] in (13, 15, 16) else "")
    L.append(f"| {r['name']} | {r['id']} | {r['time']} | {note} | {', '.join(f'{x:.1f}' for x in r['dip_cm'])} | {r['dip_mean_cm']:.1f} | {min(r['dip_cm']):.1f} | {r['z_mean_cm']:+.1f} | {r['z_abs_mean_cm']:.1f} | {r['z_rms_cm']:.2f} | {r['vbat_start']:.2f} | {r['vbat_mean']:.2f} | {r['vbat_min']:.2f} | {r['motor_max_pct']:.0f} % | {r['tilt_max_deg']:.0f}° |")
L += ["", "## Per strategy", "| strat | flights | mean dip [cm] | per-flight mean dips | worst crossing | z mean [cm] | z RMS [cm] | vbat mean [V] | vbat min [V] | motor max |", "|---|---|---|---|---|---|---|---|---|---|"]
def agg(name, sel, label):
    rr = [r for r in rows if r["name"] == name and sel(r)]
    if not rr: return
    dips = np.array([r["dip_cm"] for r in rr])
    L.append(f"| {label} | {', '.join(str(r['id']) for r in rr)} | **{dips.mean():.1f}** | {', '.join(f'{r['dip_mean_cm']:.1f}' for r in rr)} | {dips.min():.1f} | {np.mean([r['z_mean_cm'] for r in rr]):+.1f} | {np.mean([r['z_rms_cm'] for r in rr]):.2f} | {np.mean([r['vbat_mean'] for r in rr]):.2f} | {min(r['vbat_min'] for r in rr):.2f} | {min(r['motor_max_pct'] for r in rr):.0f}–{max(r['motor_max_pct'] for r in rr):.0f} % |")
for n in ("S0", "S1", "S2"): agg(n, lambda r: True, n)
agg("S3", lambda r: True, "S3 all 6 uSD flights"); agg("S3", lambda r: r["id"] in (4, 13, 15, 16), "S3 without the 2 draining-battery flights (4, 13, 15, 16)"); agg("S3", lambda r: r["id"] in (13, 15, 16), "S3 fresh battery only (13, 15, 16)")
(OUT / "a8_overview.md").write_text("\n".join(L) + "\n"); print("\n".join(L))
