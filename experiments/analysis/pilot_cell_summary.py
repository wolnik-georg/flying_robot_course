#!/usr/bin/env python3
"""Per-flight summary of pilot flights (A1 / A4 / A8) from a merged bank, bottom vehicle, uSD only, with the strategy verified from the logs.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/pilot_cell_summary.py <bank dir> [--out <dir>]
Metrics (window: airborne, scenario time 1 s .. end): signed mean z error, z RMS, worst negative / positive z error, xy RMS, seconds inside the NS2 gate,
loaded battery start / min, motor maximum, maximum roll / pitch, quality flags of the merge."""
import argparse, glob, json, sys
from pathlib import Path
import numpy as np, pandas as pd
REPO = Path(__file__).resolve().parents[2]; sys.path.insert(0, str(REPO / "experiments/analysis"))
import ns2_applied_check as N  # noqa: E402
ap = argparse.ArgumentParser(); ap.add_argument("bank"); ap.add_argument("--out", default=None); a = ap.parse_args()
raw = {}
for f in glob.glob(str(REPO / "experiments/logs/usd_raw/cf5_*")):
    raw[Path(f).name] = f
rows = []
for d in sorted(glob.glob(a.bank + "/A*_*")):
    nm = Path(d).name; scen = nm.split("_")[0]; meta = json.load(open(REPO / "experiments/logs" / f"{nm}.meta.json"))
    csv = glob.glob(d + "/*_merged_usd.csv")[0]; df = pd.read_csv(csv, comment="#")
    pb = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf5_")})[0]; pt = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf_second_")})[0]
    b = df[[c for c in df.columns if c.startswith(pb + ".")]].rename(columns=lambda c: c.split(".", 1)[1]); t = df[[c for c in df.columns if c.startswith(pt + ".")]].rename(columns=lambda c: c.split(".", 1)[1]); tt = df.t.values
    w = (b.z.values > 0.2) & (tt >= 1.0)
    ez = (b.z - b.ctrltarget_z).values; ex = (b.x - b.ctrltarget_x).values; ey = (b.y - b.ctrltarget_y).values
    gate = ((t.x - b.x).abs() < 0.2) & ((t.y - b.y).abs() < 0.2) & (t.z > b.z) & ((t.vx - b.vx).abs() < 1.5)
    f = N.fit(next(v for k, v in raw.items() if pb.split("_")[2] in k.split("_") and f"thesis{pb.split('_')[-1]}" in k)) if False else None
    p = meta["params"]
    rows.append(dict(folder=nm, scenario=scen, offset=p.get("offset"), dz=p.get("dz"), time=nm.split("_")[-1], z_mean_cm=100 * float(ez[w].mean()), z_rms_cm=100 * float(np.sqrt((ez[w] ** 2).mean())),
                     z_min_cm=100 * float(ez[w].min()), z_max_cm=100 * float(ez[w].max()), xy_rms_cm=100 * float(np.sqrt((ex[w] ** 2 + ey[w] ** 2).mean())), gate_s=float(gate[w].sum() / 500),
                     vbat_start=float(b.vbat[w].iloc[0]), vbat_min=float(b.vbat[w].min()), top_vbat_min=float(t.vbat[w].min()), motor_max_pct=float(100 * max(b[f"motor_m{i}"][w].max() for i in range(1, 5)) / 65535),
                     tilt_max_deg=float(max(np.abs(b.roll_deg[w]).max(), np.abs(b.pitch_deg[w]).max())), top_z_rms_cm=100 * float(np.sqrt(((t.z - t.ctrltarget_z).values[w] ** 2).mean())), end_s=float(tt[-1])))
R = pd.DataFrame(rows); out = Path(a.out or a.bank); R.to_json(out / "cell_summary.json", orient="records", indent=1)
pd.set_option("display.width", 250); pd.set_option("display.max_columns", 30)
print(R.drop(columns=["folder"]).round(2).to_string(index=False))
