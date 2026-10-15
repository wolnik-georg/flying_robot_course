#!/usr/bin/env python3
"""Compact, committable extract of the merged tree-validation flights (100 Hz, plot-relevant channels) + per-flight metrics.
  python3 experiments/analysis/tree_validation_export.py   -> experiments/results/tree_validation_2026-10-15/
Source (not committed, 217 MB+): experiments/logs/merged_tree_2026-10-15/merged_*.csv (rebuild: tree_validation_merge.py)."""
import glob, json, re, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from tree_validation_analysis import load, metrics
SRC, OUT = "experiments/logs/merged_tree_2026-10-15", Path("experiments/results/tree_validation_2026-10-15")
CH = ["x","y","z","vx","vy","vz","ctrltarget_x","ctrltarget_y","ctrltarget_z","roll_deg","pitch_deg","rnn_pred_z","omar_a_rpm_fz","omar_a_imu_fz","a_res_z","vbat"]
OUT.mkdir(parents=True, exist_ok=True); summ = {}
for f in sorted(glob.glob(f"{SRC}/merged_*.csv")):
    name = Path(f).stem.replace("merged_", ""); c, b, t = load(f); T = c["t"]; idx = np.arange(0, len(T), 5)   # 500 -> 100 Hz
    cols = ["t"] + [f"bottom_{k}" for k in CH] + [f"top_{k}" for k in CH[:11]]
    data = [T[idx]] + [c.get(f"{b}.{k}", np.full(len(T), np.nan))[idx] for k in CH] + [c.get(f"{t}.{k}", np.full(len(T), np.nan))[idx] for k in CH[:11]]
    np.savetxt(OUT / f"{name}.csv", np.column_stack(data), delimiter=",", header=",".join(cols), comments="", fmt="%.5g")
    summ[name] = metrics(f)
json.dump(summ, open(OUT / "metrics.json", "w"), indent=1)
(OUT / "README.md").write_text("""# Tree validation 2026-10-15 — plot-ready extract
One CSV per flight (`tree_<S0..S3>_<A8|A1>_<hhmm>.csv`), 100 Hz, time `t` = seconds on the commanded-trajectory alignment, `bottom_*` = cf5 (carries the strategy), `top_*` = cf_second (always S0). Channels: position/velocity, commanded target, roll/pitch (deg), NS2 prediction z (m/s^2), Omar a_rpm_f / a_imu_f z, measured residual a_res_z, battery.
`metrics.json`: per-flight numbers (z error mean/std, xy rms, attitude max, separation error, A8 crossing dip / upward peak / offset). Definitions: `experiments/analysis/tree_validation_analysis.py`.
Flights used: outliers (battery / logging cut-offs) already excluded, see docs/lab_sessions/2026-10-15_tree_validation_coverage.md. Raw uSD logs: experiments/logs/usd_raw/*tree_*.
""")
print(len(summ), "flights ->", OUT)
