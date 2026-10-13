#!/usr/bin/env python3
"""Direct on-drone check of the RPM filter from ONE uSD flight log recorded with the usd96+rpmf firmware (config has rpmf.m1..m4).
Top: raw DShot reading (motor.mN_rpm). Bottom: the value the controller used (rpmf.mN, output of the firmware filter).
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/rpm_filter_check_flight.py <log.bin> [--motor 1..4] [--out file.png]
Prints per motor: bad raw readings (> 28,000 incl. 0xFFFF), the maximum of the used value, and how many valid samples the filter changed."""
import argparse, sys
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "flying_drone_stack" / "tools"))
import decode_usd_log as dec

ap = argparse.ArgumentParser(); ap.add_argument("log"); ap.add_argument("--motor", type=int, default=0); ap.add_argument("--out", default=None); a = ap.parse_args()
r = dec.load(a.log)
if "rpmf_m1" not in r: sys.exit("this log has no rpmf.m1..m4 (needs the usd96+rpmf firmware and usd_study_config.txt)")
up = r["z"] > 0.3; t = r["t"] - r["t"][0]
for i in range(1, 5):
    raw = r[f"motor_m{i}_rpm"].astype(float); used = r[f"rpmf_m{i}"].astype(float)
    bad = up & (raw > 28000)
    print(f"motor {i}: bad raw readings {int(bad.sum())}, max raw {raw[up].max():,.0f}, max used {used[up].max():,.0f}, "
          f"valid samples changed {int((up & (raw < 28000) & (raw != used)).sum())} of {int((up & (raw < 28000)).sum())}")
m = a.motor or int(np.argmax([(up & (r[f'motor_m{i}_rpm'] > 28000)).sum() for i in range(1, 5))]) + 1
raw = r[f"motor_m{m}_rpm"].astype(float); used = r[f"rpmf_m{m}"].astype(float); deck = r[f"rpm_m{m}"].astype(float)
fig, ax = plt.subplots(2, 1, figsize=(11, 6.4), sharex=True, sharey=True)
ax[0].plot(t, deck, color="#888888", lw=0.8, label="optical deck RPM (reference)"); ax[0].plot(t, raw, color="#0072B2", lw=0.9, label="raw DShot reading (no filter)")
ax[1].plot(t, deck, color="#888888", lw=0.8, label="optical deck RPM (reference)"); ax[1].plot(t, used, color="#009E73", lw=0.9, label="RPM filter applied (used by the controller, logged on the drone)")
for x in ax: x.set_ylim(0, 70000); x.set_ylabel(f"motor {m} RPM"); x.legend(fontsize=9, loc="center right"); x.grid(alpha=0.3)
ax[1].set_xlim(t[up][0] - 0.3, t[up][-1] + 0.3); ax[1].set_xlabel("time (s)"); ax[0].set_title("Before / after the RPM filter")
fig.tight_layout(); out = a.out or str(Path(a.log).with_suffix(".rpm_filter.png")); fig.savefig(out, dpi=150); print("wrote", out)
