#!/usr/bin/env python3
"""Simple meeting figure: how far are the INDI variants from Omar C when they are given the same inputs, native vs all parameters equal.
Bars = A8 flight 2026-10-02, dots = today's A8 flight (ours on-policy). Reference: Omar C (attitude integral off). Source: summary.json of indi_replay_equal.py.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/indi_equal_simple_plot.py --out file.png"""
import json, sys
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
A = json.loads((HERE / "out/indi_replay_equal/A8/summary.json").read_text())["vs_omar_c_noKI"]
B = json.loads((HERE / "out/indi_replay_equal/2026-10-13_ours_3/summary.json").read_text())["vs_omar_c_noKI"]
RUNS = [("ours\nnative", "ours_L0_native", "#9ecae1"), ("ours, all\nparameters equal", "ours_L8_limits_off", "#0072B2"),
        ("Tal & Karaman\nnative", "tk_rust_native", "#fdbf6f"), ("Tal & Karaman,\nOmar's values", "tk_rust_omar_values_torque", "#D55E00")]


def val(S, run, kind):
    c = S[run]
    if kind == "thrust":
        return c["thrust_N"]["rms_diff"]
    if kind == "roll_pitch":
        return 1e3 * float(np.mean([c["tau_x"]["rms_diff"], c["tau_y"]["rms_diff"]]))
    return 1e3 * c["tau_z"]["rms_diff"]


fig, ax = plt.subplots(1, 3, figsize=(13.5, 4.4))
for a, kind, title, unit in ((ax[0], "thrust", "Thrust", "N"), (ax[1], "roll_pitch", "Roll / pitch torque", "mN·m"), (ax[2], "yaw", "Yaw torque", "mN·m")):
    x = np.arange(len(RUNS))
    a.bar(x, [val(A, r, kind) for _, r, _ in RUNS], color=[c for *_, c in RUNS], width=0.62)
    a.plot(x, [val(B, r, kind) for _, r, _ in RUNS], "ko", ms=6, label="today's flight")
    a.set_xticks(x); a.set_xticklabels([n for n, *_ in RUNS], fontsize=8)
    a.set_title(title); a.set_ylabel(f"RMS difference to Omar C [{unit}]"); a.grid(axis="y", alpha=0.3)
ax[0].legend(fontsize=8)
fig.suptitle("Same logged inputs through every variant: difference to Omar C (bars: A8 2026-10-02)")
fig.tight_layout()
out = Path(sys.argv[sys.argv.index("--out") + 1]) if "--out" in sys.argv else HERE / "out/indi_equal_simple.png"
fig.savefig(out, dpi=150); print("wrote", out)
