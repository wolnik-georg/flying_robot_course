#!/usr/bin/env python3
"""Plot the equal-parameter replay ladder (reads out/indi_replay_equal/<tag>/summary.json).
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/indi_replay_equal_plot.py A8 [--out file.png]"""
import json, sys
from pathlib import Path
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
tag = sys.argv[1]
out = Path(sys.argv[sys.argv.index("--out") + 1]) if "--out" in sys.argv else HERE / "out" / "indi_replay_equal" / tag / "ladder.png"
S = json.loads((HERE / "out" / "indi_replay_equal" / tag / "summary.json").read_text())

ref = S["vs_omar_c_noKI"]   # attitude integral off: in an open-loop replay Omar's K_I winds up and swamps the torque comparison
ladder = [k for k in ref if k.startswith("ours_L")]
labels = ["native", "+ mass", "+ kt", "+ position\ngains", "+ residual\nsign", "+ attitude\ngains", "+ inertia", "+ filters\n30 Hz", "+ limits\noff"]
fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
x = range(len(ladder))
ax[0].plot(x, [ref[k]["thrust_N"]["rms_diff"] for k in ladder], "o-", color="#0072B2", label="ours (ladder)")
for nm, col, lab in (("tk_rust_native", "#D55E00", "Tal & Karaman native"), ("tk_rust_omar_values_torque", "#009E73", "Tal & Karaman, Omar's values")):
    if nm in ref:
        ax[0].axhline(ref[nm]["thrust_N"]["rms_diff"], color=col, ls="--", label=lab)
ax[0].set_xticks(list(x)); ax[0].set_xticklabels(labels[: len(ladder)], fontsize=8)
ax[0].set_ylabel("thrust RMS difference to Omar C [N]"); ax[0].set_title("Thrust"); ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8)
for axis, col in (("tau_x", "#0072B2"), ("tau_y", "#E69F00"), ("tau_z", "#CC79A7")):
    ax[1].plot(x, [ref[k][axis]["rms_diff"] * 1e3 for k in ladder], "o-", color=col, label=f"ours {axis}")
ax[1].set_xticks(list(x)); ax[1].set_xticklabels(labels[: len(ladder)], fontsize=8)
ax[1].set_ylabel("torque RMS difference to Omar C [mN m]"); ax[1].set_title("Torque"); ax[1].grid(alpha=0.3); ax[1].legend(fontsize=8)
fig.suptitle(f"Equal-parameter replay ({tag}): distance to Omar C (attitude integral off) while ours takes over Omar's values step by step")
fig.tight_layout(); fig.savefig(out, dpi=150); print("wrote", out)
