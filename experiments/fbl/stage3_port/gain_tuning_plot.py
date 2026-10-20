#!/usr/bin/env python3
"""Figure for the FBL gain tuning: ~/.pyenv/versions/flying_robots/bin/python experiments/fbl/stage3_port/gain_tuning_plot.py"""
import json, re
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
O = Path(__file__).resolve().parents[1] / "out/stage3"; sc = json.load(open(O / "gain_tuning_scale.json")); ef = json.load(open(O / "gain_tuning_eff.json")); gr = json.load(open(O / "gain_tuning_grid1.json"))
A = [1, 0.5, 0.35, 0.25, 0.18, 0.12]; Bs = [1, 0.5, 0.25, 0.12]
fig, ax = plt.subplots(1, 3, figsize=(17, 4.8))
M = np.array([[sc[f"fbl_rom|v0.3|a{a}|b{b}"]["z_rms_cm"] for a in A] for b in Bs]); T = np.array([[sc[f"fbl_rom|v0.3|a{a}|b{b}"]["tripped"] or sc[f"fbl_rom|v0.3|a{a}|b{b}"]["tilt_max_deg"] > 20 for a in A] for b in Bs])
im = ax[0].imshow(np.where(T, np.nan, M), cmap="YlGn_r", vmin=0, vmax=6, aspect="auto"); ax[0].imshow(np.where(T, 1.0, np.nan), cmap="Reds", vmin=0, vmax=1.6, aspect="auto")
for i in range(len(Bs)):
    for j in range(len(A)): ax[0].text(j, i, "unstable" if T[i, j] else f"{M[i, j]:.1f}", ha="center", va="center", fontsize=8)
ax[0].set_xticks(range(len(A))); ax[0].set_xticklabels(A); ax[0].set_yticks(range(len(Bs))); ax[0].set_yticklabels(Bs)
ax[0].set_xlabel("position-loop gains (kp, kv, ki) as a fraction of the authors' values"); ax[0].set_ylabel("rate-loop gain (kr) as a fraction"); ax[0].set_title("Authors' gains are unstable with our motor lag (44 ms)\nz RMS [cm], A8 at 0.3 m/s, realistic plant")
sp = [0.1, 0.3, 0.5]; w = 0.27
for k, (key, lab, c) in enumerate((("omar|v{}", "Omar geometric (S0 controller)", "#7f7f7f"), ("fbl_none|v{}|a0.25|b0.5", "FBL, no downwash model", "#9ecae1"), ("fbl_rom|v{}|a0.25|b0.25", "FBL + downwash model, tuned gains", "#2171b5"))):
    ax[1].bar(np.arange(3) + (k - 1) * w, [-gr[key.format(s)]["z_dip_cm"] for s in sp], w, color=c, label=lab)
ax[1].set_xticks(range(3)); ax[1].set_xticklabels([f"{s} m/s" for s in sp]); ax[1].set_ylabel("crossing dip [cm] (bigger = worse)"); ax[1].legend(fontsize=8); ax[1].set_title("With tuned gains FBL beats Omar in simulation\n(A8, realistic plant; simulation, not hardware)")
kis = [0.75, 3, 8, 16]; effs = [1.0, 0.95, 0.9, 0.85, 0.8]
for i, kz in enumerate(kis): ax[2].plot(effs, [ef[f"fbl_rom|v0.3|eff{e}|kiz{kz}"]["z_rms_cm"] for e in effs], "o-", label=f"z-integral gain {kz}")
ax[2].invert_xaxis(); ax[2].set_xlabel("share of the commanded thrust that really arrives"); ax[2].set_ylabel("z RMS [cm]"); ax[2].legend(fontsize=8); ax[2].set_title("A thrust-model error lifts the drone off its target;\na stronger z integral limits it")
fig.suptitle("FBL gain tuning in the C-in-the-loop simulation (command delay 4 ms, motor lag 44 ms, sensor noise)", fontsize=11); fig.tight_layout(); out = O.parent.parent / "gain_tuning.png"; fig.savefig(out, dpi=125); print(out)
