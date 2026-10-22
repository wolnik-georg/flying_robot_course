#!/usr/bin/env python3
"""Figure for the step-3 doc: C_D identification per thrust input, and test R2 of ROM only / KNODE / label-fit ceiling / NS2 v2.
  ~/.pyenv/versions/flying_robots/bin/python experiments/fbl/stage4_knode/plot_real.py"""
import json
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
OUT = Path(__file__).resolve().parents[1] / "out/stage4"; r = json.load(open(OUT / "real_v0/results.json")); ceil = json.load(open(OUT / "ceiling.json"))
tags = [("s103", "command ×1.03"), ("base", "command ×1.00"), ("s0977", "command ×0.977"), ("rpm", "rotor-speed force")]
col = {"s103": "tab:red", "base": "tab:orange", "s0977": "tab:green", "rpm": "tab:blue"}
fig, ax = plt.subplots(1, 2, figsize=(12.5, 4.2))
for t, lab in tags:
    g = r[f"cd_grid_{t}"] if f"cd_grid_{t}" in r else r["cd_grid"]
    cds = sorted(float(k) for k in g if float(k) > 0); v = np.array([g[str(c) if str(c) in g else repr(c)]["val"] for c in cds])
    ax[0].plot(cds, v / v.min(), "-o", ms=3, c=col[t], label=f"{lab} (min at C_D {cds[int(v.argmin())]})")
ax[0].set_xlabel("ROM drag coefficient C_D"); ax[0].set_ylabel("one-step validation loss / its minimum"); ax[0].set_ylim(0.95, 2.2); ax[0].set_title("C_D identification (nets off)", fontsize=9); ax[0].legend(fontsize=8)
xs = np.arange(len(tags)); w = 0.2
ax[1].bar(xs - w, [r[t]["rom_only"]["r2"] for t, _ in tags], w, label="ROM only (C_D from the left)", color="0.65")
ax[1].bar(xs, [r[t]["knode"]["r2"] for t, _ in tags], w, label="ROM + KNODE nets (force only)", color="tab:blue")
ax[1].bar(xs + w, [r[t]["ns2_v2"]["r2"] for t, _ in tags], w, label="NS2 v2 (logged)", color="tab:green")
ax[1].axhline(ceil["(r, dz) only"]["r2"], c="k", ls="--", lw=1); ax[1].text(3.45, ceil["(r, dz) only"]["r2"] + 0.01, "ceiling of any function of (r, dz): %.2f" % ceil["(r, dz) only"]["r2"], ha="right", fontsize=8)
ax[1].set_xticks(xs); ax[1].set_xticklabels([l for _, l in tags], fontsize=8); ax[1].set_ylabel("R² against the INDI label (z), 37 test flights"); ax[1].set_ylim(0, 0.85); ax[1].legend(fontsize=8, loc="upper left"); ax[1].set_title("thrust input matters, the nets add nothing", fontsize=9)
fig.tight_layout(); fig.savefig(OUT / "real_v0/step3_overview.png", dpi=130)
