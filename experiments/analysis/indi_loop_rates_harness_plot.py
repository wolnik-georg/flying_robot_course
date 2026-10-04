#!/usr/bin/env python3
"""Plot harness_sweep.json (run: INDI_HARNESS_SWEEP=1 cargo test harness_sweep -- --nocapture)."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

OUT = Path(__file__).resolve().parent / "out" / "indi_loop_rates"
SWEEP = OUT / "harness_sweep.json"


def main() -> None:
    if not SWEEP.exists():
        print(f"Missing {SWEEP}; run harness sweep first.")
        return
    rows = json.loads(SWEEP.read_text())
    # Group by decimate_hold, facet prewarp x filt_dt
    fig, axes = plt.subplots(2, 2, figsize=(10, 8), sharey=True)
    for ax, (decim, title) in zip(axes.flat, [(False, "1 kHz law"), (True, "500 Hz hold"), (False, ""), (True, "")]):
        pass
    # simpler grouped bars: freq vs notch for each filt_dt/prewarp
    fig, ax = plt.subplots(figsize=(10, 5))
    labels = []
    freqs = []
    sigmas = []
    for i, r in enumerate(rows):
        if r["decimate_hold"]:
            continue
        labels.append(
            f"dt={r['filt_dt_us']} pw={int(r['prewarp'])} n={int(r['notch_en'])}"
        )
        freqs.append(r["freq_hz"])
        sigmas.append(r["omega_sigma"])
    x = np.arange(len(labels))
    ax.bar(x, freqs, color="steelblue")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=7)
    ax.axhline(6.3, color="gray", ls="--", label="6.3 Hz ref")
    ax.set_ylabel("Limit-cycle freq [Hz] (harness)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT / "fig_harness_sweep_freq.png", dpi=150)
    print(f"Wrote {OUT / 'fig_harness_sweep_freq.png'}")


if __name__ == "__main__":
    main()
