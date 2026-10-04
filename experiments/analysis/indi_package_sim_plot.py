#!/usr/bin/env python3
"""Plots for package simulation sweep."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

OUT = Path(__file__).resolve().parent / "out" / "indi_package"
JSON = OUT / "package_sim_results.json"


def main() -> None:
    if not JSON.exists():
        print("Missing package_sim_results.json — run indi_package_sim.py first")
        return
    data = json.loads(JSON.read_text())
    val = data.get("baseline_validation", [])
    if len(val) >= 2:
        fig, ax = plt.subplots(figsize=(6, 3.5))
        labels = ["ours_flown", "omar_c"]
        freqs = [val[0]["result"]["freq_hz"], val[1]["result"]["freq_hz"]]
        rms = [val[0]["result"]["gyro_rms_deg_s"], val[1]["result"]["gyro_rms_deg_s"]]
        x = np.arange(2)
        ax.bar(x - 0.15, freqs, width=0.3, label="freq Hz")
        ax.bar(x + 0.15, np.array(rms) / 100.0, width=0.3, label="gyro RMS /100")
        ax.set_xticks(x, labels)
        ax.set_title(f"Baselines (validated={data.get('harness_validated_for_baseline_separation')})")
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(OUT / "fig_baseline_bars.png")
        plt.close(fig)

    grid = data.get("grid", [])
    if grid:
        fig, ax = plt.subplots(figsize=(8, 4))
        sub = [g for g in grid if not g.get("decimate_500") and g["kp"] == 64]
        names = [f"kr={g['kr']} rs={g['res_sign']}" for g in sub]
        y = [g["gyro_rms_deg_s"] for g in sub]
        ax.barh(range(len(y)), y)
        ax.set_yticks(range(len(y)), names, fontsize=7)
        ax.set_xlabel("gyro RMS [deg/s] (stiff pos, illustrative)")
        fig.tight_layout()
        fig.savefig(OUT / "fig_grid_gyro_rms.png")
        plt.close(fig)
    print(f"Wrote plots to {OUT}")


if __name__ == "__main__":
    main()
