#!/usr/bin/env python3
"""Plot SIL package results (optional; needs working matplotlib)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

OUT = Path(__file__).resolve().parent / "out" / "indi_sil_package"


def main() -> None:
    path = OUT / "results.json"
    if not path.is_file():
        print("missing", path, file=sys.stderr)
        sys.exit(1)
    payload = json.loads(path.read_text())
    results = payload["results"]
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except (ImportError, AttributeError) as e:
        print("skip plot:", e)
        sys.exit(0)

    ok = [r for r in results if "metrics" in r]
    labels = [r["label"] for r in ok]
    gyro = [r["metrics"]["gyro_rms_deg_s"] for r in ok]
    freq = [r["metrics"]["dominant_hz_gyro_x"] or 0 for r in ok]
    zerr = [r["metrics"]["mean_z_err_cm"] for r in ok]

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    x = np.arange(len(labels))
    axes[0].bar(x, gyro, color="steelblue")
    axes[0].axhspan(260, 290, color="red", alpha=0.12)
    axes[0].axhspan(55, 97, color="green", alpha=0.12)
    axes[0].set_ylabel("Gyro RMS [°/s]")

    axes[1].bar(x, freq, color="darkorange")
    axes[1].axhspan(4.7, 5.7, color="red", alpha=0.12)
    axes[1].axhspan(3.2, 3.9, color="green", alpha=0.12)
    axes[1].set_ylabel("f_dom gyro_x [Hz]")

    axes[2].bar(x, zerr, color="purple")
    axes[2].axhspan(-19, -17, color="red", alpha=0.12)
    axes[2].axhspan(2, 4, color="green", alpha=0.12)
    axes[2].set_ylabel("mean z err [cm]")
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(labels, rotation=75, ha="right", fontsize=5)
    fig.tight_layout()
    fig.savefig(OUT / "fig_grid_metrics.png", dpi=150)
    plt.close(fig)

    # Baseline PSD-style comparison (steady-window scalar only here)
    base = [r for r in ok if r["label"].startswith("baseline")]
    if len(base) >= 2:
        fig2, ax = plt.subplots(figsize=(6, 3))
        ax.bar([0, 1], [base[0]["metrics"]["gyro_rms_deg_s"], base[1]["metrics"]["gyro_rms_deg_s"]])
        ax.set_xticks([0, 1])
        ax.set_xticklabels(["ours SIL", "Omar C SIL"])
        ax.set_ylabel("Gyro RMS [°/s]")
        ax.set_title("Baseline vs flight bands (gray/red/green in full grid plot)")
        fig2.tight_layout()
        fig2.savefig(OUT / "fig_baseline_gyro.png", dpi=150)
        plt.close(fig2)
    print("Wrote plots to", OUT)


if __name__ == "__main__":
    main()
