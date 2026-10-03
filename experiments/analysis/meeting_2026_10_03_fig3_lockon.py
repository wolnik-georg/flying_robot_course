#!/usr/bin/env python3
"""Fig. 3 — NS2 2026-10-03 tumble → lock-on timeline.

Run:
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/meeting_2026_10_03_fig3_lockon.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from lock_on_scan import (  # noqa: E402
    TILT_DEG,
    analyse_pair,
    load_radio,
    liftoff_time,
)
from meeting_hw_common import COL, LOGS, apply_mpl_style

OUT_DIR = REPO / "docs/meetings/assets/2026-10-03"
MD_OUT = REPO / "experiments/analysis/out/meeting_2026-10-03/fig3_lockon.md"
PNG = OUT_DIR / "fig3_ns2_lockon_timeline.png"

FLIGHTS = (
    ("A8_cf5_2026-10-03_13-10-32.csv", "A8_cf_second_2026-10-03_13-10-32.csv"),
    ("A8_cf5_2026-10-03_13-15-00.csv", "A8_cf_second_2026-10-03_13-15-00.csv"),
)


def plot_flight(fig, row_base, p5_name, p2_name, ann: dict):
    p5 = LOGS / p5_name
    p2 = LOGS / p2_name
    m5, c5 = load_radio(p5)
    m2, c2 = load_radio(p2)
    t0 = max(c5["time_s"][0], c2["time_s"][0])
    t1 = min(c5["time_s"][-1], c2["time_s"][-1])
    t = c5["time_s"]
    mask = (t >= t0) & (t <= t1)
    t = t[mask]
    y5, z5 = c5["pos_y"][mask], c5["pos_z"][mask]
    y2 = np.interp(t, c2["time_s"], c2["pos_y"])
    z2 = np.interp(t, c2["time_s"], c2["pos_z"])
    tilt = np.sqrt(c5["roll"][mask] ** 2 + c5["pitch"][mask] ** 2)

    ax_y = fig.add_subplot(4, 1, row_base)
    ax_z = fig.add_subplot(4, 1, row_base + 1, sharex=ax_y)
    ax_t = fig.add_subplot(4, 1, row_base + 2, sharex=ax_y)

    for ax, ylabel, s5, s2 in (
        (ax_y, "y (m)", y5, y2),
        (ax_z, "z (m)", z5, z2),
    ):
        ax.plot(t, s5, color=COL["green_mid"], lw=1.2, label="cf5")
        ax.plot(t, s2, color=COL["gray"], lw=1.0, ls="--", label="cf_second")
        ax.set_ylabel(ylabel)
        ax.legend(loc="upper right", fontsize=7)

    ax_t.plot(t, tilt, color=COL["red"], lw=1.2)
    ax_t.axhline(TILT_DEG, color=COL["amber"], ls=":", lw=1, label=f"tilt>{TILT_DEG:.0f}°")
    ax_t.set_ylabel("cf5 tilt (°)")
    ax_t.legend(loc="upper right", fontsize=7)

    for ax in (ax_y, ax_z, ax_t):
        for key, color, lbl in (
            ("liftoff_s", "black", "liftoff"),
            ("tilt20_s", COL["amber"], "tilt>20°"),
            ("lock_s", COL["red"], "lock-on"),
        ):
            tv = ann.get(key)
            if tv is not None:
                ax.axvline(tv, color=color, lw=1, alpha=0.85)
        ax.grid(True, alpha=0.35)

    short = p5_name.replace("A8_cf5_", "").replace(".csv", "")
    ax_y.set_title(f"Flight {short}", fontsize=10, loc="left")
    if row_base == 1:
        ax_t.set_xlabel("time (s)")


def main() -> None:
    apply_mpl_style()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    MD_OUT.parent.mkdir(parents=True, exist_ok=True)

    annotations = []
    for p5n, p2n in FLIGHTS:
        r = analyse_pair(p5n.replace(".csv", "").replace("A8_cf5_", "A8_"), LOGS / p5n, LOGS / p2n)
        if r:
            annotations.append((p5n, r))

    fig = plt.figure(figsize=(10, 8))
    fig.suptitle(
        "cf5 tumbles first; position lock-on to cf_second follows\n(likely tracker re-assignment)",
        fontsize=11,
    )
    plot_flight(fig, 1, FLIGHTS[0][0], FLIGHTS[0][1], annotations[0][1] if annotations else {})
    # second flight: rows 4-6 — use gridspec
    plt.close(fig)

    fig, axes = plt.subplots(6, 1, figsize=(10, 9), sharex="col")
    fig.suptitle(
        "cf5 tumbles first; position lock-on to cf_second follows (likely tracker re-assignment)",
        fontsize=11,
    )

    for fi, (p5n, p2n) in enumerate(FLIGHTS):
        ann = next((a[1] for a in annotations if a[0] == p5n), {})
        p5, p2 = LOGS / p5n, LOGS / p2n
        m5, c5 = load_radio(p5)
        m2, c2 = load_radio(p2)
        t0 = max(c5["time_s"][0], c2["time_s"][0])
        t1 = min(c5["time_s"][-1], c2["time_s"][-1])
        t = c5["time_s"]
        mask = (t >= t0) & (t <= t1)
        t = t[mask]
        y5, z5 = c5["pos_y"][mask], c5["pos_z"][mask]
        y2 = np.interp(t, c2["time_s"], c2["pos_y"])
        z2 = np.interp(t, c2["time_s"], c2["pos_z"])
        tilt = np.maximum(np.abs(c5["roll"][mask]), np.abs(c5["pitch"][mask]))
        base = fi * 3
        for j, (ylabel, a, b) in enumerate((("y (m)", y5, y2), ("z (m)", z5, z2))):
            ax = axes[base + j]
            ax.plot(t, a, color=COL["green_mid"], lw=1.2, label="cf5")
            ax.plot(t, b, color=COL["gray"], lw=1.0, ls="--", label="cf_second")
            ax.set_ylabel(ylabel)
            if fi == 0 and j == 0:
                ax.legend(fontsize=7, loc="upper right")
            ax.set_title(p5n.replace("A8_cf5_", ""), fontsize=9, loc="left")
        ax_t = axes[base + 2]
        ax_t.plot(t, tilt, color=COL["red"], lw=1.2)
        ax_t.axhline(20, color=COL["amber"], ls=":", lw=1)
        ax_t.set_ylabel("cf5 tilt (°)")
        for key, color in (("liftoff_s", "black"), ("tilt20_s", COL["amber"]), ("lock_s", COL["red"])):
            tv = ann.get(key)
            if tv is not None:
                for ax in axes[base : base + 3]:
                    ax.axvline(tv, color=color, lw=1, alpha=0.8)

    axes[-1].set_xlabel("time (s)")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(PNG, bbox_inches="tight")
    print(f"wrote {PNG}")

    md = [
        "# Fig. 3 — NS2 2026-10-03 lock-on timeline",
        "",
        f"![fig3]({PNG.relative_to(REPO)})",
        "",
        "## Rules (**high confidence**, `lock_on_scan.py`)",
        "- Shared radio clock; interpolate cf_second onto cf5 timestamps.",
        "- Liftoff: first cf5 `pos_z > 0.12` m (scan script default).",
        "- Lock-on: earliest time such that |Δy|<5 cm and |Δz|<15 cm for remainder of flight (≥0.4 s sustained).",
        "- Vertical lines: liftoff, first tilt>20°, lock-on.",
        "",
        "| Flight | liftoff (s) | first tilt>20° (s) | lock-on (s) | max tilt (°) | lock after flip? |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for p5n, p2n in FLIGHTS:
        ann = next((a[1] for a in annotations if a[0] == p5n), None)
        if not ann:
            md.append(f"| {p5n} | — | — | — | — | — |")
            continue
        md.append(
            f"| {p5n} | {ann['liftoff_s']:.1f} | {ann['tilt20_s']:.1f} | {ann['lock_s']:.1f} | "
            f"{ann['max_tilt_deg']:.1f} | {'yes' if ann['lock_after_tilt20'] else 'no'} |"
        )

    md += [
        "",
        "## What this figure does not show",
        "Does not prove mocap ID swap mechanism (radio positions only); no uSD estimator state; no RNN predictions.",
    ]
    MD_OUT.write_text("\n".join(md) + "\n")
    print(f"wrote {MD_OUT}")


if __name__ == "__main__":
    main()
