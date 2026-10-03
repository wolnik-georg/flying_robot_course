#!/usr/bin/env python3
"""Task 1 — geometric + Z-integral z tracking (meeting 2026-10-03).

Run:
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/meeting_simple_z_geometric.py
"""

from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt

from meeting_hw_common import CONTROLS_LOGS, REPO, load_controls_csv, load_radio_csv
from meeting_simple_common import (
    OUT_ASSETS,
    OUT_TABLES,
    SOLO_F8_KI16,
    SOLO_HOVER_KI16,
    SOLO_HOVER_Z_START_M,
    STEADY_AFTER_LIFTOFF_S,
    PLATEAU_DROP_M,
    Z_BAND_CM,
    Z_ERR_YLIM_CM,
    analyze_controls_track,
    analyze_formation_track,
    apply_meeting_style,
    controls_time_and_z,
    formation_candidate_paths,
)

PNG = OUT_ASSETS / "fig_z_tracking_geometric.png"
MD = OUT_TABLES / "table_z_tracking_geometric.md"


def trace_controls(row):
    _, cols = load_controls_csv(row.path)
    t_abs, z = controls_time_and_z(cols)
    t = t_abs - row.t_lift
    err = (z - row.z_cmd_m) * 100.0
    land_start = row.t_steady_hi - row.t_lift
    keep = (t >= 0) & (t <= land_start + 0.5)
    return t[keep], err[keep], land_start


def trace_formation(row):
    _, cols = load_radio_csv(row.path)
    t = cols["time_s"] - row.t_lift
    err = (cols["pos_z"] - row.z_cmd_m) * 100.0
    land_start = row.t_steady_hi - row.t_lift
    keep = (t >= 0) & (t <= land_start + 0.5)
    return t[keep], err[keep], land_start


def plot_panel(ax, rows: list, trace_fn, title: str):
    inc = [r for r in rows if r.included]
    ax.axhspan(-Z_BAND_CM, Z_BAND_CM, color="#4FB39A", alpha=0.18, zorder=0, label=f"±{Z_BAND_CM:g} cm")
    ax.axhline(0, color="black", lw=0.8, alpha=0.5)
    for r in inc:
        t, err, land = trace_fn(r)
        ax.plot(t, err, color="#2E6DA4", lw=1.0, alpha=0.85)
    ax.set_ylim(-Z_ERR_YLIM_CM, Z_ERR_YLIM_CM)
    ax.set_xlabel("time since takeoff (s)")
    ax.set_ylabel("z error (cm)")
    ax.set_title(title)
    ax.text(0.98, 0.04, f"{len(inc)} flights (one line each)", transform=ax.transAxes, ha="right", fontsize=10)
    ax.legend(loc="upper right", fontsize=10)


def summarize(rows: list):
    inc = [r for r in rows if r.included]
    if not inc:
        return None
    return {
        "n": len(inc),
        "mean_cm": float(np.mean([r.mean_err_cm for r in inc])),
        "mean_abs_cm": float(np.mean([r.mean_abs_err_cm for r in inc])),
        "max_abs_cm": float(np.max([r.max_abs_err_cm for r in inc])),
    }


def main() -> None:
    apply_meeting_style()
    OUT_ASSETS.mkdir(parents=True, exist_ok=True)
    OUT_TABLES.mkdir(parents=True, exist_ok=True)

    hover_rows = [analyze_controls_track(CONTROLS_LOGS / n) for n in SOLO_HOVER_KI16]
    f8_rows = [analyze_controls_track(CONTROLS_LOGS / n) for n in SOLO_F8_KI16]
    form_paths = [p for p in formation_candidate_paths() if "cf_second" in p.name]
    form_rows = [analyze_formation_track(p, "cf_second") for p in form_paths]
    a1_rows = [r for r in form_rows if r.scenario == "A1"]
    a8_rows = [r for r in form_rows if r.scenario == "A8"]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=False)
    plot_panel(axes[0, 0], hover_rows, trace_controls, "Hover (1 drone)")
    plot_panel(axes[0, 1], f8_rows, trace_controls, "Figure-8 (1 drone)")
    plot_panel(
        axes[1, 0], a1_rows, trace_formation,
        "A1 (2 drones stacked, top drone shown)",
    )
    plot_panel(
        axes[1, 1], a8_rows, trace_formation,
        "A8 (2 drones swap sides, top drone shown)",
    )
    fig.suptitle("z error = measured z − commanded z (geometric controller with Z-integral)", y=1.02)
    fig.tight_layout()
    fig.savefig(PNG, bbox_inches="tight")
    print(f"wrote {PNG}")

    md = [
        "# Table — geometric Z-integral z tracking",
        "",
        f"![fig]({PNG.relative_to(REPO)})",
        "",
        "## Window rule",
        "- **Formation (radio):** liftoff = first `pos_z` > 0.05 m; steady = liftoff + 6.0 s … hold end; tilt ≤ 25°.",
        f"- **Solo (Controls):** segment start = first `z` ≥ {SOLO_HOVER_Z_START_M} m (logs begin on ground at z≈−0.02 m; "
        f"first z>0.05 m is **not** hover); steady = segment start + {STEADY_AFTER_LIFTOFF_S} s … hold end; tilt ≤ 25°.",
        "- **Plot time axis:** seconds since that segment start (solo) or liftoff (formation).",
        f"- **Steady end:** walk forward from steady start; last sample with "
        f"`z ≥ max(plateau_median − {PLATEAU_DROP_M*100:.0f} cm, z_cmd − 2 cm)` before sustained descent "
        "(solo + formation); formation also capped by A1 `param_hold` / A8 `duration_s`.",
        "- **Figure-8 time:** lap resets in raw CSV are stitched into one monotonic timeline before windows/plots.",
        "",
        "## Exclusions (listed per panel; not drawn)",
        "- tilt > 45°; wrong controller/ki_z; no segment start; no steady samples.",
        "",
    ]

    panels = [
        ("Hover", hover_rows),
        ("Figure-8", f8_rows),
        ("A1", a1_rows),
        ("A8", a8_rows),
    ]
    all_exc: list[str] = []
    for pname, rows in panels:
        s = summarize(rows)
        md.append(f"## {pname} — summary (steady window only)")
        if s:
            md.append(
                f"- n={s['n']} | mean error {s['mean_cm']:.2f} cm | "
                f"mean |error| {s['mean_abs_cm']:.2f} cm | max |error| {s['max_abs_cm']:.2f} cm"
            )
        else:
            md.append("- no included flights")
        md.append("")
        md.append("| flight | clean | mean (cm) | mean |e| (cm) | max |e| (cm) | note |")
        md.append("|---|---:|---:|---:|---:|---|")
        for r in sorted(rows, key=lambda x: x.path.name):
            note = r.exclude_reason or r.meta_note
            if not r.included:
                all_exc.append(f"`{r.path.name}` ({pname}): {note}")
            md.append(
                f"| `{r.path.name}` | {'yes' if r.included else 'no'} | "
                f"{r.mean_err_cm:.2f} | {r.mean_abs_err_cm:.2f} | {r.max_abs_err_cm:.2f} | {note} |"
            )
        md.append("")

    md.append("## Change vs previous table (window end = log end − 2.5 s)")
    md.append(
        "- Previous max |error| on hovers included **landing descent**; "
        "descent-based end gives max |error| **≈2 cm** on 09-30 hovers (see Hover rows)."
    )
    md.append("")
    md.append("## All exclusions")
    for line in sorted(set(all_exc)):
        md.append(f"- {line}")

    MD.write_text("\n".join(md) + "\n")
    print(f"wrote {MD}")


if __name__ == "__main__":
    main()
