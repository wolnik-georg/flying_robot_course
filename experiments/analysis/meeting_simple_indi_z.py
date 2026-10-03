#!/usr/bin/env python3
"""Task 2 — INDI variant z tracking on 2026-10-02 cf5 (meeting 2026-10-03).

Run:
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/meeting_simple_indi_z.py
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from meeting_hw_common import (
    LOGS,
    TUMBLE_TILT_DEG,
    cf5_config,
    liftoff_time,
    load_radio_csv,
    meta_json_for_csv,
    tilt_deg,
    z_command_for_vehicle,
    STEADY_AFTER_LIFTOFF_S,
    TILT_EXCLUDE_DEG,
)
from meeting_simple_common import (
    OUT_ASSETS,
    OUT_TABLES,
    apply_meeting_style,
    hold_end_formation,
    variant_name,
)

PNG = OUT_ASSETS / "fig_indi_z_tracking.png"
MD = OUT_TABLES / "table_indi_z_error.md"

DATE = "2026-10-02"
PRE_FIX = {"A8_cf5_2026-10-02_18-57-49.csv", "A8_cf5_2026-10-02_18-59-17.csv"}
ABORT = {"A8_cf5_2026-10-02_19-08-13.csv"}
FLIP_CRASH = {"A8_cf5_2026-10-02_18-22-05.csv"}
EXCURSION_FLIGHTS = {
    "A8_cf5_2026-10-02_18-23-23.csv",
    "A1_cf5_2026-10-02_18-48-33.csv",
}


@dataclass
class IndiRow:
    path: Path
    scenario: str
    variant: str
    flag: str
    mean_err_cm: float
    max_abs_err_cm: float
    max_tilt_deg: float
    t_lift: float
    z_cmd_m: float
    t_steady_lo: float
    t_steady_hi: float
    duration_s: float


def flight_stamp(path: Path) -> str:
    m = re.search(r"_(\d{2}-\d{2}-\d{2})\.csv", path.name)
    return m.group(1) if m else "?"


def flight_flag(path: Path, max_tilt: float) -> str:
    if path.name in PRE_FIX:
        return "pre-fix"
    if path.name in ABORT:
        return "aborted"
    if path.name in FLIP_CRASH or max_tilt > 150:
        return "crashed (flip/abort)"
    if path.name in EXCURSION_FLIGHTS or (max_tilt > TUMBLE_TILT_DEG and max_tilt <= 150):
        return "completed, tilt excursion >45°"
    return "clean"


def analyze_cf5(path: Path) -> IndiRow | None:
    if DATE not in path.name or not path.name.startswith(("A1_cf5_", "A8_cf5_")):
        return None
    try:
        radio_meta, cols = load_radio_csv(path)
    except ValueError:
        return None
    mj = meta_json_for_csv(path)
    ctrl, mode, _ = cf5_config(radio_meta, mj)
    var = variant_name(ctrl, mode)
    if var is None:
        return None
    scen = path.name.split("_")[0]
    t = cols["time_s"]
    z = cols["pos_z"]
    tilt = tilt_deg(cols)
    max_tilt = float(np.max(tilt)) if len(tilt) else 0.0
    flag = flight_flag(path, max_tilt)
    t_lift = liftoff_time(cols)
    dur = float(t[-1] - t[0]) if len(t) else float("nan")
    if t_lift is None:
        return IndiRow(
            path, scen, var, flag, float("nan"), float("nan"), max_tilt,
            float("nan"), float("nan"), float("nan"), float("nan"), dur,
        )
    z_cmd = z_command_for_vehicle(radio_meta, mj, "cf5")
    t_lo = t_lift + STEADY_AFTER_LIFTOFF_S
    t_hi = hold_end_formation(scen, radio_meta, mj, t_lift, float(t[-1]))
    mask = (t >= t_lo) & (t <= t_hi) & (tilt <= TILT_EXCLUDE_DEG)
    if not np.any(mask):
        err_cm = float("nan")
        max_abs = float("nan")
    else:
        err = (z[mask] - z_cmd) * 100.0
        err_cm = float(np.mean(err))
        max_abs = float(np.max(np.abs(err)))
    return IndiRow(
        path, scen, var, flag, err_cm, max_abs, max_tilt, t_lift, z_cmd, t_lo, t_hi, dur,
    )


def pick_representative(rows: list[IndiRow]) -> IndiRow | None:
    clean = [r for r in rows if r.flag == "clean" and np.isfinite(r.mean_err_cm)]
    if not clean:
        return None
    med = float(np.median([r.mean_err_cm for r in clean]))
    return min(clean, key=lambda r: abs(r.mean_err_cm - med))


def median_summary(rows: list[IndiRow], scen: str, var: str, flags: set[str]) -> str:
    vals = [
        r.mean_err_cm
        for r in rows
        if r.scenario == scen and r.variant == var and r.flag in flags and np.isfinite(r.mean_err_cm)
    ]
    if not vals:
        return f"- **{scen} / {var}:** n=0"
    return f"- **{scen} / {var}:** n={len(vals)}, median = {float(np.median(vals)):.2f} cm"


def main() -> None:
    apply_meeting_style()
    OUT_ASSETS.mkdir(parents=True, exist_ok=True)
    OUT_TABLES.mkdir(parents=True, exist_ok=True)

    rows: list[IndiRow] = []
    for p in sorted(LOGS.glob(f"*_cf5_{DATE}_*.csv")):
        r = analyze_cf5(p)
        if r:
            rows.append(r)

    fig = plt.figure(figsize=(12, 7))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.2, 0.8], hspace=0.35)
    colors = {"Ours": "#1F5C4D", "Omar C": "#2E6DA4", "Omar Rust": "#9A6B12"}

    for col, scen in enumerate(["A1", "A8"]):
        ax_z = fig.add_subplot(gs[0, col])
        ax_e = fig.add_subplot(gs[1, col], sharex=ax_z)
        scen_rows = [r for r in rows if r.scenario == scen]
        for var in ("Ours", "Omar C", "Omar Rust"):
            sub = [r for r in scen_rows if r.variant == var]
            rep = pick_representative(sub)
            if rep is None:
                continue
            _, cols = load_radio_csv(rep.path)
            t = cols["time_s"] - rep.t_lift
            z = cols["pos_z"]
            err = (z - rep.z_cmd_m) * 100.0
            c = colors[var]
            ax_z.plot(t, z, color=c, lw=1.6, label=var)
            z_cmd_used = rep.z_cmd_m
            ax_e.plot(t, err, color=c, lw=1.3)
        ax_z.axhline(z_cmd_used, color="black", ls="--", lw=1.2, label="commanded z")
        ax_z.set_ylabel("z position (m)")
        ax_z.set_title({"A1": "A1 — two drones stacked, hovering", "A8": "A8 — two drones swap sides"}[scen])
        ax_z.legend(fontsize=10, loc="best")
        ax_e.axhline(0, color="black", lw=0.6)
        ax_e.set_xlabel("time since liftoff (s)")
        ax_e.set_ylabel("z error (cm)")
        ax_e.set_ylim(-35, 35)

    fig.suptitle("Height of the bottom drone vs commanded height (one flight per variant)", y=1.01)
    fig.savefig(PNG, bbox_inches="tight")
    print(f"wrote {PNG}")

    md = [
        "# Table — INDI z error (cf5 bottom, 2026-10-02)",
        "",
        "## Steady window",
        f"- Liftoff: pos_z > 0.05 m; steady = liftoff + {STEADY_AFTER_LIFTOFF_S} s … hold end (A1: param_hold; A8: duration_s − 2.5 s); tilt ≤ {TILT_EXCLUDE_DEG}°.",
        "- Representative plot: **clean** flight closest to variant median mean error; legend time = HH-MM-SS stamp.",
        "- **pre-fix:** 18-57-49, 18-59-17 (ki_z=16 tumbles). **aborted:** 19-08-13 (~18 s). **crashed (flip/abort):** 18-22-05 or tilt≈180°.",
        "- **completed, tilt excursion >45°:** flight ran but peak |roll|/|pitch| > 45° (e.g. 18-23-23, 18-48-33).",
        "",
        "### A1 Ours `19-15-47` — z→0 dips at t≈16 s and ≈23–25 s",
        "- Full trace shows **pos_z near 0 m** after the hold window; this is **post-scenario landing / ground contact** on the radio log (A1 hold ends ~15 s after liftoff; landing follows).",
        "- **Mean/max in the table use only the steady window** (tilt ≤ 25°, before landing); the dips are **not** included in those statistics.",
        "",
        "| scenario | variant | flight | flag | mean z error (cm) | max |z error| (cm) | max tilt (°) |",
        "|---|---|---|---|---:|---:|---:|",
    ]
    for r in sorted(rows, key=lambda x: (x.scenario, x.variant, x.path.name)):
        md.append(
            f"| {r.scenario} | {r.variant} | `{r.path.name}` | {r.flag} | "
            f"{r.mean_err_cm:.2f} | {r.max_abs_err_cm:.2f} | {r.max_tilt_deg:.1f} |"
        )
    md.append("")
    md.append("## Variant medians — clean only")
    for scen in ("A1", "A8"):
        for var in ("Ours", "Omar C", "Omar Rust"):
            md.append(median_summary(rows, scen, var, {"clean"}))
    md.append("")
    md.append("## Variant medians — clean + completed tilt excursion >45°")
    for scen in ("A1", "A8"):
        for var in ("Ours", "Omar C", "Omar Rust"):
            md.append(median_summary(rows, scen, var, {"clean", "completed, tilt excursion >45°"}))

    MD.write_text("\n".join(md) + "\n")
    print(f"wrote {MD}")


if __name__ == "__main__":
    main()
