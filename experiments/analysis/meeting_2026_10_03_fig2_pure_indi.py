#!/usr/bin/env python3
"""Fig. 2 — Pure-INDI variants on A1/A8 (2026-10-02 hardware).

Run:
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/meeting_2026_10_03_fig2_pure_indi.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from meeting_hw_common import (
    COL,
    LOGS,
    REPO,
    TILT_EXCLUDE_DEG,
    TUMBLE_TILT_DEG,
    apply_mpl_style,
    att_std_steady_window,
    meta_json_for_csv,
    ours_a8_marker_class,
    paired_cf_second,
    steady_height_error,
    variant_label,
    load_radio_csv,
)

OUT_DIR = REPO / "docs/meetings/assets/2026-10-03"
MD_OUT = REPO / "experiments/analysis/out/meeting_2026-10-03/fig2_pure_indi.md"
PNG = OUT_DIR / "fig2_pure_indi_height_error.png"
SEP_PNG = OUT_DIR / "fig2b_a8_crossing_separation.png"

DATE = "2026-10-02"
SCENARIOS = ("A1", "A8")
YLIM_CM = 35.0

CLEAN_A8_OURS = ("19-09-54", "19-11-30")


def list_comparison_flights() -> list[Path]:
    out = []
    for scen in SCENARIOS:
        for p in sorted(LOGS.glob(f"{scen}_cf5_{DATE}_*.csv")):
            out.append(p)
    return out


def cf_second_ref_ok(path: Path, sm) -> tuple[bool, str]:
    mj = meta_json_for_csv(path)
    if mj:
        d = mj["per_drone"]["cf_second"]
        if int(d["controller"]) != 6 or int(d["ctrl_mode"]) != 0:
            return False, f"cf_second c={d['controller']} mode={d['ctrl_mode']}"
    if sm.max_tilt_deg > TILT_EXCLUDE_DEG:
        return False, f"max tilt {sm.max_tilt_deg:.0f}° > {TILT_EXCLUDE_DEG:.0f}°"
    if not sm.included:
        return False, sm.exclude_reason or "not included"
    return True, "ok"


def median_flights_for_table(scen: str, var: str, fl: list) -> tuple[list, str]:
    """Return (flights in median, description)."""
    inc = [f for f in fl if f.included]
    if scen == "A8" and "Ours" in var:
        clean = [f for f in inc if any(s in f.csv_path.name for s in CLEAN_A8_OURS)]
        names = ", ".join(f"`{f.csv_path.name}`" for f in clean)
        return clean, f"median over clean post-ki_z=0 only: {names or 'none'}"
    return inc, "median over all included flights: " + ", ".join(f"`{f.csv_path.name}`" for f in inc)


def a8_crossing_sep_err(cf5_path: Path) -> float | None:
    p2 = paired_cf_second(cf5_path)
    if p2 is None:
        return None
    _, c5 = load_radio_csv(cf5_path)
    _, c2 = load_radio_csv(p2)
    mj = meta_json_for_csv(cf5_path)
    if not mj:
        return None
    dz_cmd = float(mj["params"]["dz"])
    t5 = c5["time_s"]
    t0 = max(t5[0], c2["time_s"][0])
    t1 = min(t5[-1], c2["time_s"][-1])
    m = (t5 >= t0) & (t5 <= t1)
    t = t5[m]
    y5 = c5["pos_y"][m]
    y2 = np.interp(t, c2["time_s"], c2["pos_y"])
    z5 = c5["pos_z"][m]
    z2 = np.interp(t, c2["time_s"], c2["pos_z"])
    rel_y = y5 - y2
    sgn = np.sign(rel_y)
    sgn[sgn == 0] = 1
    cross_idx = np.where(np.diff(sgn) != 0)[0]
    idx = int(cross_idx[0]) if len(cross_idx) else int(np.argmin(np.abs(rel_y)))
    return float((z2[idx] - z5[idx]) - dz_cmd)


def main() -> None:
    apply_mpl_style()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    MD_OUT.parent.mkdir(parents=True, exist_ok=True)

    rows = []
    ref_rows = []
    ref_excluded = []

    for path in list_comparison_flights():
        try:
            sm = steady_height_error(path)
        except ValueError:
            continue
        lbl = variant_label(sm.controller, sm.ctrl_mode)
        if lbl is None:
            continue
        sm.extra["variant"] = lbl
        rows.append(sm)
        p2 = paired_cf_second(path)
        if p2:
            try:
                rs = steady_height_error(p2, vehicle="cf_second")
            except ValueError:
                continue
            ok, reason = cf_second_ref_ok(p2, rs)
            rs.extra["variant"] = "cf_second ref"
            rs.extra["pair"] = path.name
            if ok:
                ref_rows.append(rs)
            else:
                ref_excluded.append((p2.name, reason, rs.mean_err_cm if rs.included else float("nan")))

    variants = ["Omar C (c=9)", "Omar Rust (c=10)", "Ours full INDI (c=6, mode=3)"]
    fig = plt.figure(figsize=(11, 5.2))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 0.35], hspace=0.45)
    ax_a1 = fig.add_subplot(gs[0, 0])
    ax_a8 = fig.add_subplot(gs[0, 1], sharey=ax_a1)
    ax_att = fig.add_subplot(gs[1, :])

    table: dict[tuple[str, str], list] = {}
    median_notes: dict[tuple[str, str], str] = {}

    for ax_i, (ax, scen) in enumerate(((ax_a1, "A1"), (ax_a8, "A8"))):
        for vi, var in enumerate(variants):
            fl = [r for r in rows if r.scenario == scen and r.extra.get("variant") == var]
            table[(scen, var)] = fl
            med_set, note = median_flights_for_table(scen, var, fl)
            median_notes[(scen, var)] = note
            x = vi
            if not fl:
                continue
            valid = [f.mean_err_cm for f in med_set]
            med = float(np.median(valid)) if valid else float("nan")
            if valid:
                yplot = np.clip(med, -YLIM_CM, YLIM_CM)
                ax.bar(x, yplot, width=0.6, color=COL["green_dark"], alpha=0.75)

            for j, f in enumerate(fl):
                dx = (j - (len(fl) - 1) / 2) * 0.09
                hollow = False
                lbl_ann = ""
                if scen == "A8" and "Ours" in var:
                    cls = ours_a8_marker_class(f.csv_path)
                    hollow = cls != "filled_clean"
                    lbl_ann = cls.replace("_", " ")
                elif not f.included or f.max_tilt_deg > TUMBLE_TILT_DEG:
                    hollow = True
                y = f.mean_err_cm if f.included else 0.0
                if not f.included:
                    hollow = True
                if hollow:
                    ax.scatter(
                        x + dx, np.clip(y, -YLIM_CM, YLIM_CM) if f.included else 0,
                        marker="o", facecolors="none", edgecolors=COL["red"], s=42, linewidths=1.2, zorder=5,
                    )
                else:
                    ax.scatter(x + dx, np.clip(y, -YLIM_CM, YLIM_CM), s=36, c=COL["green_mid"], zorder=5)
            if valid:
                ax.text(x, np.clip(med, -YLIM_CM, YLIM_CM), f"n={len(fl)}", ha="center", va="bottom", fontsize=7)

        refs = [r for r in ref_rows if r.scenario == scen]
        if refs:
            ref_med = float(np.median([r.mean_err_cm for r in refs]))
            ax.bar(3.2, np.clip(ref_med, -YLIM_CM, YLIM_CM), width=0.5, color=COL["gray"], alpha=0.55)
            for j, r in enumerate(refs):
                ax.scatter(3.2 + (j - (len(refs) - 1) / 2) * 0.07, np.clip(r.mean_err_cm, -YLIM_CM, YLIM_CM), s=28, c=COL["gray"], zorder=5)
        n_ex = len([1 for _n, _r, _ in ref_excluded if _n.startswith(scen)])
        if n_ex:
            ax.text(3.2, -YLIM_CM * 0.85, f"+{n_ex} ref excl.", ha="center", fontsize=6, color=COL["red"])

        ax.set_xticks(list(range(len(variants))) + [3.2])
        ax.set_xticklabels([v.replace(" ", "\n") for v in variants] + ["cf_second\nref"], fontsize=6)
        ax.set_title(f"{scen} — cf5 height error")
        ax.axhline(0, color="black", lw=0.8)
        ax.set_ylim(-YLIM_CM, YLIM_CM)

    ax_a1.set_ylabel("mean z − z_cmd (cm)")
    fig.suptitle("Fig. 2 — Pure INDI comparison (2026-10-02); hollow = crash/abort/excluded from median", y=1.01)

    # A1 roll/pitch std in steady window
    att_labels = []
    roll_std = []
    pitch_std = []
    for var in variants:
        fl = [r for r in rows if r.scenario == "A1" and r.extra.get("variant") == var]
        rs = []
        ps = []
        for f in fl:
            st = att_std_steady_window(f.csv_path)
            if st:
                rs.append(st[0])
                ps.append(st[1])
        if rs:
            att_labels.append(var.split()[0] + "\n" + var.split()[1])
            roll_std.append(float(np.mean(rs)))
            pitch_std.append(float(np.mean(ps)))
    if att_labels:
        x = np.arange(len(att_labels))
        w = 0.35
        ax_att.bar(x - w / 2, roll_std, w, label="roll std (°)", color=COL["amber"])
        ax_att.bar(x + w / 2, pitch_std, w, label="pitch std (°)", color=COL["blue"])
        ax_att.set_xticks(x)
        ax_att.set_xticklabels(att_labels, fontsize=7)
        ax_att.set_ylabel("deg (steady window)")
        ax_att.set_title("A1 — attitude oscillation in steady window (mean std across flights)")
        ax_att.legend(fontsize=7)

    fig.savefig(PNG, bbox_inches="tight")
    print(f"wrote {PNG}")

    # Fig 2b — only clean A8 variants
    sep_keep = False
    sep_md = ""
    fig2, ax2 = plt.subplots(figsize=(6, 3.8))
    for var in variants:
        fl = [r for r in rows if r.scenario == "A8" and r.extra.get("variant") == var and r.included]
        if "Ours" in var:
            fl = [f for f in fl if ours_a8_marker_class(f.csv_path) == "filled_clean"]
        vals = []
        for f in fl:
            if f.included:
                v = a8_crossing_sep_err(f.csv_path)
                if v is not None:
                    vals.append(v * 100.0)
        if vals:
            sep_keep = True
            ax2.scatter([var.split()[0]] * len(vals), vals, s=40, c=COL["blue"])
    ax2.axhline(0, color="black", lw=0.8)
    ax2.set_ylabel("Δsep − dz_cmd at crossing (cm)")
    ax2.set_title("A8 — separation at y crossing (clean flights only)")
    fig2.tight_layout()
    if sep_keep:
        fig2.savefig(SEP_PNG, bbox_inches="tight")
        print(f"wrote {SEP_PNG}")
        sep_md = f"![fig2b]({SEP_PNG.relative_to(REPO)})"
    else:
        sep_md = "*(fig2b omitted — insufficient clean A8 crossing data)*"

    context = {
        ("A1", "Omar C (c=9)"): "±7–9 cm (session; uSD)",
        ("A1", "Omar Rust (c=10)"): "8–11 cm below cmd",
        ("A8", "Omar C (c=9)"): "15–25 cm above cmd",
        ("A8", "Omar Rust (c=10)"): "18–19 cm above cmd",
        ("A8", "Ours full INDI (c=6, mode=3)"): "4.4–4.5 cm (19-09-54, 19-11-30 only)",
    }

    md = [
        "# Fig. 2 — Pure INDI variants (2026-10-02)",
        "",
        f"![fig2]({PNG.relative_to(REPO)})",
        "",
        sep_md,
        "",
        "## Rules",
        "Formation radio CSVs + sidecar meta; steady window = liftoff+6 s through hold, tilt≤25°.",
        "**cf_second ref:** only paired flights where cf_second has controller=6, ctrl_mode=0, "
        f"max tilt ≤ {TILT_EXCLUDE_DEG:.0f}°, valid steady window.",
        "**Ours A8 median:** only `19-09-54` and `19-11-30` (ki_z=0, post-fix). "
        "Hollow: `18-57-49`, `18-59-17` (pre-fix crash), `19-08-13` (abort).",
        "",
        "## Summary table (**high confidence** on listed flights)",
        "",
        "| Scenario | Variant | Median mean err (cm) | Flights in median |",
        "|---|---|---:|---|",
    ]

    for scen in SCENARIOS:
        for var in variants:
            fl = table.get((scen, var), [])
            med_set, note = median_flights_for_table(scen, var, fl)
            med_m = float(np.median([f.mean_err_cm for f in med_set])) if med_set else float("nan")
            md.append(f"| {scen} | {var} | {med_m:.2f} | {note} |")

    md.append("")
    md.append("## Per-flight detail")
    md.append("")
    md.append("| File | mean err (cm) | max tilt (°) | in median? |")
    md.append("|---|---:|---:|---|")
    for scen in SCENARIOS:
        for var in variants:
            fl = table.get((scen, var), [])
            med_set, _ = median_flights_for_table(scen, var, fl)
            med_ids = {f.csv_path.name for f in med_set}
            for f in fl:
                in_med = "yes" if f.csv_path.name in med_ids else "no"
                md.append(
                    f"| `{f.csv_path.name}` | {f.mean_err_cm if f.included else '—':} | "
                    f"{f.max_tilt_deg:.0f} | {in_med} |"
                )

    md.append("")
    md.append(f"## cf_second reference (included n={len(ref_rows)})")
    for r in ref_rows:
        md.append(f"- `{r.csv_path.name}` (pair `{r.extra.get('pair')}`): {r.mean_err_cm:.2f} cm")
    md.append(f"## cf_second excluded (n={len(ref_excluded)})")
    for name, reason, err in ref_excluded:
        md.append(f"- `{name}`: {reason}" + (f" (mean {err:.1f} cm)" if err == err else ""))

    md += [
        "",
        "## fig2b",
        "Kept: separation at first y-crossing for clean A8 flights; interpret as radio Δz vs commanded dz only (medium confidence).",
        "",
        "## What this figure does not show",
        "uSD Omar error channels; pre-10-02 INDI history.",
    ]
    MD_OUT.write_text("\n".join(md) + "\n")
    print(f"wrote {MD_OUT}")


if __name__ == "__main__":
    main()
