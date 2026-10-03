#!/usr/bin/env python3
"""Fig. 1 — Z-integral before/after from Controls/logs (unbiased quasi-static plateau).

Run:
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/meeting_2026_10_03_fig1_z_integral.py

Controller key evidence: crazyswarm2/crazyflie_examples/crazyflie_examples/flight.py
  - Hover applies yaml controller under phase \"trajectory\" (_log_phase line ~1194).
  - CSV writes trajectory_stabilizer_controller / trajectory_ctrl_mode per phase (~591-595).
  - yaml_stabilizer_controller is the yaml snapshot at load (~1112), not per-phase effective.
See also docs/41 §20 (log hygiene) and docs/lab_sessions/2026-09-28_alt_indi_shakedown.md.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from meeting_hw_common import (
    COL,
    CONTROLS_LOGS,
    REPO,
    TILT_EXCLUDE_DEG,
    TUMBLE_TILT_DEG,
    apply_mpl_style,
    controls_tilt_deg,
    load_all_meta_lines,
    load_controls_csv,
    trajectory_ki_z,
)

OUT_DIR = REPO / "docs/meetings/assets/2026-10-03"
MD_OUT = REPO / "experiments/analysis/out/meeting_2026-10-03/fig1_z_integral.md"
PNG = OUT_DIR / "fig1_z_integral_height_error.png"

Z_CMD_M = 1.0
Z_QS_MIN_M = 0.8
VZ_QS_MAX = 0.05
SKIP_QS_S = 3.0
FIG1_YLIM_CM = 30.0

FLIGHT_PY = (
    Path.home() / "Desktop/crazyswarm2/crazyflie_examples/crazyflie_examples/flight.py"
)

# After validation set (docs/51): ki_z=16, trajectory c=6 mode=0
AFTER_HOVERS = (
    "hover_mode1_kt0.008_2026-09-30_19-16-19.csv",
    "hover_mode1_kt0.008_2026-09-30_19-16-56.csv",
    "hover_mode1_kt0.008_2026-09-30_19-20-10.csv",
)
AFTER_F8 = (
    "figure8_mode1_kt0.05_2026-09-30_19-20-41.csv",
    "figure8_mode1_kt0.05_2026-09-30_19-21-14.csv",
)
AFTER_F8_TUMBLE = "figure8_mode1_kt0.05_2026-09-30_19-17-28.csv"
AFTER_OMAR_HOVERS = (
    "hover_mode1_kt0.008_2026-09-30_17-27-16.csv",
    "hover_mode1_kt0.008_2026-09-30_17-48-47.csv",
    "hover_mode1_kt0.008_2026-09-30_18-08-52.csv",
    "hover_mode1_kt0.008_2026-09-30_18-21-30.csv",
    "hover_mode1_kt0.008_2026-09-30_18-22-08.csv",
    "hover_mode1_kt0.008_2026-09-30_18-40-46.csv",
    "hover_mode1_kt0.008_2026-09-30_18-41-23.csv",
    "hover_mode1_kt0.008_2026-09-30_18-54-28.csv",
)


@dataclass
class QSResult:
    path: Path
    group: str
    controller: int
    ctrl_mode: int
    ki_z: float
    z_cmd_m: float
    t_lo: float
    t_hi: float
    mean_err_m: float
    mean_first_half_m: float
    mean_second_half_m: float
    max_tilt_deg: float
    n_samples: int
    included: bool
    exclude_reason: str = ""
    meta: dict[str, str] = field(default_factory=dict)

    @property
    def mean_err_cm(self) -> float:
        return self.mean_err_m * 100.0


def active_controller(meta: dict[str, str]) -> tuple[int, int]:
    """Controller during hover hold = trajectory phase (flight.py hover_mode path)."""
    c = int(float(meta.get("trajectory_stabilizer_controller", -1)))
    m = int(float(meta.get("trajectory_ctrl_mode", -1)))
    return c, m


def quasi_static_analysis(path: Path, z_cmd: float = Z_CMD_M) -> QSResult:
    meta = load_all_meta_lines(path)
    ctrl, mode = active_controller(meta)
    kz = trajectory_ki_z(meta)
    _, cols = load_controls_csv(path)
    t = cols["time_s"]
    z = cols["z"]
    vz = cols["vz"]
    tilt = controls_tilt_deg(cols)
    max_tilt = float(np.max(tilt)) if len(tilt) else 0.0

    if max_tilt > TUMBLE_TILT_DEG:
        return QSResult(
            path, "", ctrl, mode, kz, z_cmd, float("nan"), float("nan"),
            float("nan"), float("nan"), float("nan"), max_tilt, 0, False,
            f"max tilt {max_tilt:.1f}° > {TUMBLE_TILT_DEG:.0f}°", meta,
        )

    above = z > Z_QS_MIN_M
    if not np.any(above):
        return QSResult(
            path, "", ctrl, mode, kz, z_cmd, float("nan"), float("nan"),
            float("nan"), float("nan"), float("nan"), max_tilt, 0, False,
            f"never reached z > {Z_QS_MIN_M} m", meta,
        )

    i0 = int(np.where(above)[0][0])
    t_qs0 = float(t[i0])
    after = (t >= t_qs0) & (z < Z_QS_MIN_M)
    idx_below = np.where(after)[0]
    t_qs1 = float(t[idx_below[0]]) if len(idx_below) else float(t[-1])
    t_lo = t_qs0 + SKIP_QS_S
    mask = (
        (t >= t_lo)
        & (t <= t_qs1)
        & (np.abs(vz) < VZ_QS_MAX)
        & (tilt <= TILT_EXCLUDE_DEG)
    )
    if not np.any(mask):
        return QSResult(
            path, "", ctrl, mode, kz, z_cmd, t_lo, t_qs1,
            float("nan"), float("nan"), float("nan"), max_tilt, 0, False,
            "no quasi-static samples after filters", meta,
        )

    err = z[mask] - z_cmd
    mid = len(err) // 2
    return QSResult(
        path,
        "",
        ctrl,
        mode,
        kz,
        z_cmd,
        t_lo,
        t_qs1,
        float(np.mean(err)),
        float(np.mean(err[:mid])) if mid else float("nan"),
        float(np.mean(err[mid:])) if mid else float("nan"),
        max_tilt,
        int(mask.sum()),
        True,
        "",
        meta,
    )


def classify_hover(path: Path) -> str:
    """Return group label or exclusion reason prefix."""
    meta = load_all_meta_lines(path)
    if meta.get("run_trajectory") != "hover":
        return "skip:not_hover"
    c, m = active_controller(meta)
    kz = trajectory_ki_z(meta)
    name = path.name
    if name in AFTER_HOVERS or name in AFTER_F8:
        return "after_ki_z16"
    if name in AFTER_OMAR_HOVERS:
        return "excl:omar_c10"
    if name == AFTER_F8_TUMBLE:
        return "excl:f8_tumble"
    if "2026-09-30" in name:
        return "excl:2026-09-30_other"
    if c == 10:
        return "excl:omar_c10"
    if c != 6:
        return f"excl:controller_{c}"
    if m == 3:
        return "before_full_indi_mode3"
    if m != 0:
        return f"excl:ctrl_mode_{m}"
    if abs(kz - 16.0) < 0.01:
        return "excl:ki_z16"
    return "before_geometric"


def discover_hovers() -> list[Path]:
    return sorted(
        p for p in CONTROLS_LOGS.glob("hover*.csv") if p.name.startswith(("hover_mode0_", "hover_mode1_"))
    )


def pick_representative(flights: list[QSResult]) -> QSResult | None:
    inc = [f for f in flights if f.included]
    if not inc:
        return None
    med = float(np.median([f.mean_err_cm for f in inc]))
    return min(inc, key=lambda f: abs(f.mean_err_cm - med))


def plot_trace(ax, res: QSResult, color: str, label: str) -> None:
    _, cols = load_controls_csv(res.path)
    t = cols["time_s"]
    err_cm = (cols["z"] - res.z_cmd_m) * 100.0
    ax.plot(t, err_cm, color=color, lw=1.3, label=label)
    if res.included and np.isfinite(res.t_lo):
        ax.axvspan(res.t_lo, res.t_hi, color=color, alpha=0.15)


def main() -> None:
    apply_mpl_style()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    MD_OUT.parent.mkdir(parents=True, exist_ok=True)

    all_paths = discover_hovers()
    f8_after = [CONTROLS_LOGS / n for n in AFTER_F8]
    f8_tumble = CONTROLS_LOGS / AFTER_F8_TUMBLE

    by_group: dict[str, list[QSResult]] = {
        "before_geometric": [],
        "before_full_indi_mode3": [],
        "after_ki_z16": [],
    }
    exclusions: list[tuple[str, str, QSResult]] = []

    for p in all_paths:
        grp = classify_hover(p)
        try:
            qs = quasi_static_analysis(p)
        except ValueError as e:
            msg = "empty or missing header" if "empty or missing" in str(e) else str(e)
            exclusions.append((p.name, msg, QSResult(p, grp, -1, -1, 0, Z_CMD_M, float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), 0, 0, False, msg)))
            continue
        qs.group = grp
        if grp == "before_geometric":
            if qs.included:
                by_group["before_geometric"].append(qs)
            else:
                exclusions.append((p.name, qs.exclude_reason, qs))
        elif grp == "before_full_indi_mode3":
            if qs.included:
                by_group["before_full_indi_mode3"].append(qs)
            else:
                exclusions.append((p.name, qs.exclude_reason, qs))
        elif grp == "after_ki_z16":
            if qs.included:
                by_group["after_ki_z16"].append(qs)
            else:
                exclusions.append((p.name, qs.exclude_reason, qs))
        elif grp.startswith("excl:"):
            exclusions.append((p.name, grp.replace("excl:", ""), qs))
        # skip:not_hover ignored

    for p in f8_after + [f8_tumble]:
        try:
            qs = quasi_static_analysis(p)
        except ValueError as e:
            exclusions.append((p.name, str(e), QSResult(p, "after", -1, -1, 0, Z_CMD_M, float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), 0, 0, False, str(e))))
            continue
        qs.group = "after_ki_z16" if p.name in AFTER_F8 else "excl:f8_tumble"
        if p.name in AFTER_F8:
            if qs.included:
                by_group["after_ki_z16"].append(qs)
            else:
                exclusions.append((p.name, qs.exclude_reason, qs))
        else:
            exclusions.append((p.name, qs.exclude_reason or "tumble", qs))

    before = by_group["before_geometric"]
    after = by_group["after_ki_z16"]
    mode3 = by_group["before_full_indi_mode3"]

    ex_before = pick_representative(before)
    ex_after = pick_representative([f for f in after if "hover" in f.path.name])

    fig = plt.figure(figsize=(11.5, 4.6))
    gs = fig.add_gridspec(1, 3, width_ratios=[1.45, 1.05, 0.9], wspace=0.34)
    ax_ts = fig.add_subplot(gs[0, 0])
    ax_bar = fig.add_subplot(gs[0, 1])
    ax_half = fig.add_subplot(gs[0, 2])

    ax_ts.axhline(0, color="black", lw=0.8, alpha=0.6)
    if ex_before:
        plot_trace(ax_ts, ex_before, COL["gray"], "Before geometric")
    if ex_after:
        plot_trace(ax_ts, ex_after, COL["green_mid"], "After ki_z=16")
    ax_ts.set_xlabel("time (s)")
    ax_ts.set_ylabel("z − 1.0 m (cm)")
    ax_ts.set_title("Examples from included flights (shaded = quasi-static window)")
    ax_ts.legend(fontsize=8)

    groups_plot = [
        ("Before\ngeometric", before, COL["gray"]),
        ("After\nki_z=16", after, COL["green_mid"]),
    ]
    n_out_axis = 0
    for i, (_lab, fl, color) in enumerate(groups_plot):
        means = [f.mean_err_cm for f in fl if f.included]
        if not means:
            ax_bar.text(i, 0, "n=0", ha="center")
            continue
        in_ax = [m for m in means if abs(m) <= FIG1_YLIM_CM]
        n_out_axis += len(means) - len(in_ax)
        med = float(np.median(means))
        ax_bar.bar(i, np.clip(med, -FIG1_YLIM_CM, FIG1_YLIM_CM), width=0.55, color=color, alpha=0.85)
        rng = np.random.default_rng(7)
        for j, m in enumerate(means):
            if abs(m) > FIG1_YLIM_CM:
                ax_bar.annotate(
                    "",
                    xy=(i, np.sign(m) * FIG1_YLIM_CM * 0.9),
                    xytext=(i + 0.28, m),
                    arrowprops=dict(arrowstyle="->", color=COL["red"], lw=0.7),
                )
            else:
                ax_bar.scatter(i + (rng.random() - 0.5) * 0.14, m, s=28, c=color, edgecolors="white", lw=0.4, zorder=3)
        ax_bar.text(i, np.clip(med, -FIG1_YLIM_CM, FIG1_YLIM_CM), f"n={len(means)}", ha="center", va="bottom", fontsize=8)

    ax_bar.axhline(0, color="black", lw=0.8)
    ax_bar.set_ylim(-FIG1_YLIM_CM, FIG1_YLIM_CM)
    ax_bar.set_xticks([0, 1])
    ax_bar.set_xticklabels([g[0] for g in groups_plot], fontsize=8)
    ax_bar.set_ylabel("quasi-static mean error (cm)")
    ax_bar.set_title(f"Fixed axis ±{FIG1_YLIM_CM:.0f} cm")
    if n_out_axis:
        ax_bar.text(0.98, 0.02, f"{n_out_axis} beyond axis", transform=ax_bar.transAxes, ha="right", fontsize=7)

    # Early vs late half: after hovers + before geometric (grey)
    labels, h1, h2 = [], [], []
    for f in before:
        if f.included and "hover" in f.path.name:
            labels.append(f"B\n{f.path.name[25:33]}")
            h1.append(f.mean_first_half_m * 1000)
            h2.append(f.mean_second_half_m * 1000)
    for f in after:
        if f.included and "hover" in f.path.name:
            labels.append(f"A\n{f.path.name[25:33]}")
            h1.append(f.mean_first_half_m * 1000)
            h2.append(f.mean_second_half_m * 1000)
    if labels:
        x = np.arange(len(labels))
        w = 0.35
        ax_half.bar(x - w / 2, h1, w, color=COL["gray"], alpha=0.5, label="1st half")
        ax_half.bar(x + w / 2, h2, w, color=COL["green_mid"], alpha=0.85, label="2nd half")
        ax_half.set_xticks(x)
        ax_half.set_xticklabels(labels, fontsize=6)
        ax_half.set_ylabel("mean error (mm)")
        ax_half.set_title("Early vs late plateau (B=before, A=after)")
        ax_half.legend(fontsize=7)
        ax_half.axhline(0, color="black", lw=0.6)

    fig.suptitle("Fig. 1 — Z-integral validation (Controls/logs, quasi-static plateau)", y=1.02)
    fig.savefig(PNG, bbox_inches="tight")
    print(f"wrote {PNG}")

    def stats(fl: list[QSResult]):
        v = [f.mean_err_cm for f in fl if f.included]
        if not v:
            return None
        return {
            "n": len(v),
            "min_cm": float(np.min(v)),
            "max_cm": float(np.max(v)),
            "median_cm": float(np.median(v)),
            "mean_cm": float(np.mean(v)),
            "frac_below_minus_5cm": float(np.mean([1 if x < -5 else 0 for x in v])),
        }

    pre_s = stats(before)
    post_s = stats(after)

    md = [
        "# Fig. 1 — Z-integral height error (Controls/logs, v3 quasi-static)",
        "",
        f"![fig1]({PNG.relative_to(REPO)})",
        "",
        "## What changed vs v2",
        "- **Plateau rule:** replaced “within 5 cm of target” gate (biased toward good trackers) with "
        f"**unconditional quasi-static window**: first `z > {Z_QS_MIN_M} m`, then until `z < {Z_QS_MIN_M} m`, "
        f"keep samples with `|vz| < {VZ_QS_MAX} m/s`, tilt ≤ {TILT_EXCLUDE_DEG}°, skip first {SKIP_QS_S} s of that run.",
        "- **Controller key:** classify on **`trajectory_stabilizer_controller` / `trajectory_ctrl_mode`**, "
        "not `yaml_*` (yaml is config-at-load; hover hold logs under phase `trajectory`).",
        "- **Examples:** left panel traces chosen from **included** flights closest to group median.",
        "",
        "## Controller-key evidence",
        f"- Writer: `{FLIGHT_PY}` — `_log_phase` stores per-phase effective controller (~409-415); "
        "`_apply_flight_settings` logs effective values after overrides (~418-480); hover hold calls "
        "`_apply_flight_settings(..., \"trajectory\", yaml_controller, traj_ctrl_mode, ...)` (~1191-1200) "
        "or `_log_phase(\"trajectory\", ...)` (~1202).",
        "- `_save_log` writes `# meta:{phase}_stabilizer_controller` for takeoff/trajectory/landing (~591-595); "
        "`yaml_*` is yaml-at-load only (~587-590, ~1112).",
        "- Project docs: `docs/41` §20 (log hygiene); `docs/lab_sessions/2026-09-28_alt_indi_shakedown.md` "
        "(use trajectory_*, not yaml_*).",
        "",
        "## Groups",
        f"- **Before (geometric):** hover, traj c=6, traj ctrl_mode=0, ki_z≠16, not 2026-09-30 validation.",
        "- **Before (full INDI, separate):** same but traj ctrl_mode=3 — **not** mixed into Before median.",
        f"- **After:** listed validation files (3 hovers + 2 figure-8) with ki_z=16, traj c=6 mode=0.",
        "- **Excluded:** Omar Rust (traj c=10), tumble, wrong controller/mode, no quasi-static samples.",
        "",
        f"## Error: `z − {Z_CMD_M} m`",
        "",
        "### Before geometric — included (plateau mean cm)",
    ]
    for f in sorted(before, key=lambda x: x.path.name):
        if f.included:
            md.append(
                f"- `{f.path.name}` — **{f.mean_err_cm:.2f} cm** "
                f"(traj c={f.controller} mode={f.ctrl_mode}, n={f.n_samples})"
            )
    md.append("")
    md.append("### Before geometric — excluded")
    for name, reason, qs in exclusions:
        if classify_hover(CONTROLS_LOGS / name) == "before_geometric" or (
            name.startswith("hover") and qs.ctrl_mode == 0 and qs.controller == 6
            and name not in AFTER_HOVERS and "2026-09-30" not in name
        ):
            if not any(f.path.name == name for f in before):
                md.append(f"- `{name}` — {reason}")

    md.append("")
    md.append("### Before full INDI (mode=3) — separate group")
    for f in mode3:
        md.append(f"- `{f.path.name}` — {f.mean_err_cm:.2f} cm (included)" if f.included else f"- `{f.path.name}` — {f.exclude_reason}")
    md.append(f"- n included mode-3 hovers: **{len(mode3)}**")

    md.append("")
    md.append("### After ki_z=16 — included")
    for f in sorted(after, key=lambda x: x.path.name):
        md.append(
            f"- `{f.path.name}` — **{f.mean_err_cm:.2f} cm** "
            f"(1st/2nd half: {f.mean_first_half_m*1000:.2f}/{f.mean_second_half_m*1000:.2f} mm)"
        )

    md.append("")
    md.append("### Exclusions (all)")
    for name, reason, qs in sorted(exclusions, key=lambda x: x[0]):
        extra = f", mean={qs.mean_err_cm:.1f} cm" if qs.included else ""
        md.append(f"- `{name}` — {reason}{extra}")

    md += [
        "",
        "## Medians / spread (**high confidence**, quasi-static rule)",
        "",
        "| Group | n | min | median | max | mean | frac < −5 cm |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for label, s in [("Before geometric", pre_s), ("After ki_z=16", post_s)]:
        if s:
            md.append(
                f"| {label} | {s['n']} | {s['min_cm']:.2f} | {s['median_cm']:.2f} | {s['max_cm']:.2f} | "
                f"{s['mean_cm']:.2f} | {s['frac_below_minus_5cm']:.2f} |"
            )
        else:
            md.append(f"| {label} | 0 | — | — | — | — | — |")

    md.append("")
    md.append("## docs/51 claims")
    if pre_s:
        md.append(
            f"- **~10 cm sag before:** under this solo-hover quasi-static rule, spread "
            f"**{pre_s['min_cm']:.1f}…{pre_s['max_cm']:.1f} cm** (median **{pre_s['median_cm']:.1f} cm**); "
            f"**{pre_s['frac_below_minus_5cm']*100:.0f}%** of included before hovers below −5 cm. "
            "**Partial:** several hovers at −8…−12 cm support large sag; median is pulled up by near-zero flights. "
            "**Formation ~10 cm** claim is **not directly tested** here (**medium**)."
        )
    if post_s:
        after_h = [f for f in after if "hover" in f.path.name and f.included]
        late = [f.mean_second_half_m * 1000 for f in after_h]
        md.append(
            f"- **2–3 mm after:** full-window hover means ≈ **{np.mean([f.mean_err_cm for f in after_h])*10:.1f} mm** "
            f"(**discrepancy** vs 2–3 mm headline); **late-half** hovers "
            f"**{np.min(late):.2f}…{np.max(late):.2f} mm** (**supports** convergence claim, **high**)."
        )

    md.append("")
    md.append("## v2 ‘never within 5 cm’ flights under new rule")
    v2_never = [
        "hover_mode0_2026-06-16_18-47-39.csv",
        "hover_mode1_kt0.008_2026-09-19_14-00-53.csv",
        "hover_mode1_kt0.008_2026-09-19_14-24-29.csv",
        "hover_mode1_kt0.008_2026-09-23_21-27-02.csv",
    ]
    for n in v2_never:
        p = CONTROLS_LOGS / n
        if not p.exists():
            md.append(f"- `{n}` — not found")
            continue
        qs = quasi_static_analysis(p)
        if qs.included:
            md.append(f"- `{n}` — included; mean **{qs.mean_err_cm:.2f} cm**")
        else:
            md.append(f"- `{n}` — {qs.exclude_reason}")

    md += [
        "",
        "## Unresolved",
        "- Exact line numbers drift if `flight.py` changes; paths verified in repo checkout on analyst machine.",
        "",
        "## What this figure does not show",
        "Formation A1/A8 logs; uSD ctrltarget_z; yaml_*-only classification.",
    ]
    MD_OUT.write_text("\n".join(md) + "\n")
    (MD_OUT.with_suffix(".json")).write_text(json.dumps({"before": pre_s, "after": post_s}, indent=2))
    print(f"wrote {MD_OUT}")


if __name__ == "__main__":
    main()
