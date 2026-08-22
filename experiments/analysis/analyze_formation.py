#!/usr/bin/env python3
"""
Analyse a multi-drone formation flight recorded by `formation_flight.py`.

The existing `archive/Controls/analyze_flight.py` reads the old 49-column single-drone
format and knows nothing about a second vehicle, so this is its multi-drone counterpart.

What it answers -- the two questions the thesis rests on:

  1. What separation did the team actually hold?   (commanded vs achieved)
  2. What interaction force did that produce?      f_res = mass * a_res

`a_res` (`indi.a_res_*`, world frame) is the residual acceleration measured onboard:
a_meas - a_model. In close-proximity flight it is dominated by the other drones' downwash,
and it is the signal the residual-learning methods train on. It is recorded in every
controller mode, so this works for geometric flights as well as INDI ones.

Usage
-----
    python analyze_formation.py                       # newest run in experiments/logs
    python analyze_formation.py <timestamp>           # e.g. 2026-08-25_14-31-02
    python analyze_formation.py <file1.csv> <file2.csv>
    python analyze_formation.py --no-plot
"""

import sys
from pathlib import Path

import numpy as np

LOG_DIR = Path(__file__).resolve().parents[1] / "logs"
DEFAULT_MASS = 0.041  # CF21BL brushless [kg]; overridden by the CSV meta header when present


# ── loading ─────────────────────────────────────────────────────────────────

def read_run(path: Path):
    """Return (name, meta dict, column dict) for one drone CSV."""
    meta, header, rows = {}, None, []
    with open(path) as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("# meta:"):
                k, _, v = line[7:].partition("=")
                meta[k.strip()] = v.strip()
            elif header is None:
                header = line.split(",")
            elif line:
                rows.append([float(x) for x in line.split(",")])
    if header is None or not rows:
        raise ValueError(f"{path.name}: no data")
    arr = np.array(rows)
    cols = {name: arr[:, i] for i, name in enumerate(header)}
    # drone name is the second-to-last underscore field: {label}_{cfname}_{timestamp}.csv
    parts = path.stem.split("_")
    return parts[-3] if len(parts) >= 3 else path.stem, meta, cols


def find_run(args):
    """Resolve CLI args to the CSVs of a single run."""
    files = [Path(a) for a in args if a.endswith(".csv")]
    if files:
        return files
    stamps = [a for a in args if not a.startswith("-")]
    if not LOG_DIR.exists():
        sys.exit(f"[analyze] no log directory at {LOG_DIR}")
    if stamps:
        files = sorted(LOG_DIR.glob(f"*{stamps[0]}*.csv"))
        if not files:
            sys.exit(f"[analyze] no logs matching '{stamps[0]}' in {LOG_DIR}")
        return files
    allcsv = sorted(LOG_DIR.glob("*.csv"))
    if not allcsv:
        sys.exit(f"[analyze] {LOG_DIR} is empty — fly something first")
    stamp = "_".join(allcsv[-1].stem.split("_")[-2:])  # date_time
    return sorted(LOG_DIR.glob(f"*{stamp}*.csv"))


# ── analysis ────────────────────────────────────────────────────────────────

def resample(cols, t_grid):
    """Interpolate every channel onto a common time grid."""
    t = cols["time_s"]
    return {k: np.interp(t_grid, t, v) for k, v in cols.items() if k != "time_s"}


def main():
    args = sys.argv[1:]
    do_plot = "--no-plot" not in args
    files = find_run(args)

    drones = []
    for p in files:
        try:
            drones.append((p,) + read_run(p))
        except Exception as e:
            print(f"[analyze] skipping {p.name}: {e}")
    if not drones:
        sys.exit("[analyze] nothing readable")

    meta = drones[0][2]
    mass = float(meta.get("indi_mass", meta.get("mass", DEFAULT_MASS)))

    print("=" * 72)
    print(f"Formation run — {len(drones)} drone(s)")
    print("=" * 72)
    for k in ("trajectory", "formation", "separation", "laps", "reps",
              "controller", "ctrl_mode", "run_eval_mode"):
        if k in meta:
            print(f"  {k:14s} {meta[k]}")
    print(f"  {'mass':14s} {mass} kg")
    print()

    # Common time grid over the overlapping window, so drones are directly comparable.
    t_lo = max(d[3]["time_s"][0] for d in drones)
    t_hi = min(d[3]["time_s"][-1] for d in drones)
    if t_hi <= t_lo:
        sys.exit("[analyze] logs do not overlap in time")
    grid = np.linspace(t_lo, t_hi, int((t_hi - t_lo) * 100))
    R = [(d[1], resample(d[3], grid)) for d in drones]
    print(f"  overlapping window {t_lo:.2f}–{t_hi:.2f} s ({len(grid)} samples @100 Hz)\n")

    # ── separation ──
    if len(R) > 1:
        print("-" * 72)
        print("SEPARATION (achieved)")
        print("-" * 72)
        for i in range(len(R)):
            for j in range(i + 1, len(R)):
                (na, a), (nb, b) = R[i], R[j]
                d = np.stack([a[f"pos_{k}"] - b[f"pos_{k}"] for k in "xyz"], axis=1)
                dist = np.linalg.norm(d, axis=1)
                tgt = meta.get("separation")
                extra = ""
                if tgt:
                    err = dist - float(tgt)
                    extra = f"   vs target {tgt} m: mean err {err.mean():+.3f} m"
                print(f"  {na} <-> {nb}:  min {dist.min():.3f}  mean {dist.mean():.3f}  "
                      f"max {dist.max():.3f} m{extra}")
                print(f"      per-axis mean |dx| {np.abs(d[:,0]).mean():.3f}  "
                      f"|dy| {np.abs(d[:,1]).mean():.3f}  |dz| {np.abs(d[:,2]).mean():.3f} m")
        print()

    # ── residual force ──
    print("-" * 72)
    print("RESIDUAL FORCE  f_res = mass * a_res   (the downwash signature)")
    print("-" * 72)
    any_res = False
    for name, c in R:
        a = np.stack([c[f"a_res_{k}"] for k in "xyz"], axis=1)
        if not np.any(a):
            print(f"  {name}: all zero — no RPM source, a_res needs rpm.* / motor.*_rpm telemetry")
            continue
        any_res = True
        mag = np.linalg.norm(a, axis=1)
        f = 1000.0 * mass * a  # mN
        print(f"  {name}:")
        print(f"      |a_res|  mean {mag.mean():6.3f}  max {mag.max():6.3f} m/s^2")
        print(f"      f_res    x {f[:,0].mean():+7.1f}   y {f[:,1].mean():+7.1f}   "
              f"z {f[:,2].mean():+7.1f} mN  (mean)")
        print(f"      f_res_z  min {f[:,2].min():+7.1f}  max {f[:,2].max():+7.1f} mN")
    if not any_res:
        print("\n  NOTE: no residual recorded. Check the drone has an RPM source and that")
        print("        firmware with indi.a_res_* is flashed.")
    print()

    # ── the thesis plot: residual vs separation ──
    if len(R) > 1 and any_res:
        (na, a), (nb, b) = R[0], R[1]
        dz = a["pos_z"] - b["pos_z"]
        # Whichever drone sits lower is the one flying in the other's wash.
        if dz.mean() < 0:
            lower_name, lower_c, upper_name = na, a, nb
        else:
            lower_name, lower_c, upper_name = nb, b, na
        sep = np.abs(dz)
        fz = 1000.0 * mass * lower_c["a_res_z"]

        print("-" * 72)
        print(f"DOWNWASH: {lower_name} is below {upper_name} — its vertical residual")
        print("-" * 72)
        print(f"  f_res_z on {lower_name}: mean {fz.mean():+7.1f}  min {fz.min():+7.1f}  "
              f"max {fz.max():+7.1f} mN")
        fz_up = 1000.0 * mass * (b if lower_name == na else a)["a_res_z"]
        print(f"  f_res_z on {upper_name}: mean {fz_up.mean():+7.1f} mN   "
              f"(expect ~0 — it is not in anyone's wash)")
        print(f"  -> downwash effect: {fz.mean() - fz_up.mean():+.1f} mN on the lower drone")

        # Bin against separation only when it actually varies within the run; for a held
        # formation it does not, and the cross-separation trend comes from comparing runs.
        if sep.max() - sep.min() > 0.02:
            print(f"\n  vs vertical separation (varies {sep.min():.2f}–{sep.max():.2f} m):")
            edges = np.linspace(sep.min(), sep.max(), 6)
            for k in range(len(edges) - 1):
                m = (sep >= edges[k]) & (sep < edges[k + 1])
                if m.sum() > 5:
                    print(f"    sep {edges[k]:.2f}–{edges[k+1]:.2f} m   "
                          f"f_res_z mean {fz[m].mean():+7.1f} mN   (n={m.sum()})")
        else:
            print(f"\n  separation held at {sep.mean():.3f} m — to get the force/separation")
            print("  curve, fly the ladder (0.6 → 0.4 → 0.3 → 0.25 → 0.2 m) and compare runs.")
        print()

    if do_plot:
        make_plot(R, grid, mass, meta, files[0])


def make_plot(R, grid, mass, meta, sample_path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[analyze] matplotlib not available — skipping plot")
        return

    n = 3 if len(R) > 1 else 2
    fig, ax = plt.subplots(n, 1, figsize=(11, 3.1 * n), sharex=True)
    for name, c in R:
        ax[0].plot(grid, c["pos_z"], label=name, lw=1.1)
    ax[0].set_ylabel("height [m]")
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=.3)

    for name, c in R:
        f = 1000.0 * mass * c["a_res_z"]
        ax[1].plot(grid, f, label=f"{name}", lw=.9)
    ax[1].set_ylabel("f_res,z [mN]")
    ax[1].axhline(0, color="k", lw=.5)
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=.3)

    if len(R) > 1:
        (na, a), (nb, b) = R[0], R[1]
        d = np.stack([a[f"pos_{k}"] - b[f"pos_{k}"] for k in "xyz"], axis=1)
        ax[2].plot(grid, np.linalg.norm(d, axis=1), color="tab:red", lw=1.1)
        if "separation" in meta:
            ax[2].axhline(float(meta["separation"]), ls="--", color="k", lw=.8,
                          label=f"target {meta['separation']} m")
            ax[2].legend(fontsize=8)
        ax[2].set_ylabel(f"|{na}-{nb}| [m]")
        ax[2].grid(alpha=.3)

    ax[-1].set_xlabel("time [s]")
    fig.suptitle(f"{meta.get('trajectory','?')} · {meta.get('formation','?')} · "
                 f"sep {meta.get('separation','?')} m", fontsize=11)
    fig.tight_layout()
    out = sample_path.parent.parent / "analysis" / f"{sample_path.stem}_formation.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    print(f"[analyze] plot -> {out}")


if __name__ == "__main__":
    main()
