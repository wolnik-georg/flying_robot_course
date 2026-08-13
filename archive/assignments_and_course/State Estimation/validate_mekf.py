#!/usr/bin/env python3
"""
validate_mekf.py — Step 1.5: Validate the in-flight MEKF against firmware EKF

Loads the three flight CSVs produced by run_firmware_mode (hover, circle, figure-8)
and compares the MEKF columns (logged as a passenger) against the on-board firmware EKF.

Figures produced per log (saved as PNG next to this script):
  <maneuver>_01_orientation.png   roll / pitch / yaw  — MEKF vs firmware
  <maneuver>_02_position_z.png    mekf_z vs pos_z vs range_z (height cross-check)
  <maneuver>_03_position_xy.png   mekf_x/y vs pos_x/y time-series
  <maneuver>_04_trajectory_xy.png XY ground-track: MEKF vs firmware

A summary RMS table is printed at the end.

Usage:
    python3 validate_mekf.py [--log-dir PATH]

Default log dir: ../flying_drone_stack/runs/
Picks the most-recent hover / circle / figure8 log automatically.
"""

import os
import sys
import glob
import argparse
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ── helpers ──────────────────────────────────────────────────────────────────

HERE = os.path.dirname(os.path.abspath(__file__))
DEF_LOG_DIR = os.path.join(HERE, "..", "flying_drone_stack", "runs")


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument(
        "--log-dir", default=DEF_LOG_DIR, help="Directory containing the flight CSVs"
    )
    return p.parse_args()


def latest_log(log_dir: str, prefix: str) -> str:
    """Return the most-recently-modified CSV whose name starts with prefix."""
    pattern = os.path.join(log_dir, f"{prefix}_*.csv")
    files = sorted(glob.glob(pattern), key=os.path.getmtime)
    if not files:
        raise FileNotFoundError(f"No files matching {pattern}")
    return files[-1]


def load_csv(path: str) -> dict:
    """Load CSV into a dict of column-name → np.ndarray (float64)."""
    with open(path) as f:
        header = f.readline().strip().split(",")
        rows = []
        for line in f:
            vals = line.strip().split(",")
            if len(vals) == len(header):
                rows.append([float(v) for v in vals])
    data = np.array(rows)
    return {h: data[:, i] for i, h in enumerate(header)}


def rms(a, b):
    """RMS of element-wise difference (arrays must be same length)."""
    return float(np.sqrt(np.mean((a - b) ** 2)))


def airborne_mask(d: dict, rz_thresh: float = 0.10) -> np.ndarray:
    """Boolean mask: rows where the drone is airborne (range_z > threshold)."""
    return d["range_z"] > rz_thresh


# ── plotting helpers ─────────────────────────────────────────────────────────

COLORS = {
    "mekf": "#2196F3",  # blue
    "fw": "#F44336",  # red
    "range_z": "#4CAF50",  # green
}


def save(fig, path: str):
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  saved → {os.path.basename(path)}")


def plot_orientation(d: dict, mask: np.ndarray, title_prefix: str, out_path: str):
    t = d["time_ms"][mask] / 1000.0
    fig, axes = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
    fig.suptitle(f"{title_prefix} — Orientation: MEKF vs firmware EKF", fontsize=12)

    pairs = [
        ("Roll  [deg]", "mekf_roll", "roll"),
        ("Pitch [deg]", "mekf_pitch", "pitch"),
        ("Yaw   [deg]", "mekf_yaw", "yaw"),
    ]
    for ax, (ylabel, mk, fw) in zip(axes, pairs):
        ax.plot(
            t,
            d[mk][mask],
            color=COLORS["mekf"],
            lw=0.9,
            alpha=0.85,
            label="MEKF (Rust)",
        )
        ax.plot(
            t, d[fw][mask], color=COLORS["fw"], lw=1.1, alpha=0.85, label="Firmware EKF"
        )
        r = rms(d[mk][mask], d[fw][mask])
        ax.set_ylabel(ylabel, fontsize=9)
        ax.set_title(f"RMS error = {r:.2f}°", fontsize=8, loc="right")
        ax.legend(loc="upper left", fontsize=8)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time [s]")
    save(fig, out_path)


def plot_height(d: dict, mask: np.ndarray, title_prefix: str, out_path: str):
    t = d["time_ms"][mask] / 1000.0
    fig, axes = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
    fig.suptitle(f"{title_prefix} — Height / Z estimate", fontsize=12)

    # Top: all three z sources together
    ax = axes[0]
    ax.plot(
        t,
        d["mekf_z"][mask],
        color=COLORS["mekf"],
        lw=0.9,
        alpha=0.9,
        label="MEKF z [m]",
    )
    ax.plot(
        t,
        d["pos_z"][mask],
        color=COLORS["fw"],
        lw=1.0,
        alpha=0.9,
        label="Firmware pos_z [m]",
    )
    ax.plot(
        t,
        d["range_z"][mask],
        color=COLORS["range_z"],
        lw=0.8,
        alpha=0.8,
        label="ToF range_z [m]",
    )
    ax.set_ylabel("Height [m]")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)

    # Middle: MEKF z vs range_z (direct ToF comparison)
    ax = axes[1]
    ax.plot(t, d["mekf_z"][mask], color=COLORS["mekf"], lw=0.9, label="MEKF z")
    ax.plot(
        t, d["range_z"][mask], color=COLORS["range_z"], lw=1.0, label="range_z (ToF)"
    )
    r = rms(d["mekf_z"][mask], d["range_z"][mask])
    ax.set_ylabel("Height [m]")
    ax.set_title(f"MEKF vs ToF  RMS = {r*100:.1f} cm", fontsize=8, loc="right")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)

    # Bottom: firmware pos_z vs range_z
    ax = axes[2]
    ax.plot(t, d["pos_z"][mask], color=COLORS["fw"], lw=0.9, label="firmware pos_z")
    ax.plot(
        t, d["range_z"][mask], color=COLORS["range_z"], lw=1.0, label="range_z (ToF)"
    )
    r2 = rms(d["pos_z"][mask], d["range_z"][mask])
    ax.set_ylabel("Height [m]")
    ax.set_title(f"FW EKF vs ToF  RMS = {r2*100:.1f} cm", fontsize=8, loc="right")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("Time [s]")

    save(fig, out_path)


def plot_position_xy(d: dict, mask: np.ndarray, title_prefix: str, out_path: str):
    t = d["time_ms"][mask] / 1000.0
    fig, axes = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    fig.suptitle(
        f"{title_prefix} — Horizontal position: MEKF vs firmware EKF", fontsize=12
    )

    for ax, (ylabel, mk, fw) in zip(
        axes,
        [
            ("X [m]", "mekf_x", "pos_x"),
            ("Y [m]", "mekf_y", "pos_y"),
        ],
    ):
        ax.plot(t, d[mk][mask], color=COLORS["mekf"], lw=0.9, alpha=0.85, label="MEKF")
        ax.plot(
            t, d[fw][mask], color=COLORS["fw"], lw=1.0, alpha=0.85, label="Firmware EKF"
        )
        r = rms(d[mk][mask], d[fw][mask])
        ax.set_ylabel(ylabel)
        ax.set_title(f"RMS error = {r*100:.1f} cm", fontsize=8, loc="right")
        ax.legend(loc="upper left", fontsize=8)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time [s]")
    save(fig, out_path)


def plot_trajectory(d: dict, mask: np.ndarray, title_prefix: str, out_path: str):
    fig, axes = plt.subplots(1, 2, figsize=(13, 6))
    fig.suptitle(f"{title_prefix} — XY Ground Track", fontsize=12)

    t_all = d["time_ms"][mask] / 1000.0
    t0 = t_all[0]

    for ax, (label, xk, yk, c) in zip(
        axes,
        [
            ("MEKF", "mekf_x", "mekf_y", COLORS["mekf"]),
            ("Firmware EKF", "pos_x", "pos_y", COLORS["fw"]),
        ],
    ):
        x = d[xk][mask]
        y = d[yk][mask]
        # colour-code by time so we can see the trajectory direction
        sc = ax.scatter(x, y, c=t_all, cmap="plasma", s=4, zorder=2)
        ax.plot(x, y, color=c, lw=0.5, alpha=0.4, zorder=1)
        ax.plot(x[0], y[0], "go", ms=7, label="start", zorder=3)
        ax.plot(x[-1], y[-1], "rs", ms=7, label="end", zorder=3)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_title(label)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        fig.colorbar(sc, ax=ax, label="Time [s]", fraction=0.046, pad=0.04)

    save(fig, out_path)


# ── main ─────────────────────────────────────────────────────────────────────


def process_log(log_path: str, out_dir: str) -> dict:
    """Load one CSV, produce all four figures, return RMS dict."""
    name = os.path.splitext(os.path.basename(log_path))[0]
    # e.g. "circle_2026-03-15_11-46-16" → prefix "circle"
    maneuver = name.split("_")[0]
    label = f"{maneuver.capitalize()} ({name[-19:]})"

    print(f"\n{'='*60}")
    print(f"  {label}")
    print(f"  {log_path}")
    print(f"{'='*60}")

    d = load_csv(log_path)
    rows = len(d["time_ms"])
    print(f"  {rows} rows, duration {d['time_ms'][-1]/1000:.1f} s")

    mask = airborne_mask(d, rz_thresh=0.10)
    n_air = mask.sum()
    print(
        f"  {n_air} airborne rows (range_z > 10 cm), "
        f"t = {d['time_ms'][mask][0]/1000:.1f}–{d['time_ms'][mask][-1]/1000:.1f} s"
    )

    if n_air < 10:
        print("  WARNING: too few airborne rows — skipping this log")
        return {}

    # ── Check MEKF seeded ──────────────────────────────────────────────────
    mekf_active = np.any(d["mekf_z"][mask] > 0.01)
    if not mekf_active:
        print("  WARNING: MEKF never seeds in this log — skipping")
        return {}

    seed_row = np.argmax(d["mekf_z"] > 0.01)
    print(
        f"  MEKF seeds at t = {d['time_ms'][seed_row]/1000:.2f} s  "
        f"(z={d['mekf_z'][seed_row]:.3f} m, "
        f"roll={d['mekf_roll'][seed_row]:.1f}°, "
        f"pitch={d['mekf_pitch'][seed_row]:.1f}°, "
        f"yaw={d['mekf_yaw'][seed_row]:.1f}°)"
    )

    # ── Produce figures ────────────────────────────────────────────────────
    prefix = f"{maneuver}_{name[-19:]}"  # e.g. circle_2026-03-15_11-46-16
    plot_orientation(
        d, mask, label, os.path.join(out_dir, f"{prefix}_01_orientation.png")
    )
    plot_height(d, mask, label, os.path.join(out_dir, f"{prefix}_02_height.png"))
    plot_position_xy(
        d, mask, label, os.path.join(out_dir, f"{prefix}_03_position_xy.png")
    )
    plot_trajectory(
        d, mask, label, os.path.join(out_dir, f"{prefix}_04_trajectory.png")
    )

    # ── RMS table ─────────────────────────────────────────────────────────
    m = mask
    results = {
        "maneuver": maneuver,
        "n_air": int(n_air),
        "rms_roll": rms(d["mekf_roll"][m], d["roll"][m]),
        "rms_pitch": rms(d["mekf_pitch"][m], d["pitch"][m]),
        "rms_yaw": rms(d["mekf_yaw"][m], d["yaw"][m]),
        "rms_x_cm": rms(d["mekf_x"][m], d["pos_x"][m]) * 100,
        "rms_y_cm": rms(d["mekf_y"][m], d["pos_y"][m]) * 100,
        "rms_z_cm": rms(d["mekf_z"][m], d["range_z"][m]) * 100,
        "rms_z_fw_cm": rms(d["pos_z"][m], d["range_z"][m]) * 100,
    }

    print(f"\n  RMS errors (airborne rows only):")
    print(
        f"    Orientation:  roll {results['rms_roll']:.2f}°  "
        f"pitch {results['rms_pitch']:.2f}°  yaw {results['rms_yaw']:.2f}°"
    )
    print(
        f"    Position:     x {results['rms_x_cm']:.1f} cm  "
        f"y {results['rms_y_cm']:.1f} cm  z {results['rms_z_cm']:.1f} cm  "
        f"(vs ToF)"
    )
    print(
        f"    FW EKF vs ToF: z {results['rms_z_fw_cm']:.1f} cm  "
        f"(reference — how well firmware EKF tracks ToF)"
    )

    return results


def print_summary(all_results: list):
    print(f"\n{'='*70}")
    print("  SUMMARY TABLE")
    print(f"{'='*70}")
    print(
        f"  {'Maneuver':<12}  {'roll[°]':>8}  {'pitch[°]':>8}  {'yaw[°]':>8}  "
        f"{'x[cm]':>8}  {'y[cm]':>8}  {'z[cm]':>8}  {'fw_z[cm]':>9}"
    )
    print(f"  {'-'*70}")
    for r in all_results:
        if not r:
            continue
        print(
            f"  {r['maneuver']:<12}  "
            f"{r['rms_roll']:>8.2f}  "
            f"{r['rms_pitch']:>8.2f}  "
            f"{r['rms_yaw']:>8.2f}  "
            f"{r['rms_x_cm']:>8.1f}  "
            f"{r['rms_y_cm']:>8.1f}  "
            f"{r['rms_z_cm']:>8.1f}  "
            f"{r['rms_z_fw_cm']:>9.1f}"
        )
    print(
        f"\n  Columns: MEKF vs firmware EKF (orientation, XY); "
        f"MEKF and FW EKF vs ToF range_z (height).\n"
    )


def main():
    args = parse_args()
    log_dir = os.path.realpath(args.log_dir)
    out_dir = HERE

    if not os.path.isdir(log_dir):
        print(f"ERROR: log directory not found: {log_dir}", file=sys.stderr)
        sys.exit(1)

    print(f"Log directory : {log_dir}")
    print(f"Output figures: {out_dir}")

    all_results = []
    for maneuver in ["hover", "circle", "figure8"]:
        try:
            path = latest_log(log_dir, maneuver)
            result = process_log(path, out_dir)
            all_results.append(result)
        except FileNotFoundError as e:
            print(f"\nSkipping {maneuver}: {e}")

    print_summary(all_results)
    print("Done.")


if __name__ == "__main__":
    main()
