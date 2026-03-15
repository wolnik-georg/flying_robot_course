#!/usr/bin/env python3
"""plot_shadow_eval.py – Shadow controller evaluation plots.

Loads a flight CSV that contains the 37-column format written by main.rs and
compares the shadow controller outputs (our_roll_cmd, our_pitch_cmd,
our_thrust, our_ref_x/y/z) against the actual firmware state.

Usage
-----
    # Most-recent CSV in runs/:
    python scripts/plot_shadow_eval.py

    # Specific file:
    python scripts/plot_shadow_eval.py runs/circle_2026-03-01_13-59-43.csv

Requirements: numpy, matplotlib, pandas (all in the flying_robots pyenv).
"""

import sys
import os
import glob
import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ── paths ────────────────────────────────────────────────────────────────────
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RUNS_DIR = os.path.join(ROOT, "runs")
IMG_DIR = os.path.join(ROOT, "results", "images")
os.makedirs(IMG_DIR, exist_ok=True)

# ── helpers ──────────────────────────────────────────────────────────────────
SHADOW_COLS = [
    "our_roll_cmd",
    "our_pitch_cmd",
    "our_thrust",
    "our_ref_x",
    "our_ref_y",
    "our_ref_z",
]


def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df["time_s"] = df["time_ms"] / 1000.0
    return df


def has_shadow_data(df: pd.DataFrame) -> bool:
    """Return True if the shadow columns are present and contain non-zero values."""
    for col in SHADOW_COLS:
        if col not in df.columns:
            return False
    # Columns present but all-zero means the main binary ran without shadow active
    if df["our_roll_cmd"].abs().max() < 1e-6 and df["our_pitch_cmd"].abs().max() < 1e-6:
        return False
    return True


def shadow_status(df: pd.DataFrame) -> str:
    """Return a human-readable status string for the shadow columns."""
    missing = [c for c in SHADOW_COLS if c not in df.columns]
    if missing:
        return f"missing columns: {missing}"
    if df["our_roll_cmd"].abs().max() < 1e-6 and df["our_pitch_cmd"].abs().max() < 1e-6:
        return "columns present but all-zero (main binary flew without shadow active)"
    return "ok"


def airborne_mask(df: pd.DataFrame) -> "np.ndarray":
    """Return boolean mask for rows where the drone is airborne (range_z > 0.05 m)."""
    if "range_z" in df.columns:
        return df["range_z"].values > 0.05
    # Fallback: use firmware z position
    if "pos_z" in df.columns:
        return df["pos_z"].values > 0.05
    return np.ones(len(df), dtype=bool)


def rmse(a, b):
    diff = np.asarray(a) - np.asarray(b)
    return float(np.sqrt(np.mean(diff**2)))


# ── plotting ─────────────────────────────────────────────────────────────────
def plot_attitude_tracking(df: pd.DataFrame, stem: str):
    """Roll / pitch: shadow cmd vs firmware attitude."""
    mask = airborne_mask(df)
    t = df["time_s"].values[mask]

    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    fig.suptitle(f"Shadow attitude tracking — {stem}", fontsize=13)

    for ax, fw_col, cmd_col, label in [
        (axes[0], "roll", "our_roll_cmd", "Roll"),
        (axes[1], "pitch", "our_pitch_cmd", "Pitch"),
    ]:
        fw = df[fw_col].values[mask]
        cmd = df[cmd_col].values[mask]
        err = rmse(cmd, fw)
        ax.plot(t, fw, label=f"Firmware {label.lower()} (measured)", linewidth=1.2)
        ax.plot(
            t, cmd, label=f"Shadow {label.lower()} cmd", linewidth=1.0, linestyle="--"
        )
        ax.set_ylabel(f"{label} [°]")
        ax.legend(loc="upper right", fontsize=8)
        ax.set_title(f"RMSE = {err:.2f}°")
        ax.grid(True, alpha=0.4)

    axes[-1].set_xlabel("Time [s]")
    fig.tight_layout()
    out = os.path.join(IMG_DIR, f"shadow_attitude_{stem}.png")
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f"  Saved {out}")
    return {
        "roll_rmse_deg": rmse(df["our_roll_cmd"].values[mask], df["roll"].values[mask]),
        "pitch_rmse_deg": rmse(
            df["our_pitch_cmd"].values[mask], df["pitch"].values[mask]
        ),
    }


def plot_thrust_tracking(df: pd.DataFrame, stem: str):
    """Thrust: shadow estimate vs firmware thrust column (if available)."""
    mask = airborne_mask(df)
    t = df["time_s"].values[mask]
    shadow_thrust = df["our_thrust"].values[mask]

    fig, ax = plt.subplots(figsize=(12, 4))
    fig.suptitle(f"Shadow thrust estimate — {stem}", fontsize=13)

    ax.plot(t, shadow_thrust, label="Shadow thrust [N]", linewidth=1.2)
    if "thrust" in df.columns:
        fw_thrust = df["thrust"].values[mask]
        ax.plot(
            t,
            fw_thrust,
            label="Firmware thrust (raw)",
            linewidth=1.0,
            linestyle="--",
            alpha=0.7,
        )
        ax.set_title(f"RMSE = {rmse(shadow_thrust, fw_thrust):.4f} N")

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Thrust [N]")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.4)
    fig.tight_layout()
    out = os.path.join(IMG_DIR, f"shadow_thrust_{stem}.png")
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f"  Saved {out}")


def plot_trajectory_tracking(df: pd.DataFrame, stem: str):
    """XY and Z: shadow reference vs firmware position."""
    mask = airborne_mask(df)
    t = df["time_s"].values[mask]

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(f"Shadow trajectory tracking — {stem}", fontsize=13)

    for ax, pos_col, ref_col, label in [
        (axes[0], "pos_x", "our_ref_x", "X"),
        (axes[1], "pos_y", "our_ref_y", "Y"),
        (axes[2], "pos_z", "our_ref_z", "Z"),
    ]:
        fw = df[pos_col].values[mask]
        ref = df[ref_col].values[mask]
        err = rmse(fw, ref)
        ax.plot(t, fw, label=f"Firmware {label.lower()} (EKF)", linewidth=1.2)
        ax.plot(
            t, ref, label=f"Shadow ref {label.lower()}", linewidth=1.0, linestyle="--"
        )
        ax.set_ylabel(f"{label} [m]")
        ax.legend(loc="upper right", fontsize=8)
        ax.set_title(f"RMSE = {err*100:.1f} cm")
        ax.grid(True, alpha=0.4)

    axes[-1].set_xlabel("Time [s]")
    fig.tight_layout()
    out = os.path.join(IMG_DIR, f"shadow_traj_{stem}.png")
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f"  Saved {out}")

    # XY top-down
    fig2, ax2 = plt.subplots(figsize=(6, 6))
    ax2.plot(
        df["pos_x"].values[mask],
        df["pos_y"].values[mask],
        label="Firmware EKF",
        linewidth=1.2,
    )
    ax2.plot(
        df["our_ref_x"].values[mask],
        df["our_ref_y"].values[mask],
        label="Shadow reference",
        linewidth=1.0,
        linestyle="--",
    )
    ax2.set_xlabel("X [m]")
    ax2.set_ylabel("Y [m]")
    ax2.set_title(f"XY top-down — {stem}")
    ax2.legend()
    ax2.set_aspect("equal")
    ax2.grid(True, alpha=0.4)
    fig2.tight_layout()
    out2 = os.path.join(IMG_DIR, f"shadow_xy_{stem}.png")
    fig2.savefig(out2, dpi=120)
    plt.close(fig2)
    print(f"  Saved {out2}")


# ── main ─────────────────────────────────────────────────────────────────────
def main():
    if len(sys.argv) >= 2:
        csv_path = sys.argv[1]
        if not os.path.isabs(csv_path):
            csv_path = os.path.join(ROOT, csv_path)
    else:
        # Find most-recent CSV in runs/
        csvs = sorted(glob.glob(os.path.join(RUNS_DIR, "*.csv")))
        if not csvs:
            print(f"No CSV files found in {RUNS_DIR}")
            sys.exit(1)
        csv_path = csvs[-1]

    print(f"Loading: {csv_path}")
    df = load_csv(csv_path)
    stem = os.path.splitext(os.path.basename(csv_path))[0]

    print(f"  Rows: {len(df)}, Columns: {list(df.columns)}")

    if not has_shadow_data(df):
        status = shadow_status(df)
        print(f"\n⚠  Shadow data not usable ({status}).")
        print("   To get full shadow evaluation:")
        print("   1. cd flying_drone_stack && cargo build --release --bin main")
        print('   2. Fly with MANEUVER="circle" (or figure8, hover)')
        print("   3. Re-run this script on the resulting CSV in runs/")
        print("   Plotting flight overview from available columns...\n")
        _plot_raw_attitude(df, stem)
        return

    print("\nShadow data detected — generating evaluation plots...")
    metrics = plot_attitude_tracking(df, stem)
    plot_thrust_tracking(df, stem)
    plot_trajectory_tracking(df, stem)

    print("\n── Summary ────────────────────────────────────")
    for k, v in metrics.items():
        unit = "°" if "deg" in k else ""
        status = "✓" if v < 5.0 else "✗"
        print(f"  {k:25s}: {v:.2f}{unit}  {status}")
    if metrics["roll_rmse_deg"] < 5.0 and metrics["pitch_rmse_deg"] < 5.0:
        print("\n✅  Shadow controller tracking within 5° threshold.")
    else:
        print("\n❌  Tracking error exceeds 5° — check gains / HOVER_PWM.")


def _plot_raw_attitude(df: pd.DataFrame, stem: str):
    """Richer fallback plot when shadow data is unavailable — 2x2 grid."""
    mask = airborne_mask(df)
    t = df["time_s"].values[mask]

    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    fig.suptitle(f"Flight overview (no shadow data) — {stem}", fontsize=12)

    # Roll / pitch
    ax = axes[0, 0]
    for col, label, color in [("roll", "Roll", "blue"), ("pitch", "Pitch", "red")]:
        if col in df.columns:
            ax.plot(t, df[col].values[mask], color=color, lw=0.9, label=f"fw {label}")
        mekf_col = f"mekf_{col}"
        if mekf_col in df.columns:
            ax.plot(
                t,
                df[mekf_col].values[mask],
                color=color,
                lw=0.9,
                linestyle="--",
                alpha=0.6,
                label=f"mekf {label}",
            )
    ax.set_ylabel("angle [°]")
    ax.set_xlabel("t [s]")
    ax.set_title("Roll / pitch")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Height
    ax = axes[0, 1]
    if "range_z" in df.columns:
        ax.plot(t, df["range_z"].values[mask], "k-", lw=0.9, label="range_z (ToF)")
    if "pos_z" in df.columns:
        ax.plot(t, df["pos_z"].values[mask], "b--", lw=0.9, alpha=0.7, label="fw EKF z")
    if "mekf_z" in df.columns:
        ax.plot(t, df["mekf_z"].values[mask], "g--", lw=0.9, alpha=0.7, label="mekf z")
    ax.set_ylabel("height [m]")
    ax.set_xlabel("t [s]")
    ax.set_title("Height")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # XY trajectory
    ax = axes[1, 0]
    if "pos_x" in df.columns and "pos_y" in df.columns:
        ax.plot(
            df["pos_x"].values[mask],
            df["pos_y"].values[mask],
            "b-",
            lw=0.9,
            alpha=0.8,
            label="fw EKF",
        )
        ax.plot(df["pos_x"].values[mask][0], df["pos_y"].values[mask][0], "bs", ms=5)
    if "mekf_x" in df.columns and "mekf_y" in df.columns:
        ax.plot(
            df["mekf_x"].values[mask],
            df["mekf_y"].values[mask],
            "g-",
            lw=0.9,
            alpha=0.8,
            label="MEKF",
        )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("XY trajectory")
    ax.legend(fontsize=7)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # Optical flow
    ax = axes[1, 1]
    if "flow_dx" in df.columns:
        ax.plot(t, df["flow_dx"].values[mask], "b-", lw=0.6, alpha=0.8, label="flow_dx")
    if "flow_dy" in df.columns:
        ax.plot(t, df["flow_dy"].values[mask], "r-", lw=0.6, alpha=0.8, label="flow_dy")
    ax.set_ylabel("flow [px/frame]")
    ax.set_xlabel("t [s]")
    ax.set_title("Optical flow")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    out = os.path.join(IMG_DIR, f"shadow_overview_{stem}.png")
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"  Saved {out}")


if __name__ == "__main__":
    main()
