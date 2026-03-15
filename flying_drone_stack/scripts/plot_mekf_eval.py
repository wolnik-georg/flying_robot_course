#!/usr/bin/env python3
"""Run mekf_eval on the latest hover/circle/figure8 flights and plot the results.

Usage (from repo root):
    python3 scripts/plot_mekf_eval.py

Generates PNGs in results/images/mekf_eval_*.png.
"""
import subprocess, sys, io
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parent.parent
RUNS = REPO / "runs"
OUT  = REPO / "results" / "images"
OUT.mkdir(parents=True, exist_ok=True)

MANEUVERS = ["hover", "circle", "figure8"]


def latest_csv(maneuver: str) -> Path | None:
    """Return the most recently modified run CSV for the given maneuver."""
    candidates = sorted(RUNS.glob(f"{maneuver}_*.csv"), key=lambda p: p.stat().st_mtime)
    return candidates[-1] if candidates else None


def run_mekf_eval(csv_path: Path) -> tuple[np.ndarray, str]:
    """Run mekf_eval binary and return (data array, stderr text)."""
    result = subprocess.run(
        ["cargo", "run", "--bin", "mekf_eval", "--", str(csv_path)],
        capture_output=True, text=True, cwd=REPO,
    )
    if result.returncode != 0:
        print(f"mekf_eval failed for {csv_path.name}:\n{result.stderr}", file=sys.stderr)
        return None, result.stderr

    data = np.genfromtxt(io.StringIO(result.stdout), delimiter=",", names=True)
    return data, result.stderr


def plot_maneuver(maneuver: str, data: np.ndarray, rmse_text: str):
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(f"MEKF vs Firmware EKF — {maneuver}\n{rmse_text.strip()}", fontsize=10)

    # ── Position: X, Y, Z vs time ────────────────────────────────────────────
    ax_x = fig.add_subplot(3, 3, 1)
    ax_x.plot(data["time_s"], data["ref_x"],   label="fw EKF", color="C0")
    ax_x.plot(data["time_s"], data["mekf_x"], label="MEKF",   color="C1", ls="--")
    ax_x.set_ylabel("x [m]"); ax_x.set_title("Position X"); ax_x.legend(fontsize=7)

    ax_y = fig.add_subplot(3, 3, 2)
    ax_y.plot(data["time_s"], data["ref_y"],   color="C0")
    ax_y.plot(data["time_s"], data["mekf_y"], color="C1", ls="--")
    ax_y.set_ylabel("y [m]"); ax_y.set_title("Position Y")

    ax_z = fig.add_subplot(3, 3, 3)
    ax_z.plot(data["time_s"], data["ref_z"],   color="C0")
    ax_z.plot(data["time_s"], data["mekf_z"], color="C1", ls="--")
    ax_z.set_ylabel("z [m]"); ax_z.set_title("Height Z")

    # ── Attitude: Roll, Pitch, Yaw vs time ───────────────────────────────────
    ax_r = fig.add_subplot(3, 3, 4)
    ax_r.plot(data["time_s"], data["ref_roll"],   color="C0")
    ax_r.plot(data["time_s"], data["mekf_roll"], color="C1", ls="--")
    ax_r.set_ylabel("roll [°]"); ax_r.set_title("Roll")

    ax_p = fig.add_subplot(3, 3, 5)
    ax_p.plot(data["time_s"], data["ref_pitch"],   color="C0")
    ax_p.plot(data["time_s"], data["mekf_pitch"], color="C1", ls="--")
    ax_p.set_ylabel("pitch [°]"); ax_p.set_title("Pitch")

    ax_yw = fig.add_subplot(3, 3, 6)
    ax_yw.plot(data["time_s"], data["ref_yaw"],   color="C0")
    ax_yw.plot(data["time_s"], data["mekf_yaw"], color="C1", ls="--")
    ax_yw.set_ylabel("yaw [°]"); ax_yw.set_title("Yaw")

    # ── XY top-down trajectory ────────────────────────────────────────────────
    ax_xy = fig.add_subplot(3, 3, 7, aspect="equal")
    ax_xy.plot(data["ref_x"],   data["ref_y"],   label="fw EKF", color="C0")
    ax_xy.plot(data["mekf_x"], data["mekf_y"], label="MEKF",   color="C1", ls="--")
    ax_xy.set_xlabel("x [m]"); ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("XY trajectory (top-down)"); ax_xy.legend(fontsize=7)

    # ── Position error norm ───────────────────────────────────────────────────
    ex = data["mekf_x"] - data["ref_x"]
    ey = data["mekf_y"] - data["ref_y"]
    ez = data["mekf_z"] - data["ref_z"]
    err3d = np.sqrt(ex**2 + ey**2 + ez**2)
    ax_err = fig.add_subplot(3, 3, 8)
    ax_err.plot(data["time_s"], err3d, color="C2")
    ax_err.axhline(err3d.mean(), color="C2", ls=":", lw=0.8, label=f"mean {err3d.mean():.3f} m")
    ax_err.set_ylabel("3D error [m]"); ax_err.set_title("Position error vs time")
    ax_err.legend(fontsize=7)

    # ── Attitude error (yaw) ──────────────────────────────────────────────────
    dyaw = data["mekf_yaw"] - data["ref_yaw"]
    dyaw = (dyaw + 180) % 360 - 180  # wrap to [-180, 180]
    ax_yerr = fig.add_subplot(3, 3, 9)
    ax_yerr.plot(data["time_s"], dyaw, color="C3")
    ax_yerr.set_ylabel("yaw error [°]"); ax_yerr.set_title("Yaw error vs time")

    for ax in fig.get_axes():
        ax.set_xlabel("t [s]" if ax not in (ax_xy,) else ax.get_xlabel())
        ax.grid(True, lw=0.4)

    fig.tight_layout()
    out_path = OUT / f"mekf_eval_{maneuver}.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Saved: {out_path}")


def extract_rmse(stderr: str) -> str:
    """Pull the === RMSE ... === block out of stderr."""
    lines = stderr.splitlines()
    out, in_block = [], False
    for l in lines:
        if "RMSE" in l:
            in_block = True
        if in_block:
            out.append(l)
            if l.strip() == "" and len(out) > 2:
                break
    return "\n".join(out)


def main():
    print("Building mekf_eval...")
    r = subprocess.run(["cargo", "build", "--bin", "mekf_eval"], cwd=REPO,
                       capture_output=True, text=True)
    if r.returncode != 0:
        print("Build failed:\n", r.stderr); sys.exit(1)
    print("Build OK.\n")

    for maneuver in MANEUVERS:
        csv = latest_csv(maneuver)
        if csv is None:
            print(f"No {maneuver} CSV found in {RUNS}, skipping.")
            continue

        print(f"[{maneuver}] using {csv.name}")
        data, stderr = run_mekf_eval(csv)
        if data is None:
            continue

        rmse = extract_rmse(stderr)
        print(rmse)
        plot_maneuver(maneuver, data, rmse)
        print()


if __name__ == "__main__":
    main()
