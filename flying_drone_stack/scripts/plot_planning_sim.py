#!/usr/bin/env python3
"""
Planning Mode Simulation — Visualisation
=========================================
Reads closed-loop CSVs from results/planning_sim/closed_loop/:
  mode{0,1}/<circle|figure8|helix|loop>/{geo,indi}.csv
  mode2/<circle|figure8|helix|loop|flip>/{geo,indi}.csv
  mode3/<circle|figure8|helix|corner|loop>/{geo,indi}.csv
  mode4/<circle|figure8|helix|corner|loop>/{geo,indi}.csv

Each trajectory is run with GeometricController (geo) and IndiController (indi).

Produces PNG plots in results/planning_sim/closed_loop/images/:
  fig1_3d_trajectories.png   — 3-D reference vs simulated paths (3 rows: M0 / M1 / M2)
                               geo=solid, indi=dotted (same colour)
  fig2_position_errors.png   — position-component tracking error over time
  fig3_attitude.png          — ref vs sim roll/pitch/yaw  (Mode 2 shows inversion)
  fig4_thrust_mode01.png     — commanded thrust for Modes 0 and 1
  fig4b_thrust_mode2.png     — Mode 2 thrust profiles showing negative-thrust regions
  fig5_rms_summary.png       — grouped bar chart: geo vs indi RMS per trajectory/mode
  fig6_cross_mode.png        — all applicable modes overlaid per trajectory shape

Run:
  cargo run --release --bin planning_sim
  ~/.pyenv/versions/flying_robots/bin/python scripts/plot_planning_sim.py

Dependencies (numpy, matplotlib) live in the flying_robots pyenv; install if needed:
  ~/.pyenv/versions/flying_robots/bin/pip install -r requirements-planning-plots.txt
"""

import os
import sys
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.patches import Patch

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(_ROOT, "results", "planning_sim")
IMG_DIR = os.path.join(DATA_DIR, "closed_loop", "images")
os.makedirs(IMG_DIR, exist_ok=True)


def _cl(mode: int, traj: str, kind: str) -> str:
    """Relative path under DATA_DIR for a closed-loop run CSV."""
    return os.path.join("closed_loop", f"mode{mode}", traj, f"{kind}.csv")

# ---------------------------------------------------------------------------
# Load all runs
# ---------------------------------------------------------------------------

# key → (csv_relpath, mode_label, traj_label, ctrl_label, color)
RUNS = {
    "m0_circle_geo":  (_cl(0, "circle", "geo"),  "Mode 0", "circle",  "Geo",  "#1f77b4"),
    "m0_circle_indi": (_cl(0, "circle", "indi"),  "Mode 0", "circle",  "INDI", "#1f77b4"),
    "m0_fig8_geo":    (_cl(0, "figure8", "geo"),  "Mode 0", "fig8",   "Geo",  "#ff7f0e"),
    "m0_fig8_indi":   (_cl(0, "figure8", "indi"), "Mode 0", "fig8",   "INDI", "#ff7f0e"),
    "m0_helix_geo":   (_cl(0, "helix", "geo"),    "Mode 0", "helix",  "Geo",  "#2ca02c"),
    "m0_helix_indi":  (_cl(0, "helix", "indi"),   "Mode 0", "helix",  "INDI", "#2ca02c"),
    "m0_corner_geo":  (_cl(0, "corner", "geo"),   "Mode 0", "corner", "Geo",  "#17becf"),
    "m0_corner_indi": (_cl(0, "corner", "indi"),  "Mode 0", "corner", "INDI", "#17becf"),
    "m0_loop_geo":    (_cl(0, "loop", "geo"),     "Mode 0", "loop",   "Geo",  "#d62728"),
    "m0_loop_indi":   (_cl(0, "loop", "indi"),    "Mode 0", "loop",   "INDI", "#d62728"),
    "m1_circle_geo":  (_cl(1, "circle", "geo"),   "Mode 1", "circle", "Geo",  "#1f77b4"),
    "m1_circle_indi": (_cl(1, "circle", "indi"),  "Mode 1", "circle", "INDI", "#1f77b4"),
    "m1_fig8_geo":    (_cl(1, "figure8", "geo"),   "Mode 1", "fig8",   "Geo",  "#ff7f0e"),
    "m1_fig8_indi":   (_cl(1, "figure8", "indi"),  "Mode 1", "fig8",   "INDI", "#ff7f0e"),
    "m1_helix_geo":   (_cl(1, "helix", "geo"),     "Mode 1", "helix",  "Geo",  "#2ca02c"),
    "m1_helix_indi":  (_cl(1, "helix", "indi"),    "Mode 1", "helix",  "INDI", "#2ca02c"),
    "m1_corner_geo":  (_cl(1, "corner", "geo"),    "Mode 1", "corner", "Geo",  "#17becf"),
    "m1_corner_indi": (_cl(1, "corner", "indi"),   "Mode 1", "corner", "INDI", "#17becf"),
    "m1_loop_geo":    (_cl(1, "loop", "geo"),      "Mode 1", "loop",   "Geo",  "#d62728"),
    "m1_loop_indi":   (_cl(1, "loop", "indi"),     "Mode 1", "loop",   "INDI", "#d62728"),
    "m2_circle_geo":  (_cl(2, "circle", "geo"),   "Mode 2", "circle", "Geo",  "#1f77b4"),
    "m2_circle_indi": (_cl(2, "circle", "indi"),  "Mode 2", "circle", "INDI", "#1f77b4"),
    "m2_fig8_geo":    (_cl(2, "figure8", "geo"),   "Mode 2", "fig8",   "Geo",  "#ff7f0e"),
    "m2_fig8_indi":   (_cl(2, "figure8", "indi"),  "Mode 2", "fig8",   "INDI", "#ff7f0e"),
    "m2_helix_geo":   (_cl(2, "helix", "geo"),     "Mode 2", "helix",  "Geo",  "#2ca02c"),
    "m2_helix_indi":  (_cl(2, "helix", "indi"),    "Mode 2", "helix",  "INDI", "#2ca02c"),
    "m2_corner_geo":  (_cl(2, "corner", "geo"),    "Mode 2", "corner(SE3)", "Geo",  "#17becf"),
    "m2_corner_indi": (_cl(2, "corner", "indi"),   "Mode 2", "corner(SE3)", "INDI", "#17becf"),
    "m2_loop_geo":    (_cl(2, "loop", "geo"),      "Mode 2", "loop(SE3)", "Geo",  "#8c564b"),
    "m2_loop_indi":   (_cl(2, "loop", "indi"),     "Mode 2", "loop(SE3)", "INDI", "#8c564b"),
    "m2_flip_geo":    (_cl(2, "flip", "geo"),      "Mode 2", "flip",   "Geo",  "#9467bd"),
    "m2_flip_indi":   (_cl(2, "flip", "indi"),     "Mode 2", "flip",   "INDI", "#9467bd"),
    "m3_circle_geo":  (_cl(3, "circle", "geo"),    "Mode 3", "circle(joint)", "Geo",  "#1f77b4"),
    "m3_circle_indi": (_cl(3, "circle", "indi"),   "Mode 3", "circle(joint)", "INDI", "#1f77b4"),
    "m3_fig8_geo":    (_cl(3, "figure8", "geo"),   "Mode 3", "fig8(joint)",   "Geo",  "#ff7f0e"),
    "m3_fig8_indi":   (_cl(3, "figure8", "indi"),  "Mode 3", "fig8(joint)",   "INDI", "#ff7f0e"),
    "m3_helix_geo":   (_cl(3, "helix", "geo"),     "Mode 3", "helix(joint)",  "Geo",  "#2ca02c"),
    "m3_helix_indi":  (_cl(3, "helix", "indi"),    "Mode 3", "helix(joint)",  "INDI", "#2ca02c"),
    "m3_corner_geo":  (_cl(3, "corner", "geo"),    "Mode 3", "corner(joint)", "Geo",  "#17becf"),
    "m3_corner_indi": (_cl(3, "corner", "indi"),   "Mode 3", "corner(joint)", "INDI", "#17becf"),
    "m3_loop_geo":    (_cl(3, "loop", "geo"),      "Mode 3", "loop(joint)",   "Geo",  "#8c564b"),
    "m3_loop_indi":   (_cl(3, "loop", "indi"),     "Mode 3", "loop(joint)",   "INDI", "#8c564b"),
    "m4_circle_geo":  (_cl(4, "circle", "geo"),    "Mode 4", "circle(joint+constr)", "Geo",  "#1f77b4"),
    "m4_circle_indi": (_cl(4, "circle", "indi"),   "Mode 4", "circle(joint+constr)", "INDI", "#1f77b4"),
    "m4_fig8_geo":    (_cl(4, "figure8", "geo"),   "Mode 4", "fig8(joint+constr)",   "Geo",  "#ff7f0e"),
    "m4_fig8_indi":   (_cl(4, "figure8", "indi"),  "Mode 4", "fig8(joint+constr)",   "INDI", "#ff7f0e"),
    "m4_helix_geo":   (_cl(4, "helix", "geo"),     "Mode 4", "helix(joint+constr)",  "Geo",  "#2ca02c"),
    "m4_helix_indi":  (_cl(4, "helix", "indi"),    "Mode 4", "helix(joint+constr)",  "INDI", "#2ca02c"),
    "m4_corner_geo":  (_cl(4, "corner", "geo"),    "Mode 4", "corner(joint+constr)", "Geo",  "#17becf"),
    "m4_corner_indi": (_cl(4, "corner", "indi"),   "Mode 4", "corner(joint+constr)", "INDI", "#17becf"),
    "m4_loop_geo":    (_cl(4, "loop", "geo"),      "Mode 4", "loop(joint+constr)",   "Geo",  "#8c564b"),
    "m4_loop_indi":   (_cl(4, "loop", "indi"),     "Mode 4", "loop(joint+constr)",   "INDI", "#8c564b"),
}

# Controller line styles: Geo=solid, INDI=dotted
CTRL_LS = {"Geo": "-", "INDI": ":"}
CTRL_LW = {"Geo": 1.8, "INDI": 2.2}

data = {}
missing = []
for key, (fname, *_) in RUNS.items():
    path = os.path.join(DATA_DIR, fname)
    if not os.path.exists(path):
        missing.append(path)
    else:
        data[key] = np.genfromtxt(path, delimiter=",", names=True)

if missing:
    print("ERROR: missing CSV files. Run: cargo run --release --bin planning_sim")
    print("Expected under results/planning_sim/closed_loop/mode<N>/<trajectory>/geo.csv and indi.csv")
    for p in missing:
        print(f"  {p}")
    sys.exit(1)


def rms3d(d):
    return float(np.sqrt(np.mean(d["error_3d"] ** 2)))


# ---------------------------------------------------------------------------
# Per-trajectory 3-D axis limits and view angles
# ---------------------------------------------------------------------------

TRAJ_LIMS = {
    "circle": ((-0.7, 0.7),  (-0.7, 0.7),  (0.0, 1.5)),
    "fig8":   ((-1.5, 1.5),  (-1.5, 1.5),  (0.0, 1.5)),
    "helix":  ((-0.5, 0.5),  (-0.5, 0.5),  (0.0, 2.0)),
    "corner": ((-1.1, 0.5),  (-0.4, 1.3),  (0.0, 1.6)),
    "loop":   ((-0.7, 0.7),  (-0.7, 0.7),  (0.0, 2.5)),
    "flip":   ((-0.5, 0.5),  (-0.5, 0.5),  (0.0, 2.5)),
}

TRAJ_VIEW = {
    "circle": (25, 225),
    "fig8":   (25, 225),
    "helix":  (25, 225),
    "corner": (30, 240),
    "loop":   (15, 270),
    "flip":   (15, 270),
}


def _euler_to_body_z(roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """ZYX Euler -> world body-z column of R."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    # Third column of Rz(yaw) Ry(pitch) Rx(roll)
    z_x = cy * sp * cr + sy * sr
    z_y = sy * sp * cr - cy * sr
    z_z = cp * cr
    return z_x, z_y, z_z


def _draw_attitude_arrows(
    ax,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    roll: np.ndarray,
    pitch: np.ndarray,
    yaw: np.ndarray,
    *,
    color: str,
    axis_len: float = 0.08,
    count: int = 11,
    alpha: float = 0.5,
) -> None:
    """Sparse body-z arrows over a 3D trajectory."""
    n = len(x)
    if n < 3:
        return
    k = max(6, min(count, n))
    idx = np.unique(np.linspace(0, n - 1, k, dtype=int))
    zx, zy, zz = _euler_to_body_z(roll[idx], pitch[idx], yaw[idx])
    ax.quiver(
        x[idx], y[idx], z[idx],
        zx, zy, zz,
        length=axis_len, normalize=True, color=color, alpha=alpha, linewidth=0.8
    )


def fmt_ax3d(ax, traj_key):
    xl, yl, zl = TRAJ_LIMS[traj_key]
    ax.set_xlim(xl); ax.set_ylim(yl); ax.set_zlim(zl)
    ax.set_box_aspect([xl[1]-xl[0], yl[1]-yl[0], zl[1]-zl[0]])
    elev, azim = TRAJ_VIEW[traj_key]
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("x [m]", fontsize=6)
    ax.set_ylabel("y [m]", fontsize=6)
    ax.set_zlabel("z [m]", fontsize=6)
    ax.tick_params(labelsize=5)


# ---------------------------------------------------------------------------
# Print RMS summary
# ---------------------------------------------------------------------------
print("\nRMS 3-D tracking errors")
print("=" * 60)
for key, (_, mode_lbl, traj_lbl, ctrl_lbl, _) in RUNS.items():
    d = data[key]
    r = rms3d(d)
    print(f"  {mode_lbl} {traj_lbl:<12} {ctrl_lbl:<5}  {r*1000:7.1f} mm")
print()

# ---------------------------------------------------------------------------
# Figure 1 — 3-D trajectories: geo (solid) + indi (dotted), per mode row
# ---------------------------------------------------------------------------

MODE0_TRAJ = [("m0_circle", "circle",  "circle"),
              ("m0_fig8",   "figure8", "fig8"),
              ("m0_helix",  "helix",   "helix"),
              ("m0_corner", "corner",  "corner"),
              ("m0_loop",   "loop",    "loop")]
MODE1_TRAJ = [("m1_circle", "circle",  "circle"),
              ("m1_fig8",   "figure8", "fig8"),
              ("m1_helix",  "helix",   "helix"),
              ("m1_corner", "corner",  "corner"),
              ("m1_loop",   "loop",    "loop")]
MODE2_TRAJ = [("m2_circle", "circle",  "circle"),
              ("m2_fig8",   "figure8", "fig8"),
              ("m2_helix",  "helix",   "helix"),
              ("m2_corner", "corner(SE3)", "corner"),
              ("m2_loop",   "loop(SE3)", "loop"),
              ("m2_flip",   "flip",    "flip")]

M0_COLOR = "#1f77b4"
M1_COLOR = "#d62728"
M2_COLORS = {
    "m2_circle": "#1f77b4",
    "m2_fig8": "#ff7f0e",
    "m2_helix": "#2ca02c",
    "m2_corner": "#17becf",
    "m2_loop": "#8c564b",
    "m2_flip": "#9467bd",
}

NCOLS = 6
fig = plt.figure(figsize=(30, 15))
fig.suptitle("3-D Trajectories: path + sparse attitude arrows (body-z)", fontsize=13)

# Row 1 — Mode 0
for col, (base, tlbl, tk) in enumerate(MODE0_TRAJ):
    ax = fig.add_subplot(3, NCOLS, col + 1, projection="3d")
    dg = data[base + "_geo"]
    di = data[base + "_indi"]
    ax.plot(dg["ref_x"], dg["ref_y"], dg["ref_z"], color="gray", ls="--", lw=1.0, alpha=0.7, label="ref")
    ax.plot(dg["sim_x"], dg["sim_y"], dg["sim_z"], color=M0_COLOR, ls="-",  lw=1.8, label="Geo")
    ax.plot(di["sim_x"], di["sim_y"], di["sim_z"], color=M0_COLOR, ls=":",  lw=2.2, label="INDI")
    _draw_attitude_arrows(ax, dg["ref_x"], dg["ref_y"], dg["ref_z"], dg["ref_roll"], dg["ref_pitch"], dg["ref_yaw"],
                          color="gray", axis_len=0.07, count=9, alpha=0.45)
    _draw_attitude_arrows(ax, dg["sim_x"], dg["sim_y"], dg["sim_z"], dg["sim_roll"], dg["sim_pitch"], dg["sim_yaw"],
                          color=M0_COLOR, axis_len=0.065, count=9, alpha=0.35)
    _draw_attitude_arrows(ax, di["sim_x"], di["sim_y"], di["sim_z"], di["sim_roll"], di["sim_pitch"], di["sim_yaw"],
                          color=M0_COLOR, axis_len=0.065, count=9, alpha=0.22)
    ax.set_title(f"Mode 0 — {tlbl}", fontsize=9)
    fmt_ax3d(ax, tk)
    if col == 0:
        ax.legend(fontsize=7)

# Row 2 — Mode 1
for col, (base, tlbl, tk) in enumerate(MODE1_TRAJ):
    ax = fig.add_subplot(3, NCOLS, col + 1 + NCOLS, projection="3d")
    dg = data[base + "_geo"]
    di = data[base + "_indi"]
    ax.plot(dg["ref_x"], dg["ref_y"], dg["ref_z"], color="gray", ls="--", lw=1.0, alpha=0.7, label="ref")
    ax.plot(dg["sim_x"], dg["sim_y"], dg["sim_z"], color=M1_COLOR, ls="-",  lw=1.8, label="Geo")
    ax.plot(di["sim_x"], di["sim_y"], di["sim_z"], color=M1_COLOR, ls=":",  lw=2.2, label="INDI")
    _draw_attitude_arrows(ax, dg["ref_x"], dg["ref_y"], dg["ref_z"], dg["ref_roll"], dg["ref_pitch"], dg["ref_yaw"],
                          color="gray", axis_len=0.07, count=9, alpha=0.45)
    _draw_attitude_arrows(ax, dg["sim_x"], dg["sim_y"], dg["sim_z"], dg["sim_roll"], dg["sim_pitch"], dg["sim_yaw"],
                          color=M1_COLOR, axis_len=0.065, count=9, alpha=0.35)
    _draw_attitude_arrows(ax, di["sim_x"], di["sim_y"], di["sim_z"], di["sim_roll"], di["sim_pitch"], di["sim_yaw"],
                          color=M1_COLOR, axis_len=0.065, count=9, alpha=0.22)
    ax.set_title(f"Mode 1 — {tlbl}", fontsize=9)
    fmt_ax3d(ax, tk)
    if col == 0:
        ax.legend(fontsize=7)

# Row 3 — Mode 2
for col, (base, tlbl, tk) in enumerate(MODE2_TRAJ):
    clr = M2_COLORS[base]
    ax = fig.add_subplot(3, NCOLS, col + 1 + 2 * NCOLS, projection="3d")
    dg = data[base + "_geo"]
    di = data[base + "_indi"]
    ax.plot(dg["ref_x"], dg["ref_y"], dg["ref_z"], color="gray", ls="--", lw=1.0, alpha=0.7, label="ref")
    ax.plot(dg["sim_x"], dg["sim_y"], dg["sim_z"], color=clr, ls="-",  lw=1.8, label="Geo")
    ax.plot(di["sim_x"], di["sim_y"], di["sim_z"], color=clr, ls=":",  lw=2.2, label="INDI")
    _draw_attitude_arrows(ax, dg["ref_x"], dg["ref_y"], dg["ref_z"], dg["ref_roll"], dg["ref_pitch"], dg["ref_yaw"],
                          color="gray", axis_len=0.08, count=10, alpha=0.5)
    _draw_attitude_arrows(ax, dg["sim_x"], dg["sim_y"], dg["sim_z"], dg["sim_roll"], dg["sim_pitch"], dg["sim_yaw"],
                          color=clr, axis_len=0.075, count=10, alpha=0.38)
    _draw_attitude_arrows(ax, di["sim_x"], di["sim_y"], di["sim_z"], di["sim_roll"], di["sim_pitch"], di["sim_yaw"],
                          color=clr, axis_len=0.075, count=10, alpha=0.25)
    ax.set_title(f"Mode 2 — {tlbl}", fontsize=9)
    fmt_ax3d(ax, tk)
    ax.legend(fontsize=7)

plt.tight_layout()
out = os.path.join(IMG_DIR, "fig1_3d_trajectories.png")
plt.savefig(out, dpi=140)
print(f"Saved {os.path.basename(out)}")
plt.close()

# ---------------------------------------------------------------------------
# Figure 2 — Position tracking errors over time (M0 vs M1, geo + indi each)
# ---------------------------------------------------------------------------

TRAJ_LABELS = ["circle", "fig8", "helix", "corner", "loop"]
TRAJ_COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#17becf", "#d62728"]

fig, axes = plt.subplots(5, 5, figsize=(24, 16), sharex="col")
fig.suptitle("Position Tracking Error (ref − sim) per axis  |  solid=Geo, dotted=INDI", fontsize=13)

for col, (traj, clr) in enumerate(zip(TRAJ_LABELS, TRAJ_COLORS)):
    d0g = data[f"m0_{traj}_geo"]
    d0i = data[f"m0_{traj}_indi"]
    d1g = data[f"m1_{traj}_geo"]
    d1i = data[f"m1_{traj}_indi"]

    for row, axis in enumerate(["x", "y", "z"]):
        ax = axes[row, col]
        ax.plot(d0g["t"], d0g[f"ref_{axis}"] - d0g[f"sim_{axis}"], color=clr, lw=1.2, ls="-",  label=f"M0 Geo")
        ax.plot(d0i["t"], d0i[f"ref_{axis}"] - d0i[f"sim_{axis}"], color=clr, lw=1.5, ls=":",  label=f"M0 INDI")
        ax.plot(d1g["t"], d1g[f"ref_{axis}"] - d1g[f"sim_{axis}"], color="gray", lw=1.0, ls="-",  label=f"M1 Geo")
        ax.plot(d1i["t"], d1i[f"ref_{axis}"] - d1i[f"sim_{axis}"], color="gray", lw=1.2, ls=":",  label=f"M1 INDI")
        ax.axhline(0, color="k", lw=0.5, ls=":")
        ax.set_ylabel(f"Δ{axis} [m]", fontsize=8)
        if row == 0:
            ax.set_title(traj, fontsize=10)
            ax.legend(fontsize=6)
        if row == 2:
            ax.set_xlabel("t [s]", fontsize=8)

    # Row 3: 3D error M0
    ax = axes[3, col]
    ax.plot(d0g["t"], d0g["error_3d"] * 1000, color=clr, lw=1.2, ls="-",  label=f"M0 Geo  {rms3d(d0g)*1000:.1f}mm")
    ax.plot(d0i["t"], d0i["error_3d"] * 1000, color=clr, lw=1.5, ls=":",  label=f"M0 INDI {rms3d(d0i)*1000:.1f}mm")
    ax.set_ylabel("‖e‖ [mm]", fontsize=8)
    ax.set_xlabel("t [s]", fontsize=8)
    ax.legend(fontsize=6)
    ax.set_title(f"3D error M0 — {traj}", fontsize=8)

    # Row 4: 3D error M1
    ax = axes[4, col]
    ax.plot(d1g["t"], d1g["error_3d"] * 1000, color="gray", lw=1.2, ls="-",  label=f"M1 Geo  {rms3d(d1g)*1000:.1f}mm")
    ax.plot(d1i["t"], d1i["error_3d"] * 1000, color="gray", lw=1.5, ls=":",  label=f"M1 INDI {rms3d(d1i)*1000:.1f}mm")
    ax.set_ylabel("‖e‖ [mm]", fontsize=8)
    ax.set_xlabel("t [s]", fontsize=8)
    ax.legend(fontsize=6)
    ax.set_title(f"3D error M1 — {traj}", fontsize=8)

plt.tight_layout()
out = os.path.join(IMG_DIR, "fig2_position_errors.png")
plt.savefig(out, dpi=140)
print(f"Saved {os.path.basename(out)}")
plt.close()

# ---------------------------------------------------------------------------
# Figure 3 — Attitude (ref vs sim roll/pitch/yaw)
# ---------------------------------------------------------------------------

fig, axes = plt.subplots(3, 4, figsize=(20, 10), sharex="col")
fig.suptitle("Attitude: Reference (dashed) vs Geo (solid) vs INDI (dotted)  [rad]", fontsize=13)

DISPLAY_RUNS = [
    ("m0_circle", "Mode 0 — circle",        "#1f77b4"),
    ("m0_loop",   "Mode 0 — loop",          "#d62728"),
    ("m2_flip",   "Mode 2 — flip (pitch→π)", "#9467bd"),
    ("m2_loop",   "Mode 2 — loop (pitch→2π)", "#8c564b"),
]

for col, (base, title, clr) in enumerate(DISPLAY_RUNS):
    dg = data[base + "_geo"]
    di = data[base + "_indi"]
    for row, angle in enumerate(["roll", "pitch", "yaw"]):
        ax = axes[row, col]
        ax.plot(dg["t"], dg[f"ref_{angle}"], color=clr, ls="--", lw=1.4, label="ref")
        ax.plot(dg["t"], dg[f"sim_{angle}"], color=clr, ls="-",  lw=1.2, alpha=0.8, label="Geo")
        ax.plot(di["t"], di[f"sim_{angle}"], color=clr, ls=":",  lw=1.8, alpha=0.8, label="INDI")
        ax.set_ylabel(f"{angle} [rad]", fontsize=8)
        ax.legend(fontsize=6)
        if row == 0:
            ax.set_title(title, fontsize=9)
        if row == 2:
            ax.set_xlabel("t [s]", fontsize=8)
        ax.axhline(0, color="k", lw=0.5, ls=":")
        if col >= 2:
            ax.axhline(np.pi,     color="gray", lw=0.7, ls=":", alpha=0.6)
            ax.axhline(2 * np.pi, color="gray", lw=0.7, ls=":", alpha=0.6)

plt.tight_layout()
out = os.path.join(IMG_DIR, "fig3_attitude.png")
plt.savefig(out, dpi=140)
print(f"Saved {os.path.basename(out)}")
plt.close()

# ---------------------------------------------------------------------------
# Figure 4 — Thrust profiles Modes 0 and 1
# ---------------------------------------------------------------------------

fig, axes = plt.subplots(2, 5, figsize=(24, 8))
fig.suptitle("Commanded Thrust [N] — solid=Geo, dotted=INDI", fontsize=13)

THRUST_BASES = [f"m0_{t}" for t in TRAJ_LABELS] + [f"m1_{t}" for t in TRAJ_LABELS]
THRUST_COLORS_LIST = TRAJ_COLORS * 2

for i, (base, clr) in enumerate(zip(THRUST_BASES, THRUST_COLORS_LIST)):
    row, col = divmod(i, 5)
    ax = axes[row, col]
    dg = data[base + "_geo"]
    di = data[base + "_indi"]
    ax.plot(dg["t"], dg["ref_thrust"], ls="--", color="gray",  lw=1.2, label="ref thrust")
    ax.plot(dg["t"], dg["cmd_thrust"], ls="-",  color=clr,     lw=1.4, alpha=0.85, label="Geo cmd")
    ax.plot(di["t"], di["cmd_thrust"], ls=":",  color=clr,     lw=1.8, alpha=0.85, label="INDI cmd")
    ax.axhline(0, color="k", lw=0.5, ls=":")
    mode = "M0" if i < 5 else "M1"
    ax.set_title(f"{mode} {TRAJ_LABELS[i % 5]}", fontsize=9)
    ax.set_xlabel("t [s]", fontsize=8)
    ax.set_ylabel("T [N]", fontsize=8)
    ax.legend(fontsize=7)

plt.tight_layout()
out = os.path.join(IMG_DIR, "fig4_thrust_mode01.png")
plt.savefig(out, dpi=140)
print(f"Saved {os.path.basename(out)}")
plt.close()

# Mode 2 thrust
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))
fig.suptitle("Mode 2 Thrust Profiles — Inversion Events  |  solid=Geo, dotted=INDI", fontsize=12)

for ax, base, clr in [(ax1, "m2_flip", "#9467bd"), (ax2, "m2_loop", "#8c564b")]:
    dg = data[base + "_geo"]
    di = data[base + "_indi"]
    traj_lbl = "flip" if "flip" in base else "loop(SE3)"
    ax.plot(dg["t"], dg["ref_thrust"], ls="--", color="gray",  lw=1.4, label="ref (planner)")
    ax.plot(dg["t"], dg["cmd_thrust"], ls="-",  color=clr,     lw=2.0, alpha=0.85, label="Geo cmd")
    ax.plot(di["t"], di["cmd_thrust"], ls=":",  color=clr,     lw=2.4, alpha=0.85, label="INDI cmd")
    ax.axhline(0, color="k", lw=1.0, ls="-", label="T=0")
    ax.fill_between(dg["t"], dg["ref_thrust"], 0,
                    where=(dg["ref_thrust"] < 0), color="red", alpha=0.15, label="neg thrust region")
    ax.set_title(f"Mode 2 — {traj_lbl}", fontsize=10)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("Thrust [N]")
    ax.legend(fontsize=8)

plt.tight_layout()
out = os.path.join(IMG_DIR, "fig4b_thrust_mode2.png")
plt.savefig(out, dpi=140)
print(f"Saved {os.path.basename(out)}")
plt.close()

# ---------------------------------------------------------------------------
# Figure 5 — RMS summary bar chart (grouped: Geo vs INDI per trajectory/mode)
# ---------------------------------------------------------------------------

fig, ax = plt.subplots(figsize=(16, 6))
fig.suptitle("RMS 3-D Position Error — Geo vs INDI per Run", fontsize=13)

# Group order: m0_*, m1_*, m2_*
BASES_ORDERED = (
    [f"m0_{t}" for t in TRAJ_LABELS] +
    [f"m1_{t}" for t in TRAJ_LABELS] +
    ["m2_circle", "m2_fig8", "m2_helix", "m2_corner", "m2_loop", "m2_flip"]
)
XLABELS = (
    [f"M0\n{t}" for t in ["circle", "fig8", "helix", "corner", "loop"]] +
    [f"M1\n{t}" for t in ["circle", "fig8", "helix", "corner", "loop"]] +
    ["M2\ncircle", "M2\nfig8", "M2\nhelix", "M2\ncorner(SE3)", "M2\nloop(SE3)", "M2\nflip"]
)
BASE_COLORS = (
    ["#1f77b4", "#ff7f0e", "#2ca02c", "#17becf", "#d62728"] +
    ["#aec7e8", "#ffbb78", "#98df8a", "#9edae5", "#ff9896"] +
    ["#1f77b4", "#ff7f0e", "#2ca02c", "#17becf", "#8c564b", "#9467bd"]
)

n = len(BASES_ORDERED)
x = np.arange(n)
w = 0.38

geo_vals  = [rms3d(data[b + "_geo"])  * 1000 for b in BASES_ORDERED]
indi_vals = [rms3d(data[b + "_indi"]) * 1000 for b in BASES_ORDERED]

bars_geo  = ax.bar(x - w/2, geo_vals,  w, color=BASE_COLORS, edgecolor="k", linewidth=0.5, label="Geo")
bars_indi = ax.bar(x + w/2, indi_vals, w, color=BASE_COLORS, edgecolor="k", linewidth=0.5,
                   hatch="//", alpha=0.75, label="INDI")

ax.set_xticks(x)
ax.set_xticklabels(XLABELS, fontsize=8)
ax.set_ylabel("RMS 3-D error [mm]")
ax.axhline(10,  color="green",  ls="--", lw=1.0)
ax.axhline(50,  color="orange", ls="--", lw=1.0)
ax.axhline(100, color="red",    ls="--", lw=1.0)

for bar, val in zip(list(bars_geo) + list(bars_indi),
                    geo_vals + indi_vals):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
            f"{val:.1f}", ha="center", va="bottom", fontsize=6)

legend_handles = [
    Patch(facecolor="white", edgecolor="k", label="Geo (solid fill)"),
    Patch(facecolor="white", edgecolor="k", hatch="//", label="INDI (hatched)"),
    Patch(facecolor="#1f77b4", edgecolor="k", label="Mode 0"),
    Patch(facecolor="#aec7e8", edgecolor="k", label="Mode 1"),
    Patch(facecolor="#9467bd", edgecolor="k", label="Mode 2"),
    plt.Line2D([0],[0], color="green",  ls="--", label="10 mm"),
    plt.Line2D([0],[0], color="orange", ls="--", label="50 mm"),
    plt.Line2D([0],[0], color="red",    ls="--", label="100 mm"),
]
ax.legend(handles=legend_handles, fontsize=8, loc="upper left")

plt.tight_layout()
out = os.path.join(IMG_DIR, "fig5_rms_summary.png")
plt.savefig(out, dpi=140)
print(f"Saved {os.path.basename(out)}")
plt.close()

# ---------------------------------------------------------------------------
# Figure 6 — Cross-mode comparison (all modes on same trajectory shape)
# ---------------------------------------------------------------------------

fig = plt.figure(figsize=(18, 14))
fig.suptitle(
    "Cross-Mode: Reference (gray--) | M0 Geo (solid blue) | M0 INDI (dotted blue) | M1 / M2",
    fontsize=12,
)

CROSS_SPECS = [
    (1, "circle", "circle",
     [("m0_circle", M0_COLOR), ("m1_circle", M1_COLOR), ("m2_circle", "#1f77b4")]),
    (2, "fig8",   "figure8",
     [("m0_fig8",   M0_COLOR), ("m1_fig8",   M1_COLOR), ("m2_fig8", "#ff7f0e")]),
    (3, "helix",  "helix",
     [("m0_helix",  M0_COLOR), ("m1_helix",  M1_COLOR), ("m2_helix", "#2ca02c")]),
    (4, "corner", "racing corner (banked in M2)",
     [("m0_corner", M0_COLOR), ("m1_corner", M1_COLOR), ("m2_corner", "#17becf")]),
    (5, "loop",   "loop (M0/M1 horizontal, M2 vertical SE3)",
     [("m0_loop",   M0_COLOR), ("m1_loop",   M1_COLOR), ("m2_loop", "#8c564b")]),
    (6, "flip",   "flip (Mode 2 only)",
     [("m2_flip", "#9467bd")]),
]

for sp_idx, tk, title, entries in CROSS_SPECS:
    ax = fig.add_subplot(2, 3, sp_idx, projection="3d")
    d_ref = data[entries[0][0] + "_geo"]
    ax.plot(d_ref["ref_x"], d_ref["ref_y"], d_ref["ref_z"],
            color="gray", ls="--", lw=1.0, alpha=0.7, label="reference")
    for base, clr in entries:
        dg = data[base + "_geo"]
        di = data[base + "_indi"]
        mode_tag = "M0" if "m0" in base else ("M1" if "m1" in base else "M2")
        rg = rms3d(dg)
        ri = rms3d(di)
        ax.plot(dg["sim_x"], dg["sim_y"], dg["sim_z"], color=clr, lw=1.6, ls="-",
                label=f"{mode_tag} Geo {rg*1000:.1f}mm")
        ax.plot(di["sim_x"], di["sim_y"], di["sim_z"], color=clr, lw=2.0, ls=":",
                label=f"{mode_tag} INDI {ri*1000:.1f}mm")
    ax.set_title(title, fontsize=9)
    fmt_ax3d(ax, tk)
    ax.legend(fontsize=6, loc="upper left")

plt.tight_layout()
out = os.path.join(IMG_DIR, "fig6_cross_mode.png")
plt.savefig(out, dpi=140)
print(f"Saved {os.path.basename(out)}")
plt.close()

print("\nAll plots saved to:", IMG_DIR)
