#!/usr/bin/env python3
"""
Full validation report for all exported trajectories (Phase 5 of the
trajectory-port plan): per (trajectory, mode, kt) — does it exist/work,
does it fit the flight space at the trajectory's recommended hover height,
duration, average speed — plus one 3D plot per trajectory (mode1 vs mode3
overlay, all kt values) saved to disk for visual confirmation.

Usage:
    ~/.pyenv/versions/flying_robots/bin/python plot_exported_trajectories.py
"""

import os
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

DATA_DIR = "/home/georg/Desktop/crazyswarm2/crazyflie_examples/crazyflie_examples/data"
OUT_DIR = "trajectory_plots/exported_overview"

X_LIM = (-1.2, 1.2)
Y_LIM = (-2.5, 2.5)
Z_LIM = (0.5, 1.8)

# Recommended --height per trajectory (docs/flight_space.md)
RECOMMENDED_HEIGHT = {
    "circle": 0.7, "figure8": 0.7, "corner": 0.7, "oval": 0.7, "slalom": 0.7,
    "helix": 0.65, "corkscrew": 0.65, "loop": 0.65,
    "teardrop": 1.15, "teardrop_wide": 1.15, "tilted_oval": 1.15, "loop_train": 1.15,
    "roller_coaster": 0.89,
    "corner_loop": 0.7, "slalom_loop": 0.7, "loop_train_loop": 1.15, "roller_coaster_loop": 0.89,
}

M1_KTS = {
    "corner_loop": "0.05 0.06 0.07 0.08 0.09 0.1 0.15 0.2 0.3".split(),
    "slalom_loop": "0.05 0.1 0.2 0.3 0.5 0.7 1.0".split(),
    "loop_train_loop": "0.05 0.1 0.2 0.3 0.5 0.7 1.0".split(),
    "roller_coaster_loop": "0.05 0.06 0.07 0.08 0.09 0.1 0.15 0.2 0.3 0.5 0.7 1.0".split(),
    "circle": "0.05 0.1 0.2 0.3 0.5 0.7 1.0".split(),
    "figure8": "0.008 0.05 0.1 0.2 0.3 0.5 0.7 1.0".split(),
    "helix": "0.05 0.06 0.07 0.08 0.09 0.1 0.15 0.2 0.3 0.5 0.7".split(),
    "corkscrew": "0.05 0.06 0.07 0.08 0.09 0.1 0.15 0.2 0.3 0.5 0.7".split(),
    "corner": "0.05 0.06 0.07 0.08 0.09 0.1 0.15 0.2 0.3 0.5".split(),
    "loop": "0.05 0.06 0.07 0.08 0.09 0.1 0.15 0.2 0.3 0.7 1.0".split(),
    "teardrop": "0.05 0.1 0.2 0.3 0.5 0.7 1.0".split(),
    "teardrop_wide": "0.05 0.1 0.2 0.3 0.5 0.7".split(),
    "loop_train": "0.05 0.1 0.2 0.3 0.5 0.7 1.0".split(),
    "roller_coaster": "0.05 0.06 0.07 0.08 0.09 0.1 0.15".split(),
    "oval": "0.05 0.1 0.2 0.3 0.5 0.7 1.0".split(),
    "slalom": "0.05 0.1 0.2 0.3 0.5 0.7 1.0".split(),
    "tilted_oval": "0.05 0.1 0.2 0.3 0.5 0.7 1.0".split(),
}
M3_KTS = {
    "corner_loop": "0.05 0.06 0.07 0.08 0.1 0.15".split(),
    "slalom_loop": [],
    "loop_train_loop": [],
    "roller_coaster_loop": "0.05 0.06 0.07 0.08 0.09 0.1 0.15 0.2 0.3 0.7".split(),
    "circle": "0.05 0.1 0.2 0.3 0.5 0.7 1.0".split(),
    "figure8": "0.008 0.05 0.1 0.2 0.3 0.5 0.7".split(),
    "helix": "0.05 0.06 0.07 0.08 0.09 0.1".split(),
    "corkscrew": "0.05 0.06 0.07 0.1".split(),
    "corner": "0.05 0.06 0.07 0.08 0.09".split(),
    "loop": "0.05 0.06 0.07 0.08 0.09 0.1 0.15 0.2 0.3".split(),
    "teardrop": "0.05 0.1 0.2".split(),
    "teardrop_wide": "0.05 0.1 0.2 0.3 0.5".split(),
    "loop_train": [],  # mode3 excluded — see docs/flight_space.md
    "roller_coaster": "0.05 0.06 0.07 0.09".split(),
    "oval": "0.05 0.1 0.2 0.3 0.5 0.7 1.0".split(),
    "slalom": "0.05 0.1 0.2 0.3 0.5 0.7 1.0".split(),
    "tilted_oval": "0.05 0.1 0.2 0.3 0.5 0.7 1.0".split(),
}

TRAJS = ["circle", "figure8", "helix", "corkscrew", "corner", "loop",
         "teardrop", "teardrop_wide", "loop_train", "roller_coaster",
         "oval", "slalom", "tilted_oval",
         "corner_loop", "slalom_loop", "loop_train_loop", "roller_coaster_loop"]


def poly_eval(coefs, tau):
    return sum(c * tau ** i for i, c in enumerate(coefs))


def load(path):
    segs = []
    with open(path) as f:
        for row in csv.DictReader(f):
            dur = float(row["duration"])
            cx = [float(row[f"cx{i}"]) for i in range(9)]
            cy = [float(row[f"cy{i}"]) for i in range(9)]
            cz = [float(row.get(f"cz{i}", 0.0) or 0.0) for i in range(9)]
            segs.append((dur, cx, cy, cz))
    return segs


def eval_dense(segs, pts_per_seg=60):
    xs, ys, zs = [], [], []
    for dur, cx, cy, cz in segs:
        for tau in np.linspace(0, 1, pts_per_seg, endpoint=False):
            xs.append(poly_eval(cx, tau)); ys.append(poly_eval(cy, tau)); zs.append(poly_eval(cz, tau))
    dur, cx, cy, cz = segs[-1]
    xs.append(poly_eval(cx, 1.0)); ys.append(poly_eval(cy, 1.0)); zs.append(poly_eval(cz, 1.0))
    return np.array(xs), np.array(ys), np.array(zs)


def kt_fmt(kt):
    return kt.rstrip("0").rstrip(".") if "." in kt else kt


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    report_rows = []
    plot_paths = {}

    for traj in TRAJS:
        hz = RECOMMENDED_HEIGHT[traj]
        fig = plt.figure(figsize=(18, 9))
        ax = fig.add_subplot(121, projection="3d")   # room-scale context
        ax2 = fig.add_subplot(122, projection="3d")  # zoomed, equal-aspect shape detail
        cmap1 = plt.cm.Blues
        cmap3 = plt.cm.Oranges

        # Draw the flight-space bounding box for scale reference (physically
        # proportioned — see below for equal-aspect axis setup).
        for xs in X_LIM:
            for ys in Y_LIM:
                ax.plot([xs, xs], [ys, ys], list(Z_LIM), color="gray", lw=0.6, alpha=0.4)
        for xs in X_LIM:
            for zs in Z_LIM:
                ax.plot([xs, xs], list(Y_LIM), [zs, zs], color="gray", lw=0.6, alpha=0.4)
        for ys in Y_LIM:
            for zs in Z_LIM:
                ax.plot(list(X_LIM), [ys, ys], [zs, zs], color="gray", lw=0.6, alpha=0.4)

        all_x, all_y, all_z = [], [], []

        for mode, ktlist, cmap in ((1, M1_KTS[traj], cmap1), (3, M3_KTS[traj], cmap3)):
            n = max(len(ktlist), 1)
            for i, kt in enumerate(ktlist):
                fname = f"{traj}_mode{mode}_kt{kt_fmt(kt)}_onboard.csv"
                path = os.path.join(DATA_DIR, fname)
                exists = os.path.exists(path)
                if not exists:
                    report_rows.append((traj, mode, kt, "MISSING", "-", "-", "-", "-"))
                    continue
                segs = load(path)
                dur = sum(s[0] for s in segs)
                x, y, z = eval_dense(segs)
                length = np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2).sum()
                spd = length / dur
                x0, x1 = x.min(), x.max()
                y0, y1 = y.min(), y.max()
                z_abs0, z_abs1 = hz + z.min(), hz + z.max()
                fits = (X_LIM[0] <= x0 and x1 <= X_LIM[1]
                        and Y_LIM[0] <= y0 and y1 <= Y_LIM[1]
                        and Z_LIM[0] <= z_abs0 and z_abs1 <= Z_LIM[1])
                report_rows.append((traj, mode, kt, "OK", f"{dur:.2f}", f"{spd:.2f}",
                                     "FITS" if fits else "OUT-OF-BOUNDS", f"hz={hz}"))
                color = cmap(0.35 + 0.5 * i / n)
                z_abs = hz + z
                ax.plot(x, y, z_abs, color=color, lw=1.1, alpha=0.85)
                ax2.plot(x, y, z_abs, color=color, lw=1.3, alpha=0.85)
                all_x.append(x); all_y.append(y); all_z.append(z_abs)

        ax.set_title(f"{traj} — room context (blue=mode1, orange=mode3)\n"
                     f"gray box = full flight space (x:±1.2m y:±2.5m z:0.5-1.8m), axes to scale",
                     fontsize=10)
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m] (absolute)")
        ax.set_xlim(X_LIM); ax.set_ylim(Y_LIM); ax.set_zlim(Z_LIM)
        ax.set_box_aspect((X_LIM[1] - X_LIM[0], Y_LIM[1] - Y_LIM[0], Z_LIM[1] - Z_LIM[0]))
        ax.view_init(elev=22, azim=-55)

        # Zoomed panel: equal aspect ratio (1m x = 1m y = 1m z) but cropped tight to the
        # trajectory itself, so shapes (circles, loops) render undistorted at readable size.
        if all_x:
            ax_c = np.concatenate(all_x); ay_c = np.concatenate(all_y); az_c = np.concatenate(all_z)
            cx, cy, cz = ax_c.mean(), ay_c.mean(), az_c.mean()
            half = max(ax_c.max()-ax_c.min(), ay_c.max()-ay_c.min(), az_c.max()-az_c.min(), 0.2) / 2 * 1.15
            ax2.set_xlim(cx-half, cx+half); ax2.set_ylim(cy-half, cy+half); ax2.set_zlim(cz-half, cz+half)
            ax2.set_box_aspect((1, 1, 1))
        ax2.set_title(f"{traj} — shape detail (equal aspect, zoomed)", fontsize=10)
        ax2.set_xlabel("x [m]"); ax2.set_ylabel("y [m]"); ax2.set_zlabel("z [m] (absolute)")
        ax2.view_init(elev=22, azim=-55)

        out_path = os.path.join(OUT_DIR, f"{traj}_3d.png")
        fig.savefig(out_path, dpi=140, bbox_inches="tight")
        plt.close(fig)
        plot_paths[traj] = out_path
        print(f"Saved: {out_path}")

    print("\n=== Full report ===")
    print(f"{'trajectory':<16}{'mode':>5}{'kt':>8}{'status':>10}{'dur[s]':>9}{'spd[m/s]':>10}{'space':>16}  note")
    for row in report_rows:
        traj, mode, kt, status, dur, spd, space, note = row
        print(f"{traj:<16}{mode:>5}{kt:>8}{status:>10}{dur:>9}{spd:>10}{space:>16}  {note}")

    print("\n=== Plot files ===")
    for traj, p in plot_paths.items():
        print(f"  {traj:<16} {p}")


if __name__ == "__main__":
    main()
