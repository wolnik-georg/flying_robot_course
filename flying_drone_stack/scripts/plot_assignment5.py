#!/usr/bin/env python3
"""Standalone plotting for assignment5 planned vs closed-loop CSVs.
Usage: run in repository root or multirotor_simulator/ directory.
Generates 3D and z-vs-time plots for each mode found (circle, figure8, hover).
"""
from pathlib import Path
import subprocess
import sys


modes = ["circle", "figure8", "hover"]
out_dir = Path("results/assignment5/images")
out_dir.mkdir(parents=True, exist_ok=True)


def _have_matplotlib_numpy():
    try:
        import numpy as np  # noqa: F401
        import matplotlib  # noqa: F401

        return True
    except Exception:
        return False


if _have_matplotlib_numpy():
    # If the user's environment supports NumPy/Matplotlib, use the original plotting.
    import numpy as np
    import matplotlib.pyplot as plt

    for mode in modes:
        planned = Path(f"results/assignment5/data/assignment5_planned_{mode}.csv")
        cl = Path(f"results/assignment5/data/assignment5_closedloop_{mode}.csv")
        if not planned.exists() or not cl.exists():
            print(f"Assignment5 files for mode='{mode}' not found, skipping.")
            continue

        print(f"Plotting Assignment5 ({mode}) results...")
        p = np.genfromtxt(planned, delimiter=",", names=True)
        c = np.genfromtxt(cl, delimiter=",", names=True)

        # 3D trajectory
        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(p["x"], p["y"], p["z"], label="planned", color="C0")
        ax.plot(c["sim_x"], c["sim_y"], c["sim_z"], label="closed-loop", color="C1")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        ax.set_title(f"Assignment5 {mode} (3D)")
        ax.legend()
        fig.savefig(out_dir / f"assignment5_3d_{mode}.png", dpi=150)
        plt.close(fig)

        # z vs time
        fig, ax = plt.subplots(figsize=(6, 3))
        ax.plot(p["t"], p["z"], label="planned z")
        ax.plot(c["t"], c["sim_z"], label="sim z")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("z [m]")
        ax.set_title(f"Assignment5 {mode} z(t)")
        ax.legend()
        fig.savefig(out_dir / f"assignment5_z_{mode}.png", dpi=150)
        plt.close(fig)

        # quick safety check print
        sim_z = c["sim_z"]
        sim_x = c["sim_x"]
        sim_y = c["sim_y"]
        z_ok = np.all((sim_z >= 0.0) & (sim_z <= 0.30))
        xy_ok = np.all((np.abs(sim_x) <= 0.5) & (np.abs(sim_y) <= 0.5))
        print(
            f"Assignment5 {mode}: planned violation=False, closed-loop violation={not (z_ok and xy_ok)}"
        )

    print("Done.")
else:
    # Fall back to the Pillow plotting script if Matplotlib/NumPy aren't importable.
    print(
        "Matplotlib/NumPy import failed or incompatible; falling back to Pillow-based plotting."
    )
    script = Path(__file__).parent / "plot_assignment5_pillow.py"
    if not script.exists():
        print("Fallback script not found:", script)
        sys.exit(1)
    rc = subprocess.call([sys.executable, str(script)])
    if rc != 0:
        print("Fallback plotting script failed with exit code", rc)
        sys.exit(rc)
