#!/usr/bin/env python3
"""Standalone plotting for assignment4 planned/open-loop/closed-loop CSVs.
Generates a few standard plots used for assignment4 analysis.
"""
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

suffixes = ["_classic", "_safety"]
out_dir = Path("results/images")
out_dir.mkdir(parents=True, exist_ok=True)

for suf in suffixes:
    planned = Path(f"results/data/assignment4_planned{suf}.csv")
    cl = Path(f"results/data/assignment4_closedloop{suf}.csv")
    ol = Path(f"results/data/assignment4_openloop{suf}.csv")
    if not planned.exists():
        print(f"Planned file {planned} not found, skipping {suf}.")
        continue
    print(f"Plotting Assignment4 (suffix={suf})...")
    p = np.genfromtxt(planned, delimiter=",", names=True)
    if cl.exists():
        c = np.genfromtxt(cl, delimiter=",", names=True)
    else:
        c = None
    if ol.exists():
        o = np.genfromtxt(ol, delimiter=",", names=True)
    else:
        o = None

    # 3D planned
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(p["x"], p["y"], p["z"], label="planned")
    if c is not None:
        ax.plot(c["sim_x"], c["sim_y"], c["sim_z"], label="closed-loop")
    if o is not None:
        ax.plot(o["sim_x"], o["sim_y"], o["sim_z"], label="open-loop")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title(f"Assignment4 {suf} (3D)")
    ax.legend()
    fig.savefig(out_dir / f"assignment4_3d_{suf.strip('_')}.png", dpi=150)
    plt.close(fig)

    # z vs t
    fig, ax = plt.subplots(figsize=(6, 3))
    ax.plot(p["t"], p["z"], label="planned z")
    if c is not None:
        ax.plot(c["t"], c["sim_z"], label="closed-loop z")
    if o is not None:
        ax.plot(o["t"], o["sim_z"], label="open-loop z")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("z [m]")
    ax.set_title(f"Assignment4 {suf} z(t)")
    ax.legend()
    fig.savefig(out_dir / f"assignment4_z_{suf.strip('_')}.png", dpi=150)
    plt.close(fig)

print("Done.")
