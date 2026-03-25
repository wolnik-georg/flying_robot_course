#!/usr/bin/env python3
"""
Assignment 4 – Minimum-Snap Spline Planning + Differential Flatness
====================================================================
Reads three CSVs from results/assignment4/data/:
  assignment4_planned.csv     — planned trajectory + flatness outputs
  assignment4_openloop.csv    — open-loop simulation (flatness feedforward only)
  assignment4_closedloop.csv  — closed-loop simulation (geometric controller)

Produces six PNG files in results/assignment4/images/.
"""

import os
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

_ROOT = os.path.dirname(os.path.dirname(__file__))
DATA_DIR = os.path.join(_ROOT, "results", "assignment4", "data")
IMG_DIR  = os.path.join(_ROOT, "results", "assignment4", "images")
os.makedirs(IMG_DIR, exist_ok=True)


def load(name):
    path = os.path.join(DATA_DIR, name)
    if not os.path.exists(path):
        raise FileNotFoundError(f"Missing: {path}\nRun: cargo run --release --bin assignment4")
    return np.genfromtxt(path, delimiter=",", names=True)


p  = load("assignment4_planned.csv")
ol = load("assignment4_openloop.csv")
cl = load("assignment4_closedloop.csv")

t_p  = p["t"]
t_ol = ol["t"]
t_cl = cl["t"]

# ── helpers ──────────────────────────────────────────────────────────────────
def rms(v):
    return float(np.sqrt(np.mean(np.asarray(v) ** 2)))

err3d_ol = np.sqrt((ol["ref_x"]-ol["sim_x"])**2 + (ol["ref_y"]-ol["sim_y"])**2 + (ol["ref_z"]-ol["sim_z"])**2)
err3d_cl = np.sqrt((cl["ref_x"]-cl["sim_x"])**2 + (cl["ref_y"]-cl["sim_y"])**2 + (cl["ref_z"]-cl["sim_z"])**2)

print(f"Open-loop  RMS 3D error : {rms(err3d_ol)*1000:.1f} mm")
print(f"Closed-loop RMS 3D error: {rms(err3d_cl)*1000:.1f} mm")


# ── Figure 1: 3-D trajectory ──────────────────────────────────────────────
fig = plt.figure(figsize=(8, 7))
ax  = fig.add_subplot(111, projection="3d")

ax.plot(p["x"], p["y"], p["z"], "b-", lw=2.5, label="Planned (min-snap)")
ax.plot(cl["sim_x"], cl["sim_y"], cl["sim_z"], "g-", lw=1.8, alpha=0.9, label="Closed-loop")
ax.plot(ol["sim_x"], ol["sim_y"], ol["sim_z"], "r--", lw=1.2, alpha=0.7, label="Open-loop")

# Mark waypoints from planned trajectory (approx segment boundaries)
total_t = t_p[-1]
n_seg   = 10          # total segments including hover pre/post
for k in range(n_seg + 1):
    t_wp = k * total_t / n_seg
    idx  = min(np.searchsorted(t_p, t_wp), len(t_p) - 1)
    ax.scatter(p["x"][idx], p["y"][idx], p["z"][idx], c="navy", s=40, zorder=5)

ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
ax.set_title("Assignment 4 — Planned vs Simulated Trajectory")
ax.legend(loc="upper left")
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_3d.png"), dpi=150)
print("Saved assignment4_3d.png")
plt.close()


# ── Figure 2: Position tracking (closed-loop) ────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
labels = ["x [m]", "y [m]", "z [m]"]
ref_cols = ["ref_x", "ref_y", "ref_z"]
sim_cols = ["sim_x", "sim_y", "sim_z"]
for i, ax in enumerate(axes):
    ax.plot(t_cl, cl[ref_cols[i]], "b-",  lw=1.8, label="Reference (planned)")
    ax.plot(t_cl, cl[sim_cols[i]], "g-",  lw=1.2, label="Simulated (closed-loop)")
    ax.set_ylabel(labels[i])
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.35)
axes[-1].set_xlabel("Time [s]")
axes[0].set_title("Closed-Loop Position Tracking")
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_position.png"), dpi=150)
print("Saved assignment4_position.png")
plt.close()


# ── Figure 3: Tracking error — open-loop vs closed-loop ─────────────────
fig, axes = plt.subplots(2, 1, figsize=(12, 7))

ax = axes[0]
ax.semilogy(t_ol, np.maximum(err3d_ol, 1e-6), "r-",  lw=1.3, label=f"Open-loop  (RMS = {rms(err3d_ol)*1000:.0f} mm)")
ax.semilogy(t_cl, np.maximum(err3d_cl, 1e-6), "g-",  lw=1.5, label=f"Closed-loop (RMS = {rms(err3d_cl)*1000:.0f} mm)")
ax.axhline(0.01, color="gray", ls=":", lw=1, label="1 cm threshold")
ax.set_ylabel("3D error [m] (log)")
ax.set_title("Position Tracking Error")
ax.legend(fontsize=9)
ax.grid(True, which="both", alpha=0.35)
ax.set_ylim(1e-6, 50)

ax = axes[1]
ax.plot(t_cl, err3d_cl * 1000, "g-", lw=1.5)
ax.set_ylabel("Closed-loop error [mm]")
ax.set_xlabel("Time [s]")
ax.set_title(f"Closed-Loop Error Detail (RMS = {rms(err3d_cl)*1000:.1f} mm)")
ax.grid(True, alpha=0.35)

plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_errors.png"), dpi=150)
print("Saved assignment4_errors.png")
plt.close()


# ── Figure 4: Differential flatness outputs — thrust & torques ───────────
fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
ylabels   = ["Thrust [N]", "τx [Nm]", "τy [Nm]", "τz [Nm]"]
p_cols    = ["thrust", "tx", "ty", "tz"]
ref_cols  = ["ref_thrust", "ref_tx", "ref_ty", "ref_tz"]
cmd_cols  = ["cmd_thrust", "cmd_tx", "cmd_ty", "cmd_tz"]

for i, ax in enumerate(axes):
    ax.plot(t_p,  p[p_cols[i]],       "b-",  lw=1.8, label="Flatness (planned)")
    ax.plot(t_cl, cl[cmd_cols[i]],    "g--", lw=1.2, label="Controller commanded")
    ax.set_ylabel(ylabels[i])
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.35)
axes[-1].set_xlabel("Time [s]")
axes[0].set_title("Differential Flatness: Thrust & Torques")
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_flatness_actions.png"), dpi=150)
print("Saved assignment4_flatness_actions.png")
plt.close()


# ── Figure 5: Angular velocity (omega) from flatness ─────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
omega_cols   = ["ox", "oy", "oz"]
omega_labels = ["ωx [rad/s]", "ωy [rad/s]", "ωz [rad/s]"]
for i, ax in enumerate(axes):
    ax.plot(t_p, p[omega_cols[i]], "b-", lw=1.5)
    ax.set_ylabel(omega_labels[i])
    ax.grid(True, alpha=0.35)
axes[-1].set_xlabel("Time [s]")
axes[0].set_title("Planned Angular Velocity from Differential Flatness")
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_omega.png"), dpi=150)
print("Saved assignment4_omega.png")
plt.close()


# ── Figure 6: Planned position / velocity / acceleration ─────────────────
fig, axes = plt.subplots(3, 3, figsize=(15, 9), sharex=True)
rows = [
    ("Position [m]",        ["x", "y", "z"]),
    ("Velocity [m/s]",      ["vx", "vy", "vz"]),
    ("Acceleration [m/s²]", ["ax", "ay", "az"]),
]
colours = ["C0", "C1", "C2"]
for row, (ylabel, keys) in enumerate(rows):
    for col, (key, c) in enumerate(zip(keys, colours)):
        ax = axes[row][col]
        ax.plot(t_p, p[key], color=c, lw=1.4)
        ax.set_ylabel(key)
        ax.grid(True, alpha=0.35)
        if row == 0:
            ax.set_title(key)
    axes[row][0].set_ylabel(ylabel)
for col in range(3):
    axes[-1][col].set_xlabel("Time [s]")
fig.suptitle("Planned Trajectory: Position / Velocity / Acceleration")
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_derivatives.png"), dpi=150)
print("Saved assignment4_derivatives.png")
plt.close()

print("Done.")
