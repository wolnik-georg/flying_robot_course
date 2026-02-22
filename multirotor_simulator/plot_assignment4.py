#!/usr/bin/env python3
"""
Assignment 4 – Motion Planning: Minimum-Snap Splines + Differential Flatness
=============================================================================
Plots:
  1. 3-D trajectory: planned vs open-loop vs closed-loop
  2. Per-axis position tracking (closed-loop)
  3. Per-axis velocity tracking (closed-loop)
  4. Planned thrust and torques from differential flatness
  5. Planned angular velocity from differential flatness
  6. Position tracking error comparison: open-loop vs closed-loop
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

DATA_DIR = os.path.join(os.path.dirname(__file__), "results", "data")
IMG_DIR = os.path.join(os.path.dirname(__file__), "results", "images")
os.makedirs(IMG_DIR, exist_ok=True)

# ── Load CSVs ────────────────────────────────────────────────────────────────


def load(name):
    path = os.path.join(DATA_DIR, name)
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"Run 'cargo run --release --bin assignment4' first.\nExpected: {path}"
        )
    return np.genfromtxt(path, delimiter=",", names=True)


planned = load("assignment4_planned.csv")
ol = load("assignment4_openloop.csv")
cl = load("assignment4_closedloop.csv")

t_p = planned["t"]
t_ol = ol["t"]
t_cl = cl["t"]

# ── Figure 1: 3-D trajectory ─────────────────────────────────────────────────
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection="3d")

ax.plot(
    planned["x"], planned["y"], planned["z"], "b-", lw=2, label="Planned (flatness)"
)
ax.plot(
    cl["sim_x"],
    cl["sim_y"],
    cl["sim_z"],
    "g-",
    lw=1.5,
    alpha=0.85,
    label="Closed-loop sim",
)
# Open-loop: only plot up to divergence (~0.65 s, i.e. first ~650 steps)
ol_horizon = ol["t"] < 1.0
ax.plot(
    ol["sim_x"][ol_horizon],
    ol["sim_y"][ol_horizon],
    ol["sim_z"][ol_horizon],
    "r--",
    lw=1.2,
    alpha=0.8,
    label="Open-loop sim (first 1 s)",
)

# Waypoints from planned trajectory start/end of each segment
n_seg = 8
seg_dur = t_p[-1] / n_seg
for k in range(n_seg + 1):
    t_wp = k * seg_dur
    idx = np.searchsorted(t_p, t_wp)
    idx = min(idx, len(t_p) - 1)
    ax.scatter(
        planned["x"][idx], planned["y"][idx], planned["z"][idx], c="b", s=50, zorder=5
    )

ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_zlabel("z [m]")
ax.set_title("Assignment 4 – 3D Trajectory")
ax.legend()
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_3d.png"), dpi=150)
print("Saved assignment4_3d.png")

# ── Figure 2: Position tracking (closed-loop) ────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
labels = ["x", "y", "z"]
for i, ax in enumerate(axes):
    ref = [cl["ref_x"], cl["ref_y"], cl["ref_z"]][i]
    sim = [cl["sim_x"], cl["sim_y"], cl["sim_z"]][i]
    ax.plot(t_cl, ref, "b-", lw=1.5, label="Reference")
    ax.plot(t_cl, sim, "g-", lw=1, label="Simulated")
    ax.set_ylabel(f"{labels[i]} [m]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.4)
axes[-1].set_xlabel("Time [s]")
axes[0].set_title("Closed-Loop Position Tracking")
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_position_cl.png"), dpi=150)
print("Saved assignment4_position_cl.png")

# ── Figure 3: Velocity tracking (closed-loop) ────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
vlabels = ["vx", "vy", "vz"]
for i, ax in enumerate(axes):
    ref = [cl["ref_vx"], cl["ref_vy"], cl["ref_vz"]][i]
    sim = [cl["sim_vx"], cl["sim_vy"], cl["sim_vz"]][i]
    ax.plot(t_cl, ref, "b-", lw=1.5, label="Reference")
    ax.plot(t_cl, sim, "g-", lw=1, label="Simulated")
    ax.set_ylabel(f"{vlabels[i]} [m/s]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.4)
axes[-1].set_xlabel("Time [s]")
axes[0].set_title("Closed-Loop Velocity Tracking")
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_velocity_cl.png"), dpi=150)
print("Saved assignment4_velocity_cl.png")

# ── Figure 4: Planned thrust and torques ─────────────────────────────────────
fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

axes[0].plot(t_p, planned["thrust"], "b-", lw=1.5)
axes[0].set_ylabel("Thrust [N]")
axes[0].grid(True, alpha=0.4)
axes[0].axhline(0.027 * 9.81, color="r", ls="--", lw=1, label="Hover thrust")
axes[0].legend(fontsize=8)

axes[1].plot(t_p, planned["tx"], lw=1.5, label="τx")
axes[1].set_ylabel("τx [Nm]")
axes[1].grid(True, alpha=0.4)
axes[1].legend()

axes[2].plot(t_p, planned["ty"], lw=1.5, label="τy")
axes[2].set_ylabel("τy [Nm]")
axes[2].grid(True, alpha=0.4)
axes[2].legend()

axes[3].plot(t_p, planned["tz"], lw=1.5, label="τz")
axes[3].set_ylabel("τz [Nm]")
axes[3].grid(True, alpha=0.4)
axes[3].legend()

axes[-1].set_xlabel("Time [s]")
axes[0].set_title("Differential Flatness – Planned Thrust and Torques")
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_actions.png"), dpi=150)
print("Saved assignment4_actions.png")

# ── Figure 5: Planned angular velocity ───────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
omega_labels = ["ωx", "ωy", "ωz"]
omega_keys = ["ox", "oy", "oz"]
for i, ax in enumerate(axes):
    ax.plot(t_p, planned[omega_keys[i]], lw=1.5, label=omega_labels[i])
    ax.set_ylabel(f"{omega_labels[i]} [rad/s]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.4)
axes[-1].set_xlabel("Time [s]")
axes[0].set_title("Differential Flatness – Planned Angular Velocity")
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_omega.png"), dpi=150)
print("Saved assignment4_omega.png")

# ── Figure 6: Position error comparison ─────────────────────────────────────
# Open-loop: show two sub-plots side-by-side — short horizon (clean) and full
fig, axes = plt.subplots(2, 1, figsize=(13, 9), sharex=False)

errs_ol = np.sqrt(
    (ol["ref_x"] - ol["sim_x"]) ** 2
    + (ol["ref_y"] - ol["sim_y"]) ** 2
    + (ol["ref_z"] - ol["sim_z"]) ** 2
)
errs_cl = np.sqrt(
    (cl["ref_x"] - cl["sim_x"]) ** 2
    + (cl["ref_y"] - cl["sim_y"]) ** 2
    + (cl["ref_z"] - cl["sim_z"]) ** 2
)

# Top: full timeline, both systems
ax = axes[0]
ax.semilogy(t_ol, np.maximum(errs_ol, 1e-5), "r-", lw=1.2, label="Open-loop")
ax.semilogy(t_cl, np.maximum(errs_cl, 1e-5), "g-", lw=1.5, label="Closed-loop")
ax.axvline(0.647, color="r", ls="--", lw=1, label="OL diverges (>1 cm) @ t=0.647 s")
ax.axhline(0.01, color="gray", ls=":", lw=1, label="1 cm threshold")
ax.set_xlabel("Time [s]")
ax.set_ylabel("3D position error [m]")
ax.set_title("Position Error: Open-Loop vs Closed-Loop (log scale)")
ax.legend(fontsize=8)
ax.grid(True, which="both", alpha=0.4)
ax.set_ylim(1e-5, 50)

# Bottom: closed-loop only, linear scale, full detail
ax = axes[1]
ax.plot(t_cl, errs_cl * 1000, "g-", lw=1.5)
ax.set_xlabel("Time [s]")
ax.set_ylabel("Closed-loop 3D error [mm]")
ax.set_title(
    f"Closed-Loop Error Detail  (RMS = {np.sqrt(np.mean(errs_cl**2))*1000:.1f} mm)"
)
ax.grid(True, alpha=0.4)

plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_errors.png"), dpi=150)
print("Saved assignment4_errors.png")

# ── Figure 8: Flatness desired vs controller commanded actions ───────────────
# This is the key comparison: how closely does the geometric controller
# reproduce the thrust and torques predicted by differential flatness?
fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True)

axes[0].plot(t_cl, cl["ref_thrust"], "b-", lw=1.5, label="Flatness desired")
axes[0].plot(t_cl, cl["cmd_thrust"], "g--", lw=1.2, label="Controller commanded")
axes[0].set_ylabel("Thrust [N]")
axes[0].axhline(0.027 * 9.81, color="r", ls=":", lw=1, label="Hover")
axes[0].legend(fontsize=8)
axes[0].grid(True, alpha=0.4)

for i, axis in enumerate(["x", "y", "z"]):
    axes[i + 1].plot(t_cl, cl[f"ref_t{axis}"], "b-", lw=1.5, label="Flatness desired")
    axes[i + 1].plot(
        t_cl, cl[f"cmd_t{axis}"], "g--", lw=1.2, label="Controller commanded"
    )
    axes[i + 1].set_ylabel(f"τ{axis} [Nm]")
    axes[i + 1].legend(fontsize=8)
    axes[i + 1].grid(True, alpha=0.4)

axes[-1].set_xlabel("Time [s]")
axes[0].set_title(
    "Desired Actions (Differential Flatness) vs Commanded Actions (Geometric Controller)"
)
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_actions_comparison.png"), dpi=150)
print("Saved assignment4_actions_comparison.png")

# ── Figure 7: Planned trajectory derivatives (pos / vel / acc) ───────────────
fig, axes = plt.subplots(3, 3, figsize=(14, 9), sharex=True)
deriv_data = {
    "Position [m]": ("x", "y", "z"),
    "Velocity [m/s]": ("vx", "vy", "vz"),
    "Acceleration [m/s²]": ("ax", "ay", "az"),
}
for row, (ylabel, keys) in enumerate(deriv_data.items()):
    for col, key in enumerate(keys):
        ax = axes[row][col]
        ax.plot(t_p, planned[key], lw=1.2)
        ax.set_ylabel(f"{key}")
        ax.grid(True, alpha=0.4)
        if row == 0:
            ax.set_title(key)
    axes[row][0].set_ylabel(ylabel)
for col in range(3):
    axes[-1][col].set_xlabel("Time [s]")
fig.suptitle("Planned Trajectory Derivatives")
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_derivatives.png"), dpi=150)
print("Saved assignment4_derivatives.png")

plt.show()


# ── Summary statistics ────────────────────────────────────────────────────────
def rms(v):
    return np.sqrt(np.mean(v**2))


print("\n── Summary ──────────────────────────────────────────────────────────")
print("Planned trajectory:")
print(f"  Duration: {t_p[-1]:.1f} s,  {len(t_p)} samples at 1 kHz")
print(
    f"  Max speed:       {np.sqrt(planned['vx']**2+planned['vy']**2+planned['vz']**2).max():.3f} m/s"
)
print(
    f"  Max thrust:      {planned['thrust'].max():.4f} N   (hover = {0.027*9.81:.4f} N)"
)
print(
    f"  Max |torque|:    {np.sqrt(planned['tx']**2+planned['ty']**2+planned['tz']**2).max():.2e} Nm"
)
print(
    f"  Max |omega|:     {np.sqrt(planned['ox']**2+planned['oy']**2+planned['oz']**2).max():.3f} rad/s"
)

ep_cl = np.sqrt(
    (cl["ref_x"] - cl["sim_x"]) ** 2
    + (cl["ref_y"] - cl["sim_y"]) ** 2
    + (cl["ref_z"] - cl["sim_z"]) ** 2
)
print("\nClosed-loop:")
print(f"  RMS 3-D position error: {rms(ep_cl):.4f} m")
print(f"  Max 3-D position error: {ep_cl.max():.4f} m")
