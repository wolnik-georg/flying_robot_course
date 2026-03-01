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

# ── Load CSVs for both modes ────────────────────────────────────────────────


def load(name):
    path = os.path.join(DATA_DIR, name)
    if not os.path.exists(path):
        raise FileNotFoundError(f"Missing: {path}")
    return np.genfromtxt(path, delimiter=",", names=True)


classic = {
    "planned": load("assignment4_planned_classic.csv"),
    "ol": load("assignment4_openloop_classic.csv"),
    "cl": load("assignment4_closedloop_classic.csv"),
}
safety = {
    "planned": load("assignment4_planned_safety.csv"),
    "ol": load("assignment4_openloop_safety.csv"),
    "cl": load("assignment4_closedloop_safety.csv"),
}

# Use classic mode for default t vectors
t_p = classic["planned"]["t"]
t_ol = classic["ol"]["t"]
t_cl = classic["cl"]["t"]

# ── Figure 1: 3-D trajectory comparison ─────────────────────────────────────
fig = plt.figure(figsize=(12, 7))
ax = fig.add_subplot(111, projection="3d")

# Classic mode
ax.plot(
    classic["planned"]["x"],
    classic["planned"]["y"],
    classic["planned"]["z"],
    "b-",
    lw=2,
    label="Planned (classic)",
)
ax.plot(
    classic["cl"]["sim_x"],
    classic["cl"]["sim_y"],
    classic["cl"]["sim_z"],
    "g-",
    lw=1.5,
    alpha=0.85,
    label="Closed-loop (classic)",
)
ax.plot(
    classic["ol"]["sim_x"],
    classic["ol"]["sim_y"],
    classic["ol"]["sim_z"],
    "r--",
    lw=1.2,
    alpha=0.8,
    label="Open-loop (classic)",
)
# Safety mode
ax.plot(
    safety["planned"]["x"],
    safety["planned"]["y"],
    safety["planned"]["z"],
    "c-",
    lw=2,
    label="Planned (safety)",
)
ax.plot(
    safety["cl"]["sim_x"],
    safety["cl"]["sim_y"],
    safety["cl"]["sim_z"],
    "m-",
    lw=1.5,
    alpha=0.85,
    label="Closed-loop (safety)",
)
ax.plot(
    safety["ol"]["sim_x"],
    safety["ol"]["sim_y"],
    safety["ol"]["sim_z"],
    "y--",
    lw=1.2,
    alpha=0.8,
    label="Open-loop (safety)",
)

# Waypoints from planned trajectory start/end of each segment (classic)
n_seg = 8
seg_dur = t_p[-1] / n_seg
for k in range(n_seg + 1):
    t_wp = k * seg_dur
    idx = np.searchsorted(t_p, t_wp)
    idx = min(idx, len(t_p) - 1)
    ax.scatter(
        classic["planned"]["x"][idx],
        classic["planned"]["y"][idx],
        classic["planned"]["z"][idx],
        c="b",
        s=50,
        zorder=5,
    )

ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_zlabel("z [m]")
ax.set_title("Assignment 4 – 3D Trajectory Comparison")
ax.legend()
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_3d_comparison.png"), dpi=150)
print("Saved assignment4_3d_comparison.png")

# ── Figure 2: Position tracking (closed-loop) ────────────────────────────────
# Example for position tracking (closed-loop)
fig, axes = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
labels = ["x", "y", "z"]
for i, ax in enumerate(axes):
    # Classic
    ref_c = [classic["cl"]["ref_x"], classic["cl"]["ref_y"], classic["cl"]["ref_z"]][i]
    sim_c = [classic["cl"]["sim_x"], classic["cl"]["sim_y"], classic["cl"]["sim_z"]][i]
    ax.plot(t_cl, ref_c, "b-", lw=1.5, label="Reference (classic)")
    ax.plot(t_cl, sim_c, "g-", lw=1, label="Simulated (classic)")
    # Safety
    ref_s = [safety["cl"]["ref_x"], safety["cl"]["ref_y"], safety["cl"]["ref_z"]][i]
    sim_s = [safety["cl"]["sim_x"], safety["cl"]["sim_y"], safety["cl"]["sim_z"]][i]
    ax.plot(t_cl, ref_s, "c--", lw=1.5, label="Reference (safety)")
    ax.plot(t_cl, sim_s, "m--", lw=1, label="Simulated (safety)")
    ax.set_ylabel(f"{labels[i]} [m]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.4)
axes[-1].set_xlabel("Time [s]")
axes[0].set_title("Closed-Loop Position Tracking: Classic vs Safety")
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_position_cl_comparison.png"), dpi=150)
print("Saved assignment4_position_cl_comparison.png")

# ── Figure 3: Velocity tracking (closed-loop) ────────────────────────────────
# Example for velocity tracking (closed-loop)
fig, axes = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
vlabels = ["vx", "vy", "vz"]
for i, ax in enumerate(axes):
    # Classic
    ref_c = [classic["cl"]["ref_vx"], classic["cl"]["ref_vy"], classic["cl"]["ref_vz"]][
        i
    ]
    sim_c = [classic["cl"]["sim_vx"], classic["cl"]["sim_vy"], classic["cl"]["sim_vz"]][
        i
    ]
    ax.plot(t_cl, ref_c, "b-", lw=1.5, label="Reference (classic)")
    ax.plot(t_cl, sim_c, "g-", lw=1, label="Simulated (classic)")
    # Safety
    ref_s = [safety["cl"]["ref_vx"], safety["cl"]["ref_vy"], safety["cl"]["ref_vz"]][i]
    sim_s = [safety["cl"]["sim_vx"], safety["cl"]["sim_vy"], safety["cl"]["sim_vz"]][i]
    ax.plot(t_cl, ref_s, "c--", lw=1.5, label="Reference (safety)")
    ax.plot(t_cl, sim_s, "m--", lw=1, label="Simulated (safety)")
    ax.set_ylabel(f"{vlabels[i]} [m/s]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.4)
axes[-1].set_xlabel("Time [s]")
axes[0].set_title("Closed-Loop Velocity Tracking: Classic vs Safety")
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_velocity_cl_comparison.png"), dpi=150)
print("Saved assignment4_velocity_cl_comparison.png")

# ── Figure 4: Planned thrust and torques ─────────────────────────────────────
# Planned thrust and torques comparison (fix NameError)
fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
# Classic
axes[0].plot(t_p, classic["planned"]["thrust"], "b-", lw=1.5, label="Thrust (classic)")
axes[1].plot(t_p, classic["planned"]["tx"], "b-", lw=1.5, label="τx (classic)")
axes[2].plot(t_p, classic["planned"]["ty"], "b-", lw=1.5, label="τy (classic)")
axes[3].plot(t_p, classic["planned"]["tz"], "b-", lw=1.5, label="τz (classic)")
# Safety
axes[0].plot(t_p, safety["planned"]["thrust"], "c--", lw=1.5, label="Thrust (safety)")
axes[1].plot(t_p, safety["planned"]["tx"], "c--", lw=1.5, label="τx (safety)")
axes[2].plot(t_p, safety["planned"]["ty"], "c--", lw=1.5, label="τy (safety)")
axes[3].plot(t_p, safety["planned"]["tz"], "c--", lw=1.5, label="τz (safety)")
for i, label in enumerate(["Thrust [N]", "τx [Nm]", "τy [Nm]", "τz [Nm]"]):
    axes[i].set_ylabel(label)
    axes[i].grid(True, alpha=0.4)
    axes[i].legend()
axes[-1].set_xlabel("Time [s]")
axes[0].set_title("Planned Thrust and Torques: Classic vs Safety")
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_actions_comparison.png"), dpi=150)
print("Saved assignment4_actions_comparison.png")

# ── Figure 5: Planned angular velocity ───────────────────────────────────────
# Planned angular velocity comparison
fig, axes = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
omega_labels = ["ωx", "ωy", "ωz"]
omega_keys = ["ox", "oy", "oz"]
for i, ax in enumerate(axes):
    # Classic
    ax.plot(
        t_p,
        classic["planned"][omega_keys[i]],
        "b-",
        lw=1.5,
        label=f"{omega_labels[i]} (classic)",
    )
    # Safety
    ax.plot(
        t_p,
        safety["planned"][omega_keys[i]],
        "c--",
        lw=1.5,
        label=f"{omega_labels[i]} (safety)",
    )
    ax.set_ylabel(f"{omega_labels[i]} [rad/s]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.4)
axes[-1].set_xlabel("Time [s]")
axes[0].set_title("Planned Angular Velocity: Classic vs Safety")
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_omega_comparison.png"), dpi=150)
print("Saved assignment4_omega_comparison.png")

# ── Figure 6: Position error comparison ─────────────────────────────────────
# Position error comparison (classic vs safety)
fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=False)
# Open-loop errors
errs_ol_classic = np.sqrt(
    (classic["ol"]["ref_x"] - classic["ol"]["sim_x"]) ** 2
    + (classic["ol"]["ref_y"] - classic["ol"]["sim_y"]) ** 2
    + (classic["ol"]["ref_z"] - classic["ol"]["sim_z"]) ** 2
)
errs_ol_safety = np.sqrt(
    (safety["ol"]["ref_x"] - safety["ol"]["sim_x"]) ** 2
    + (safety["ol"]["ref_y"] - safety["ol"]["sim_y"]) ** 2
    + (safety["ol"]["ref_z"] - safety["ol"]["sim_z"]) ** 2
)
# Closed-loop errors
errs_cl_classic = np.sqrt(
    (classic["cl"]["ref_x"] - classic["cl"]["sim_x"]) ** 2
    + (classic["cl"]["ref_y"] - classic["cl"]["sim_y"]) ** 2
    + (classic["cl"]["ref_z"] - classic["cl"]["sim_z"]) ** 2
)
errs_cl_safety = np.sqrt(
    (safety["cl"]["ref_x"] - safety["cl"]["sim_x"]) ** 2
    + (safety["cl"]["ref_y"] - safety["cl"]["sim_y"]) ** 2
    + (safety["cl"]["ref_z"] - safety["cl"]["sim_z"]) ** 2
)
# Top: full timeline, both modes
ax = axes[0]
ax.semilogy(
    t_ol, np.maximum(errs_ol_classic, 1e-5), "r-", lw=1.2, label="Open-loop (classic)"
)
ax.semilogy(
    t_ol, np.maximum(errs_ol_safety, 1e-5), "y--", lw=1.2, label="Open-loop (safety)"
)
ax.semilogy(
    t_cl, np.maximum(errs_cl_classic, 1e-5), "g-", lw=1.5, label="Closed-loop (classic)"
)
ax.semilogy(
    t_cl, np.maximum(errs_cl_safety, 1e-5), "m--", lw=1.5, label="Closed-loop (safety)"
)
ax.axhline(0.01, color="gray", ls=":", lw=1, label="1 cm threshold")
ax.set_xlabel("Time [s]")
ax.set_ylabel("3D position error [m]")
ax.set_title("Position Error: Classic vs Safety (log scale)")
ax.legend(fontsize=8)
ax.grid(True, which="both", alpha=0.4)
ax.set_ylim(1e-5, 50)
# Bottom: closed-loop only, linear scale, full detail
ax = axes[1]
ax.plot(t_cl, errs_cl_classic * 1000, "g-", lw=1.5, label="Closed-loop (classic)")
ax.plot(t_cl, errs_cl_safety * 1000, "m--", lw=1.5, label="Closed-loop (safety)")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Closed-loop 3D error [mm]")
ax.set_title(
    f"Closed-Loop Error Detail  (Classic RMS = {np.sqrt(np.mean(errs_cl_classic**2))*1000:.1f} mm, Safety RMS = {np.sqrt(np.mean(errs_cl_safety**2))*1000:.1f} mm)"
)
ax.legend(fontsize=8)
ax.grid(True, alpha=0.4)
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_errors_comparison.png"), dpi=150)
print("Saved assignment4_errors_comparison.png")


# ── Figure 7: Planned trajectory derivatives (pos / vel / acc) ───────────────
# Planned trajectory derivatives comparison
fig, axes = plt.subplots(3, 3, figsize=(16, 9), sharex=True)
deriv_data = {
    "Position [m]": ("x", "y", "z"),
    "Velocity [m/s]": ("vx", "vy", "vz"),
    "Acceleration [m/s²]": ("ax", "ay", "az"),
}
for row, (ylabel, keys) in enumerate(deriv_data.items()):
    for col, key in enumerate(keys):
        ax = axes[row][col]
        # Classic
        ax.plot(t_p, classic["planned"][key], "b-", lw=1.2, label="Classic")
        # Safety
        ax.plot(t_p, safety["planned"][key], "c--", lw=1.2, label="Safety")
        ax.set_ylabel(f"{key}")
        ax.grid(True, alpha=0.4)
        ax.legend()
        if row == 0:
            ax.set_title(key)
    axes[row][0].set_ylabel(ylabel)
for col in range(3):
    axes[-1][col].set_xlabel("Time [s]")
fig.suptitle("Planned Trajectory Derivatives: Classic vs Safety")
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_derivatives_comparison.png"), dpi=150)
print("Saved assignment4_derivatives_comparison.png")

plt.show()

# ── Enhanced Safety Visualization ───────────────────────────────────────────
import matplotlib.animation as animation

# Safety limits (example values, adjust as needed)
SAFETY_LIMITS = {
    "min_altitude": 0.2,
    "max_altitude": 2.0,
    "max_speed": 2.5,
    "x_min": -2.0,
    "x_max": 2.0,
    "y_min": -2.0,
    "y_max": 2.0,
}


# Compute safety status for each timestep
def compute_safety_status(x, y, z, vx, vy, vz):
    altitude_ok = (z >= SAFETY_LIMITS["min_altitude"]) and (
        z <= SAFETY_LIMITS["max_altitude"]
    )
    speed = np.sqrt(vx**2 + vy**2 + vz**2)
    speed_ok = speed <= SAFETY_LIMITS["max_speed"]
    geofence_ok = (
        (x >= SAFETY_LIMITS["x_min"])
        and (x <= SAFETY_LIMITS["x_max"])
        and (y >= SAFETY_LIMITS["y_min"])
        and (y <= SAFETY_LIMITS["y_max"])
    )
    if altitude_ok and speed_ok and geofence_ok:
        return "normal"
    elif not altitude_ok or not speed_ok or not geofence_ok:
        return "warning"
    else:
        return "emergency"


# Color map for safety status
SAFETY_COLORS = {"normal": "#2ecc40", "warning": "#ffdc00", "emergency": "#ff4136"}

# Animated 3D trajectory with safety overlays
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection="3d")

# Draw geofence box
gf = SAFETY_LIMITS
for s, e in [
    (
        [gf["x_min"], gf["y_min"], gf["min_altitude"]],
        [gf["x_max"], gf["y_min"], gf["min_altitude"]],
    ),
    (
        [gf["x_max"], gf["y_min"], gf["min_altitude"]],
        [gf["x_max"], gf["y_max"], gf["min_altitude"]],
    ),
    (
        [gf["x_max"], gf["y_max"], gf["min_altitude"]],
        [gf["x_min"], gf["y_max"], gf["min_altitude"]],
    ),
    (
        [gf["x_min"], gf["y_max"], gf["min_altitude"]],
        [gf["x_min"], gf["y_min"], gf["min_altitude"]],
    ),
    (
        [gf["x_min"], gf["y_min"], gf["max_altitude"]],
        [gf["x_max"], gf["y_min"], gf["max_altitude"]],
    ),
    (
        [gf["x_max"], gf["y_min"], gf["max_altitude"]],
        [gf["x_max"], gf["y_max"], gf["max_altitude"]],
    ),
    (
        [gf["x_max"], gf["y_max"], gf["max_altitude"]],
        [gf["x_min"], gf["y_max"], gf["max_altitude"]],
    ),
    (
        [gf["x_min"], gf["y_max"], gf["max_altitude"]],
        [gf["x_min"], gf["y_min"], gf["max_altitude"]],
    ),
    (
        [gf["x_min"], gf["y_min"], gf["min_altitude"]],
        [gf["x_min"], gf["y_min"], gf["max_altitude"]],
    ),
    (
        [gf["x_max"], gf["y_min"], gf["min_altitude"]],
        [gf["x_max"], gf["y_min"], gf["max_altitude"]],
    ),
    (
        [gf["x_max"], gf["y_max"], gf["min_altitude"]],
        [gf["x_max"], gf["y_max"], gf["max_altitude"]],
    ),
    (
        [gf["x_min"], gf["y_max"], gf["min_altitude"]],
        [gf["x_min"], gf["y_max"], gf["max_altitude"]],
    ),
]:
    ax.plot([s[0], e[0]], [s[1], e[1]], [s[2], e[2]], color="gray", lw=1, alpha=0.5)

# Altitude bands
ax.axhline(gf["min_altitude"], color="blue", ls=":", lw=1, label="Min altitude")
ax.axhline(gf["max_altitude"], color="purple", ls=":", lw=1, label="Max altitude")

# Safety visualization (use classic closed-loop by default)
sim_x = classic["cl"]["sim_x"]
sim_y = classic["cl"]["sim_y"]
sim_z = classic["cl"]["sim_z"]
sim_vx = classic["cl"]["sim_vx"]
sim_vy = classic["cl"]["sim_vy"]
sim_vz = classic["cl"]["sim_vz"]
safety_status = [
    compute_safety_status(x, y, z, vx, vy, vz)
    for x, y, z, vx, vy, vz in zip(sim_x, sim_y, sim_z, sim_vx, sim_vy, sim_vz)
]


# Animate trajectory with color-coded safety
def update(num, sim_x, sim_y, sim_z, safety_status, line):
    line.set_data(sim_x[:num], sim_y[:num])
    line.set_3d_properties(sim_z[:num])
    line.set_color(SAFETY_COLORS[safety_status[num - 1]])
    return (line,)


(line,) = ax.plot([], [], [], lw=2)
ax.set_xlim(gf["x_min"], gf["x_max"])
ax.set_ylim(gf["y_min"], gf["y_max"])
ax.set_zlim(gf["min_altitude"], gf["max_altitude"])
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_zlabel("z [m]")
ax.set_title("Animated 3D Trajectory with Safety Status")

# Safety event timeline
fig, ax = plt.subplots(figsize=(12, 2))
status_numeric = [
    0 if s == "normal" else 1 if s == "warning" else 2 for s in safety_status
]
ax.plot(t_cl, status_numeric, drawstyle="steps-post", lw=2)
ax.set_yticks([0, 1, 2])
ax.set_yticklabels(["Normal", "Warning", "Emergency"])
ax.set_xlabel("Time [s]")
ax.set_title("Safety Event Timeline")
ax.grid(True, alpha=0.4)
plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, "assignment4_safety_timeline.png"), dpi=150)
print("Saved assignment4_safety_timeline.png")

# Actions comparison (flatness vs controller)
fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
# Classic
axes[0].plot(
    t_cl, classic["cl"]["ref_thrust"], "b-", lw=1.5, label="Flatness desired (classic)"
)
axes[0].plot(
    t_cl,
    classic["cl"]["cmd_thrust"],
    "g--",
    lw=1.2,
    label="Controller commanded (classic)",
)
axes[0].set_ylabel("Thrust [N]")
axes[0].axhline(0.027 * 9.81, color="r", ls=":", lw=1, label="Hover")
axes[0].legend(fontsize=8)
axes[0].grid(True, alpha=0.4)
for i, axis in enumerate(["x", "y", "z"]):
    axes[i + 1].plot(
        t_cl,
        classic["cl"][f"ref_t{axis}"],
        "b-",
        lw=1.5,
        label=f"Flatness desired (classic)",
    )
    axes[i + 1].plot(
        t_cl,
        classic["cl"][f"cmd_t{axis}"],
        "g--",
        lw=1.2,
        label=f"Controller commanded (classic)",
    )
    axes[i + 1].set_ylabel(f"τ{axis} [Nm]")
    axes[i + 1].legend(fontsize=8)
    axes[i + 1].grid(True, alpha=0.4)
# Safety
axes[0].plot(
    t_cl, safety["cl"]["ref_thrust"], "c-", lw=1.5, label="Flatness desired (safety)"
)
axes[0].plot(
    t_cl,
    safety["cl"]["cmd_thrust"],
    "m--",
    lw=1.2,
    label="Controller commanded (safety)",
)
for i, axis in enumerate(["x", "y", "z"]):
    axes[i + 1].plot(
        t_cl,
        safety["cl"][f"ref_t{axis}"],
        "c-",
        lw=1.5,
        label=f"Flatness desired (safety)",
    )
    axes[i + 1].plot(
        t_cl,
        safety["cl"][f"cmd_t{axis}"],
        "m--",
        lw=1.2,
        label=f"Controller commanded (safety)",
    )
axes[-1].set_xlabel("Time [s]")
axes[0].set_title("Desired vs Commanded Actions: Classic vs Safety")
plt.tight_layout()
plt.savefig(
    os.path.join(IMG_DIR, "assignment4_actions_controller_comparison.png"), dpi=150
)
print("Saved assignment4_actions_controller_comparison.png")


# ── Assignment 5: Safe-space trajectory plots (if available) ──────────────
def try_plot_assignment5(mode_name):
    try:
        planned = load(f"assignment5_planned_{mode_name}.csv")
        cl = load(f"assignment5_closedloop_{mode_name}.csv")
    except FileNotFoundError:
        print(f"Assignment5 files for mode='{mode_name}' not found, skipping.")
        return

    print(f"Plotting Assignment5 ({mode_name}) results...")

    # Safety box used by assignment5
    x_min, x_max = -0.5, 0.5
    y_min, y_max = -0.5, 0.5
    z_min, z_max = 0.0, 0.30

    # 3D comparison
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(
        planned["x"],
        planned["y"],
        planned["z"],
        "b-",
        lw=2,
        label="Planned (assignment5)",
    )
    ax.plot(
        cl["sim_x"],
        cl["sim_y"],
        cl["sim_z"],
        "r--",
        lw=1.5,
        label="Closed-loop (assignment5)",
    )

    # draw safety box edges
    gf = dict(
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
        min_altitude=z_min,
        max_altitude=z_max,
    )
    for s, e in [
        (
            [gf["x_min"], gf["y_min"], gf["min_altitude"]],
            [gf["x_max"], gf["y_min"], gf["min_altitude"]],
        ),
        (
            [gf["x_max"], gf["y_min"], gf["min_altitude"]],
            [gf["x_max"], gf["y_max"], gf["min_altitude"]],
        ),
        (
            [gf["x_max"], gf["y_max"], gf["min_altitude"]],
            [gf["x_min"], gf["y_max"], gf["min_altitude"]],
        ),
        (
            [gf["x_min"], gf["y_max"], gf["min_altitude"]],
            [gf["x_min"], gf["y_min"], gf["min_altitude"]],
        ),
        (
            [gf["x_min"], gf["y_min"], gf["max_altitude"]],
            [gf["x_max"], gf["y_min"], gf["max_altitude"]],
        ),
        (
            [gf["x_max"], gf["y_min"], gf["max_altitude"]],
            [gf["x_max"], gf["y_max"], gf["max_altitude"]],
        ),
        (
            [gf["x_max"], gf["y_max"], gf["max_altitude"]],
            [gf["x_min"], gf["y_max"], gf["max_altitude"]],
        ),
        (
            [gf["x_min"], gf["y_max"], gf["max_altitude"]],
            [gf["x_min"], gf["y_min"], gf["max_altitude"]],
        ),
    ]:
        ax.plot([s[0], e[0]], [s[1], e[1]], [s[2], e[2]], color="gray", lw=1, alpha=0.6)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title(f"Assignment5 {mode_name} – planned vs closed-loop with safety box")
    ax.legend()
    plt.tight_layout()
    outpath = os.path.join(IMG_DIR, f"assignment5_3d_{mode_name}.png")
    plt.savefig(outpath, dpi=150)
    print(f"Saved {outpath}")

    # Check violations
    planned_viol = (
        (planned["x"] < x_min)
        | (planned["x"] > x_max)
        | (planned["y"] < y_min)
        | (planned["y"] > y_max)
        | (planned["z"] < z_min)
        | (planned["z"] > z_max)
    ).any()
    cl_viol = (
        (cl["sim_x"] < x_min)
        | (cl["sim_x"] > x_max)
        | (cl["sim_y"] < y_min)
        | (cl["sim_y"] > y_max)
        | (cl["sim_z"] < z_min)
        | (cl["sim_z"] > z_max)
    ).any()
    print(
        f"Assignment5 {mode_name}: planned violation={planned_viol}, closed-loop violation={cl_viol}"
    )

    # z over time plot with bounds
    fig, ax = plt.subplots(figsize=(10, 3))
    t = planned["t"]
    ax.plot(t, planned["z"], "b-", label="Planned z")
    # find matching t for closed-loop (if different sampling) use its t
    t_cl = cl["t"]
    ax.plot(t_cl, cl["sim_z"], "r--", label="Closed-loop z")
    ax.axhline(z_min, color="k", ls=":", label="z_min")
    ax.axhline(z_max, color="k", ls="--", label="z_max")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Altitude z [m]")
    ax.set_title(f"Assignment5 {mode_name} altitude vs time")
    ax.legend()
    plt.tight_layout()
    outpath = os.path.join(IMG_DIR, f"assignment5_z_{mode_name}.png")
    plt.savefig(outpath, dpi=150)
    print(f"Saved {outpath}")


try_plot_assignment5("circle")
try_plot_assignment5("figure8")
