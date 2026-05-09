#!/usr/bin/env python3
"""
Planner-only reference trajectories — visualisation
===================================================
Reads `reference.csv` plus sidecars from:

  cargo run --release --bin planning_sim -- --export-reference-only

Data per folder:
  reference.csv       — dense samples (includes qw,qx,qy,qz from body attitude)
  waypoints.csv       — discrete planner inputs (flat: yaw; SE(3): quaternion)
  segment_junctions.csv — interior segment boundary times (piecewise polynomial joins)
  planning_meta.txt     — feasibility limits used in Richter check, SE(3) jerk/snap flags
  feasibility.txt       — Richter-only numeric report (optional)

Outputs:
  reference/mode<N>/<traj>/overview.png
  reference/mode<N>/<traj>/frenet_sanity.png   (Frenet + finite-diff consistency)
  reference/cross_mode/<traj>/comparison.png   (Modes 0/1/2/3/4 overlaid where available)

Run:
  ~/.pyenv/versions/flying_robots/bin/python scripts/plot_planning_reference.py
"""

from __future__ import annotations

import glob
import os
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple

import matplotlib
import matplotlib.gridspec as mgridspec

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.lines import Line2D

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REF_ROOT = os.path.join(_ROOT, "results", "planning_sim", "reference")
CROSS_ROOT = os.path.join(_ROOT, "results", "planning_sim", "cross_mode")


def _quat_to_rot(qw: np.ndarray, qx: np.ndarray, qy: np.ndarray, qz: np.ndarray) -> Tuple[np.ndarray, ...]:
    """Quaternion [w,x,y,z] -> rotation matrix components (body->world)."""
    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r01 = 2.0 * (qx * qy - qz * qw)
    r02 = 2.0 * (qx * qz + qy * qw)
    r10 = 2.0 * (qx * qy + qz * qw)
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
    r12 = 2.0 * (qy * qz - qx * qw)
    r20 = 2.0 * (qx * qz - qy * qw)
    r21 = 2.0 * (qy * qz + qx * qw)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
    return r00, r01, r02, r10, r11, r12, r20, r21, r22


def _draw_orientation_glyphs(
    ax: Any,
    d: np.ndarray,
    *,
    color: str = "k",
    axis_len: float = 0.08,
    count: int = 14,
    alpha: float = 0.55,
) -> None:
    """Draw sparse body-axis glyphs on a 3D path from qw/qx/qy/qz columns."""
    req = {"px", "py", "pz", "qw", "qx", "qy", "qz"}
    names = set(d.dtype.names or ())
    if not req.issubset(names):
        return
    n = int(len(d["px"]))
    if n < 3:
        return
    k = max(6, min(count, n))
    idx = np.unique(np.linspace(0, n - 1, k, dtype=int))
    qw, qx, qy, qz = d["qw"][idx], d["qx"][idx], d["qy"][idx], d["qz"][idx]
    r00, _, r02, r10, _, r12, r20, _, r22 = _quat_to_rot(qw, qx, qy, qz)
    # Draw body-x and body-z axes for readability.
    ax.quiver(
        d["px"][idx], d["py"][idx], d["pz"][idx],
        r00, r10, r20,
        length=axis_len, normalize=True, color=color, alpha=alpha * 0.75, linewidth=0.6
    )
    ax.quiver(
        d["px"][idx], d["py"][idx], d["pz"][idx],
        r02, r12, r22,
        length=axis_len, normalize=True, color=color, alpha=alpha, linewidth=0.8
    )


def _euler_to_rot(roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> Tuple[np.ndarray, ...]:
    """ZYX Euler -> rotation matrix components (body->world)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    r00 = cy * cp
    r01 = cy * sp * sr - sy * cr
    r02 = cy * sp * cr + sy * sr
    r10 = sy * cp
    r11 = sy * sp * sr + cy * cr
    r12 = sy * sp * cr - cy * sr
    r20 = -sp
    r21 = cp * sr
    r22 = cp * cr
    return r00, r01, r02, r10, r11, r12, r20, r21, r22


def _waypoint_ref_indices(d: np.ndarray, wps: Optional[np.ndarray]) -> np.ndarray:
    """Map waypoint positions to nearest reference samples (ordered, unique)."""
    n = int(len(d["px"]))
    if n == 0:
        return np.array([], dtype=int)
    if wps is None or not getattr(wps, "size", 0):
        k = min(12, n)
        return np.unique(np.linspace(0, n - 1, k, dtype=int))
    rx = d["px"][:, None]
    ry = d["py"][:, None]
    rz = d["pz"][:, None]
    wx = np.atleast_1d(wps["px"])[None, :]
    wy = np.atleast_1d(wps["py"])[None, :]
    wz = np.atleast_1d(wps["pz"])[None, :]
    dist2 = (rx - wx) ** 2 + (ry - wy) ** 2 + (rz - wz) ** 2
    idx = np.argmin(dist2, axis=0)
    # Preserve order, remove duplicates from closed-loop repeated waypoint.
    return np.array(list(dict.fromkeys(int(i) for i in idx.tolist())), dtype=int)


def _plot_trajectory_3d_orientation(folder: str, d: np.ndarray, wps: Optional[np.ndarray], title: str) -> None:
    """
    Dedicated large 3D trajectory plot with attitude triads at waypoint locations.
    Triad colors: body-x red, body-y green, body-z blue.
    """
    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection="3d")

    px, py, pz = d["px"], d["py"], d["pz"]
    ax.plot(px, py, pz, color="black", lw=2.0, label="reference path")

    if wps is not None and getattr(wps, "size", 0):
        ax.scatter(
            wps["px"], wps["py"], wps["pz"],
            s=52, c="black", alpha=0.85, depthshade=True, label="waypoints"
        )

    idx = _waypoint_ref_indices(d, wps)
    names = set(d.dtype.names or ())
    if idx.size > 0:
        if {"qw", "qx", "qy", "qz"}.issubset(names):
            comps = _quat_to_rot(d["qw"][idx], d["qx"][idx], d["qy"][idx], d["qz"][idx])
        elif {"euler_roll", "euler_pitch", "euler_yaw"}.issubset(names):
            comps = _euler_to_rot(d["euler_roll"][idx], d["euler_pitch"][idx], d["euler_yaw"][idx])
        else:
            comps = None

        if comps is not None:
            r00, r01, r02, r10, r11, r12, r20, r21, r22 = comps
            # Keep triads visually consistent within each figure:
            # fixed physical glyph length derived from bbox (not per-axis, not per-vector).
            span_x = float(np.max(px) - np.min(px))
            span_y = float(np.max(py) - np.min(py))
            span_z = float(np.max(pz) - np.min(pz))
            diag = max(np.sqrt(span_x * span_x + span_y * span_y + span_z * span_z), 0.2)
            axis_len = 0.085 * diag
            # body-x (red)
            ax.quiver(px[idx], py[idx], pz[idx], r00, r10, r20,
                      length=axis_len, normalize=True, color="#d62728", linewidth=1.1, alpha=0.9)
            # body-y (green)
            ax.quiver(px[idx], py[idx], pz[idx], r01, r11, r21,
                      length=axis_len, normalize=True, color="#2ca02c", linewidth=1.1, alpha=0.9)
            # body-z (blue)
            ax.quiver(px[idx], py[idx], pz[idx], r02, r12, r22,
                      length=axis_len, normalize=True, color="#1f77b4", linewidth=1.2, alpha=0.95)

    # Keep a minimum visual Z span for nearly-flat paths so the 3D frame
    # remains interpretable (avoid "paper-thin" rendering).
    xmin, xmax = float(np.min(px)), float(np.max(px))
    ymin, ymax = float(np.min(py)), float(np.max(py))
    zmin, zmax = float(np.min(pz)), float(np.max(pz))
    span_x = xmax - xmin
    span_y = ymax - ymin
    span_z = zmax - zmin
    xy_span = max(span_x, span_y, 1e-3)
    # Ensure x/y are never singular (e.g. flip at fixed x,y).
    xy_floor = 0.18 * max(xy_span, span_z, 0.2)
    if span_x < xy_floor:
        xc = 0.5 * (xmin + xmax)
        xmin = xc - 0.5 * xy_floor
        xmax = xc + 0.5 * xy_floor
        span_x = xy_floor
    if span_y < xy_floor:
        yc = 0.5 * (ymin + ymax)
        ymin = yc - 0.5 * xy_floor
        ymax = yc + 0.5 * xy_floor
        span_y = xy_floor
    z_floor = 0.22 * xy_span
    if span_z < z_floor:
        zc = 0.5 * (zmin + zmax)
        zmin = zc - 0.5 * z_floor
        zmax = zc + 0.5 * z_floor
        span_z = z_floor
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_zlim(zmin, zmax)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title(f"3D trajectory + orientation triads (waypoints)\n{title}", fontsize=11)
    ax.grid(True, alpha=0.3)

    handles = [
        Line2D([0], [0], color="black", lw=2.0, label="reference path"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="black", markersize=7, label="waypoints"),
        Line2D([0], [0], color="#d62728", lw=2.0, label="body-x"),
        Line2D([0], [0], color="#2ca02c", lw=2.0, label="body-y"),
        Line2D([0], [0], color="#1f77b4", lw=2.0, label="body-z"),
    ]
    ax.legend(handles=handles, fontsize=9, loc="upper right")
    # Avoid perspective/depth distortion that makes identical arrows look different.
    ax.set_proj_type("ortho")
    # Equalize visual scaling across x/y/z so axis lengths are comparable.
    ax.set_box_aspect([max(span_x, 1e-3), max(span_y, 1e-3), max(span_z, 1e-3)])
    ax.view_init(elev=26, azim=235)
    plt.tight_layout()
    out = os.path.join(folder, "trajectory_3d_orientation.png")
    plt.savefig(out, dpi=160)
    plt.close()
    print(f"Saved {out}")


def _read_feasibility_se3(dir_path: str) -> Optional[str]:
    path = os.path.join(dir_path, "feasibility_se3.txt")
    if not os.path.isfile(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            lines = [ln.strip() for ln in f.readlines() if ln.strip()]
        return "  ".join(lines[:8])
    except OSError:
        return None


def _read_feasibility(dir_path: str) -> Optional[str]:
    path = os.path.join(dir_path, "feasibility.txt")
    if not os.path.isfile(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            lines = [ln.strip() for ln in f.readlines() if ln.strip() and not ln.startswith("Note")]
        return "  ".join(lines[:5])
    except OSError:
        return None


def _parse_meta(dir_path: str) -> Dict[str, str]:
    defaults = {
        "feasibility_max_thrust_N": "0.55",
        "feasibility_max_omega_rad_s": "12.0",
        "feasibility_max_torque_axis_Nm": "0.004",
        "se3_export_jerk_snap_zero": "0",
    }
    path = os.path.join(dir_path, "planning_meta.txt")
    if not os.path.isfile(path):
        return defaults
    out = dict(defaults)
    try:
        with open(path, "r", encoding="utf-8") as f:
            for ln in f:
                ln = ln.strip()
                if not ln or ln.startswith("#") or "=" not in ln:
                    continue
                k, v = ln.split("=", 1)
                out[k.strip()] = v.strip()
    except OSError:
        pass
    return out


def _load_junctions(dir_path: str) -> List[float]:
    path = os.path.join(dir_path, "segment_junctions.csv")
    if not os.path.isfile(path):
        return []
    try:
        w = np.genfromtxt(path, delimiter=",", names=True)
        if w.size == 0:
            return []
        col = w["t_junction_s"] if "t_junction_s" in w.dtype.names else w[w.dtype.names[0]]
        return [float(x) for x in np.atleast_1d(col)]
    except (OSError, ValueError, TypeError):
        return []


def _load_waypoints(dir_path: str) -> Optional[np.ndarray]:
    path = os.path.join(dir_path, "waypoints.csv")
    if not os.path.isfile(path):
        return None
    try:
        return np.genfromtxt(path, delimiter=",", names=True)
    except OSError:
        return None


def _junction_lines(ax: Any, times: Sequence[float]) -> None:
    for jt in times:
        ax.axvline(jt, color="#444444", ls=":", lw=0.95, alpha=0.55, zorder=0)


def _hardware_utilization(
    d: np.ndarray, max_t: float, max_w: float, max_tau: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    om = np.sqrt(d["omega_x"] ** 2 + d["omega_y"] ** 2 + d["omega_z"] ** 2)
    tau_inf = np.maximum(np.abs(d["torque_x"]), np.maximum(np.abs(d["torque_y"]), np.abs(d["torque_z"])))
    u_t = np.abs(d["thrust_N"]) / max(max_t, 1e-6)
    u_w = om / max(max_w, 1e-6)
    u_tau = tau_inf / max(max_tau, 1e-9)
    u_max = np.maximum(u_t, np.maximum(u_w, u_tau))
    viol_any = u_max > 1.0
    return u_t, u_w, u_tau, u_max, viol_any


def _shade_violation_windows(ax: Any, t: np.ndarray, viol_any: np.ndarray) -> None:
    if len(t) < 2 or not np.any(viol_any):
        return
    in_seg = False
    start = float(t[0])
    for i in range(len(t)):
        active = bool(viol_any[i])
        if active and not in_seg:
            in_seg = True
            start = float(t[i])
        elif (not active) and in_seg:
            in_seg = False
            ax.axvspan(start, float(t[i]), color="red", alpha=0.08, lw=0.0, zorder=0)
    if in_seg:
        ax.axvspan(start, float(t[-1]), color="red", alpha=0.08, lw=0.0, zorder=0)


def _longest_true_duration(t: np.ndarray, mask: np.ndarray) -> float:
    if len(t) < 2 or not np.any(mask):
        return 0.0
    best = 0.0
    start = None
    for i, m in enumerate(mask):
        if bool(m) and start is None:
            start = float(t[i])
        elif (not bool(m)) and start is not None:
            best = max(best, float(t[i]) - start)
            start = None
    if start is not None:
        best = max(best, float(t[-1]) - start)
    return best


def _motor_omega_sq_unclamped_np(
    thrust: np.ndarray,
    tx: np.ndarray,
    ty: np.ndarray,
    tz: np.ndarray,
    kf: float,
    kt: float,
    arm_length: float,
) -> np.ndarray:
    sqrt2 = np.sqrt(2.0)
    l_sqrt2 = arm_length / sqrt2
    thrust_term = thrust / (4.0 * max(kf, 1e-12))
    roll_term = tx / (4.0 * max(kf, 1e-12) * max(l_sqrt2, 1e-12))
    pitch_term = ty / (4.0 * max(kf, 1e-12) * max(l_sqrt2, 1e-12))
    yaw_term = tz / (4.0 * max(kt, 1e-12))
    w1 = thrust_term - roll_term - pitch_term + yaw_term
    w2 = thrust_term + roll_term - pitch_term - yaw_term
    w3 = thrust_term + roll_term + pitch_term + yaw_term
    w4 = thrust_term - roll_term + pitch_term - yaw_term
    return np.stack([w1, w2, w3, w4], axis=1)


def _plot_limits_dashboard(
    folder: str,
    d: np.ndarray,
    t: np.ndarray,
    junctions: List[float],
    title: str,
    meta: Dict[str, str],
) -> None:
    max_t = float(meta.get("feasibility_max_thrust_N", "0.55"))
    max_w = float(meta.get("feasibility_max_omega_rad_s", "12.0"))
    max_tau = float(meta.get("feasibility_max_torque_axis_Nm", "0.004"))
    kf = float(meta.get("motor_kf", "2.5e-6"))
    kt = float(meta.get("motor_kt", "1.0e-7"))
    arm = float(meta.get("motor_arm_length_m", "0.046"))
    per_motor_thrust_max = float(meta.get("per_motor_thrust_max_N", f"{max_t/4.0:.6f}"))
    omega_motor_max = float(meta.get("motor_omega_max_rad_s", str(np.sqrt(max(per_motor_thrust_max / max(kf, 1e-12), 1e-12)))))
    dthrust_dt_max = float(meta.get("proxy_dthrust_dt_max_N_s", "8.0"))
    dtau_dt_max = float(meta.get("proxy_dtau_dt_max_Nm_s", "0.25"))
    domega_dt_max = float(meta.get("proxy_domega_dt_max_rad_s2", "2500.0"))

    u_t, u_w, u_tau, u_max, viol_any = _hardware_utilization(d, max_t, max_w, max_tau)
    tau_inf = np.maximum(np.abs(d["torque_x"]), np.maximum(np.abs(d["torque_y"]), np.abs(d["torque_z"])))

    w_sq = _motor_omega_sq_unclamped_np(
        d["thrust_N"], d["torque_x"], d["torque_y"], d["torque_z"], kf, kt, arm
    )
    neg_mix = np.any(w_sq < 0.0, axis=1)
    w_sq_clamped = np.clip(w_sq, 0.0, None)
    w = np.sqrt(w_sq_clamped)
    f_motor = kf * w_sq_clamped
    u_wm = w / max(omega_motor_max, 1e-6)
    u_fm = f_motor / max(per_motor_thrust_max, 1e-9)

    dthrust_dt = np.gradient(d["thrust_N"], t)
    dtau_dt = np.gradient(tau_inf, t)
    domega_dt = np.gradient(np.max(w, axis=1), t)
    u_dthrust = np.abs(dthrust_dt) / max(dthrust_dt_max, 1e-6)
    u_dtau = np.abs(dtau_dt) / max(dtau_dt_max, 1e-9)
    u_domega = np.abs(domega_dt) / max(domega_dt_max, 1e-6)

    # Simple battery sag margin estimate (model): available thrust scales with V^2.
    # If V drops to r*V_nom, utilization scales by 1/r^2.
    peak_u = float(np.max(u_max))
    sag_r = np.linspace(0.75, 1.0, 120)
    est_u_under_sag = peak_u / np.maximum(sag_r * sag_r, 1e-6)

    fig, axes = plt.subplots(4, 2, figsize=(15, 12), sharex=True)
    fig.suptitle(f"Limits dashboard — {title}", fontsize=11)

    ax = axes[0, 0]
    ax.plot(t, u_t, label="|T|/Tmax", lw=1.1, color="C3")
    ax.plot(t, u_w, label="|ω|/ωmax", lw=1.1, color="C2")
    ax.plot(t, u_tau, label="|τ|∞/τmax", lw=1.1, color="C4")
    ax.plot(t, u_max, label="max util", lw=1.3, color="black")
    ax.axhline(1.0, color="red", ls="--", lw=1.0, alpha=0.85)
    _junction_lines(ax, junctions)
    _shade_violation_windows(ax, t, viol_any)
    ax.set_ylabel("ratio [-]")
    ax.set_title("Core control/actuation utilization")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.step(t, neg_mix.astype(float), where="post", lw=1.1, color="tab:red", label="negative ω² before clamp")
    _junction_lines(ax, junctions)
    ax.set_yticks([0.0, 1.0])
    ax.set_yticklabels(["feasible", "infeasible"])
    ax.set_title("Motor-mix feasibility over time")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    for i in range(4):
        ax.plot(t, u_wm[:, i], lw=0.9, label=f"ωm{i+1}/ωmax")
    ax.axhline(1.0, color="red", ls="--", lw=1.0, alpha=0.85)
    _junction_lines(ax, junctions)
    ax.set_ylabel("ratio [-]")
    ax.set_title("Per-motor speed usage")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    for i in range(4):
        ax.plot(t, u_fm[:, i], lw=0.9, label=f"Fm{i+1}/Fmax")
    ax.axhline(1.0, color="red", ls="--", lw=1.0, alpha=0.85)
    _junction_lines(ax, junctions)
    ax.set_ylabel("ratio [-]")
    ax.set_title("Per-motor thrust usage")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    ax = axes[2, 0]
    ax.plot(t, u_dthrust, lw=1.0, color="C3", label="|dT/dt| / proxy max")
    ax.plot(t, u_dtau, lw=1.0, color="C4", label="|d|τ|∞/dt| / proxy max")
    ax.plot(t, u_domega, lw=1.0, color="C2", label="|dωm_max/dt| / proxy max")
    ax.axhline(1.0, color="red", ls="--", lw=1.0, alpha=0.85)
    _junction_lines(ax, junctions)
    ax.set_ylabel("ratio [-]")
    ax.set_title("Rate-of-change proxy checks")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    ax = axes[2, 1]
    labels = ["core util>1", "neg mix", "dTdt>1", "dtaudt>1", "domegadt>1"]
    masks = [
        viol_any,
        neg_mix,
        u_dthrust > 1.0,
        u_dtau > 1.0,
        u_domega > 1.0,
    ]
    fracs = [100.0 * float(np.mean(m)) for m in masks]
    longest = [_longest_true_duration(t, m) for m in masks]
    y = np.arange(len(labels))
    ax.barh(y, fracs, color=["black", "tab:red", "C3", "C4", "C2"], alpha=0.75)
    for yi, (f, l) in enumerate(zip(fracs, longest)):
        ax.text(f + 0.8, yi, f"{f:.2f}% | longest {l:.3f}s", va="center", fontsize=8)
    ax.set_yticks(y)
    ax.set_yticklabels(labels, fontsize=8)
    ax.set_xlabel("active duration [% of trajectory]")
    ax.set_title("Sustained saturation / violation duration")
    ax.grid(True, axis="x", alpha=0.3)

    ax = axes[3, 0]
    ax.plot(sag_r, est_u_under_sag, lw=1.2, color="tab:purple", label="estimated peak utilization under sag")
    ax.axhline(1.0, color="red", ls="--", lw=1.0, alpha=0.85, label="limit boundary")
    ax.axvline(1.0, color="gray", ls=":", lw=0.8)
    ax.set_xlabel("battery voltage ratio r = V/V_nom [-]")
    ax.set_ylabel("estimated peak utilization [-]")
    ax.set_title("Battery-sag margin estimate (model: thrust capability ∝ V²)")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = axes[3, 1]
    ax.axis("off")
    txt = (
        f"Limits used\n"
        f"  Tmax={max_t:.3f} N, ωmax={max_w:.3f} rad/s, τmax={max_tau:.6f} Nm\n"
        f"Motor model\n"
        f"  kf={kf:.3e}, kt={kt:.3e}, arm={arm:.3f} m\n"
        f"  per-motor Fmax={per_motor_thrust_max:.4f} N, ωm_max={omega_motor_max:.1f} rad/s\n"
        f"Rate proxy limits\n"
        f"  |dT/dt|max={dthrust_dt_max:.2f} N/s\n"
        f"  |d|τ|∞/dt|max={dtau_dt_max:.4f} Nm/s\n"
        f"  |dωm/dt|max={domega_dt_max:.1f} rad/s²\n"
        f"Notes\n"
        f"  - rate and battery checks are model/proxy based\n"
        f"  - closed-loop controller clipping must be checked in closed-loop logs"
    )
    ax.text(0.02, 0.98, txt, va="top", ha="left", fontsize=8, family="monospace")

    plt.tight_layout(rect=(0, 0, 1, 0.97))
    out = os.path.join(folder, "limits_dashboard.png")
    plt.savefig(out, dpi=140)
    plt.close(fig)
    print(f"Saved {out}")


def _plot_frenet_sanity(
    folder: str,
    d: np.ndarray,
    t: np.ndarray,
    junctions: List[float],
    title: str,
    max_t: float,
    max_w: float,
    max_tau: float,
) -> None:
    """Frenet-style path scalars + numerical consistency (gradient vs CSV derivatives)."""
    px, py, pz = d["px"], d["py"], d["pz"]
    vx, vy, vz = d["vx"], d["vy"], d["vz"]
    ax_, ay, az = d["ax"], d["ay"], d["az"]

    # Frenet: κ = ‖v×a‖ / ‖v‖³ , tangential accel = (v·a)/|v|, arc length
    vvec = np.stack([vx, vy, vz], axis=1)
    avec = np.stack([ax_, ay, az], axis=1)
    cross = np.cross(vvec, avec)
    cross_n = np.linalg.norm(cross, axis=1)
    speed = np.linalg.norm(vvec, axis=1)
    eps = 1e-9
    speed_safe = np.maximum(speed, eps)
    kappa = cross_n / (speed_safe**3)
    a_t = np.sum(vvec * avec, axis=1) / speed_safe
    a_mag = np.linalg.norm(avec, axis=1)
    a_n = np.sqrt(np.maximum(a_mag * a_mag - a_t * a_t, 0.0))

    s = np.zeros_like(t)
    for i in range(1, len(t)):
        ds = 0.5 * (speed[i] + speed[i - 1]) * (t[i] - t[i - 1])
        s[i] = s[i - 1] + ds

    # Numerical sanity: np.gradient along t
    px_t = np.gradient(px, t, edge_order=2)
    py_t = np.gradient(py, t, edge_order=2)
    pz_t = np.gradient(pz, t, edge_order=2)
    err_v = np.sqrt((px_t - vx) ** 2 + (py_t - vy) ** 2 + (pz_t - vz) ** 2)
    denom_v = np.maximum(speed, np.sqrt(px_t**2 + py_t**2 + pz_t**2))
    rel_v = err_v / (denom_v + 1e-12)

    vx_t = np.gradient(vx, t, edge_order=2)
    vy_t = np.gradient(vy, t, edge_order=2)
    vz_t = np.gradient(vz, t, edge_order=2)
    err_a = np.sqrt((vx_t - ax_) ** 2 + (vy_t - ay) ** 2 + (vz_t - az) ** 2)
    denom_a = np.maximum(a_mag, np.sqrt(vx_t**2 + vy_t**2 + vz_t**2))
    rel_a = err_a / (denom_a + 1e-12)

    fig, axes = plt.subplots(2, 3, figsize=(13, 8))
    fig.suptitle(f"Frenet + numerical sanity — {title}", fontsize=11)

    ax = axes[0, 0]
    ax.plot(t, kappa, color="C0", lw=1.0)
    _junction_lines(ax, junctions)
    ax.set_ylabel("κ [1/m]")
    ax.set_title("Curvature κ = ‖v×a‖/‖v‖³")
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(t, a_t, label="a·t̂", lw=1.0)
    ax.plot(t, a_n, label="|a_n| est.", lw=1.0, alpha=0.85)
    _junction_lines(ax, junctions)
    ax.legend(fontsize=8)
    ax.set_ylabel("[m/s²]")
    ax.set_title("Tangential / normal acceleration (geometric)")
    ax.grid(True, alpha=0.3)

    ax = axes[0, 2]
    ax.plot(t, s, color="C2", lw=1.0)
    _junction_lines(ax, junctions)
    ax.set_ylabel("s [m]")
    ax.set_xlabel("t [s]")
    ax.set_title("Arc length ∫|v|dt (trapezoid)")
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.semilogy(t, rel_v + 1e-16, color="C3", lw=0.9)
    _junction_lines(ax, junctions)
    ax.set_ylabel("relative")
    ax.set_title(f"‖∇p/∇t − v‖ / scale  (max {np.nanmax(rel_v):.2e})")
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.semilogy(t, rel_a + 1e-16, color="C4", lw=0.9)
    _junction_lines(ax, junctions)
    ax.set_title(f"‖∇v/∇t − a‖ / scale  (max {np.nanmax(rel_a):.2e})")
    ax.grid(True, alpha=0.3)

    u_t, u_w, u_tau, u_max, _ = _hardware_utilization(d, max_t, max_w, max_tau)
    ax = axes[1, 2]
    ax.plot(t, u_t, lw=0.9, label="|T|/Tmax", color="C3")
    ax.plot(t, u_w, lw=0.9, label="|ω|/ωmax", color="C2")
    ax.plot(t, u_tau, lw=0.9, label="|τ|∞/τmax", color="C4")
    ax.plot(t, u_max, lw=1.2, label="max util", color="black")
    ax.axhline(1.0, color="red", ls="--", lw=0.9, alpha=0.8, label="limit")
    _junction_lines(ax, junctions)
    ax.set_title("Hardware utilization quick check")
    ax.set_ylabel("ratio [-]")
    ax.set_xlabel("t [s]")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(folder, "frenet_sanity.png")
    plt.savefig(out, dpi=140)
    plt.close()
    print(f"Saved {out}")


def _plot_kinematics_axes(
    folder: str,
    d: np.ndarray,
    title: str,
    max_t: float,
    max_w: float,
    max_tau: float,
) -> None:
    """Planned-only per-axis split for acceleration, jerk, snap."""
    t = d["t"]
    fig, axes = plt.subplots(3, 3, figsize=(14, 10), sharex=True)
    fig.suptitle(f"Kinematics per axis (planned): {title}", fontsize=11)

    rows = [
        ("Acceleration", ("ax", "ay", "az"), "[m/s²]"),
        ("Jerk", ("jx", "jy", "jz"), "[m/s³]"),
        ("Snap", ("sx", "sy", "sz"), "[m/s⁴]"),
    ]
    cols = [("x", "C0"), ("y", "C1"), ("z", "C2")]

    for r, (row_lbl, keys, units) in enumerate(rows):
        for c, (col_lbl, color) in enumerate(cols):
            ax = axes[r, c]
            _, _, _, _, viol_any = _hardware_utilization(d, max_t, max_w, max_tau)
            _shade_violation_windows(ax, t, viol_any)
            ax.plot(t, d[keys[c]], color=color, lw=1.2, label=keys[c])
            if r == 0:
                ax.set_title(f"{col_lbl}-axis", fontsize=9)
            if c == 0:
                ax.set_ylabel(f"{row_lbl}\n{units}", fontsize=8)
            if r == 2:
                ax.set_xlabel("t [s]", fontsize=8)
            ax.legend(fontsize=7, loc="upper right")
            ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=(0, 0, 1, 0.97))
    out = os.path.join(folder, "kinematics_axes.png")
    plt.savefig(out, dpi=140)
    plt.close(fig)
    print(f"Saved {out}")


def _plot_state_axes(
    folder: str,
    d: np.ndarray,
    title: str,
    max_t: float,
    max_w: float,
    max_tau: float,
) -> None:
    """Planned-only per-axis split for velocity, Euler angles, and quaternion."""
    t = d["t"]
    fig, axes = plt.subplots(3, 3, figsize=(14, 10), sharex=True)
    fig.suptitle(f"State components per axis (planned): {title}", fontsize=11)

    vel_rows = [("vx", "C0"), ("vy", "C1"), ("vz", "C2")]
    eul_rows = [("euler_roll", "C0"), ("euler_pitch", "C1"), ("euler_yaw", "C2")]
    quat_keys = [("qw", "C0"), ("qx", "C1"), ("qy", "C2")]

    # Row 1: velocity x/y/z (one per column)
    for c, (k, color) in enumerate(vel_rows):
        ax = axes[0, c]
        _, _, _, _, viol_any = _hardware_utilization(d, max_t, max_w, max_tau)
        _shade_violation_windows(ax, t, viol_any)
        ax.plot(t, d[k], color=color, lw=1.2, label=k)
        ax.set_title(f"{k}", fontsize=9)
        if c == 0:
            ax.set_ylabel("Velocity\n[m/s]", fontsize=8)
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, alpha=0.3)

    # Row 2: Euler roll/pitch/yaw (one per column)
    for c, (k, color) in enumerate(eul_rows):
        ax = axes[1, c]
        _, _, _, _, viol_any = _hardware_utilization(d, max_t, max_w, max_tau)
        _shade_violation_windows(ax, t, viol_any)
        ax.plot(t, d[k], color=color, lw=1.2, label=k)
        ax.set_title(f"{k}", fontsize=9)
        if c == 0:
            ax.set_ylabel("Euler\n[rad]", fontsize=8)
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, alpha=0.3)

    # Row 3: quaternion components (qw/qx/qy in columns; qz overlaid in last panel)
    has_q = d.dtype.names and "qw" in d.dtype.names
    for c, (k, color) in enumerate(quat_keys):
        ax = axes[2, c]
        _, _, _, _, viol_any = _hardware_utilization(d, max_t, max_w, max_tau)
        _shade_violation_windows(ax, t, viol_any)
        if has_q and k in d.dtype.names:
            ax.plot(t, d[k], color=color, lw=1.2, label=k)
            if c == 2 and "qz" in d.dtype.names:
                ax.plot(t, d["qz"], color="C3", lw=1.0, ls="--", label="qz")
            ax.set_title(f"{k}" if c < 2 else f"{k} (+ qz)", fontsize=9)
            ax.legend(fontsize=7, loc="upper right")
        else:
            ax.text(0.5, 0.5, "quaternion unavailable", ha="center", va="center",
                    transform=ax.transAxes, fontsize=8, color="gray", style="italic")
            ax.set_title("quaternion", fontsize=9)
        if c == 0:
            ax.set_ylabel("Quaternion\n[-]", fontsize=8)
        ax.set_xlabel("t [s]", fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=(0, 0, 1, 0.97))
    out = os.path.join(folder, "state_axes.png")
    plt.savefig(out, dpi=140)
    plt.close(fig)
    print(f"Saved {out}")


def _plot_reference_csv(csv_path: str) -> None:
    d = np.genfromtxt(csv_path, delimiter=",", names=True)
    t = d["t"]
    folder = os.path.dirname(csv_path)
    rel = os.path.relpath(folder, start=os.path.join(_ROOT, "results", "planning_sim"))
    mode_traj = rel.replace("reference" + os.sep, "").replace(os.sep, " / ")

    meta = _parse_meta(folder)
    feas_txt = _read_feasibility(folder)
    feas_se3 = _read_feasibility_se3(folder)
    title_extra = ""
    if feas_txt:
        title_extra += f"\n{feas_txt}"
    if feas_se3:
        title_extra += f"\n{feas_se3}"
    junctions = _load_junctions(folder)
    wps = _load_waypoints(folder)

    max_t = float(meta.get("feasibility_max_thrust_N", "0.55"))
    max_w = float(meta.get("feasibility_max_omega_rad_s", "12.0"))
    max_tau = float(meta.get("feasibility_max_torque_axis_Nm", "0.004"))
    se3_zero = meta.get("se3_export_jerk_snap_zero", "0") == "1"

    px, py, pz = d["px"], d["py"], d["pz"]
    vx = np.sqrt(d["vx"] ** 2 + d["vy"] ** 2 + d["vz"] ** 2)
    axm = np.sqrt(d["ax"] ** 2 + d["ay"] ** 2 + d["az"] ** 2)
    jm = np.sqrt(d["jx"] ** 2 + d["jy"] ** 2 + d["jz"] ** 2)
    sm = np.sqrt(d["sx"] ** 2 + d["sy"] ** 2 + d["sz"] ** 2)
    om = np.sqrt(d["omega_x"] ** 2 + d["omega_y"] ** 2 + d["omega_z"] ** 2)
    am = np.sqrt(d["angacc_x"] ** 2 + d["angacc_y"] ** 2 + d["angacc_z"] ** 2)

    has_q = d.dtype.names and "qw" in d.dtype.names

    fig = plt.figure(figsize=(14, 28))
    fig.suptitle(f"Planner-only reference: {mode_traj}{title_extra}", fontsize=11, y=0.995)

    nrow, ncol = 9, 2

    gs = mgridspec.GridSpec(nrow, ncol, figure=fig)

    def sp(idx: int) -> Any:
        row, col = divmod(idx - 1, ncol)
        return fig.add_subplot(gs[row, col])

    # 1 — 3D
    ax0 = fig.add_subplot(gs[0, 0], projection="3d")
    ax0.plot(px, py, pz, color="C0", lw=1.2, label="reference")
    if wps is not None and wps.size:
        ax0.scatter(
            wps["px"],
            wps["py"],
            wps["pz"],
            s=36,
            c="darkred",
            depthshade=True,
            label="waypoints",
        )
    ax0.set_xlabel("x [m]", fontsize=8)
    ax0.set_ylabel("y [m]", fontsize=8)
    ax0.set_zlabel("z [m]", fontsize=8)
    ax0.set_title("Position (world)", fontsize=9)
    ax0.legend(fontsize=7, loc="upper right")
    ax0.tick_params(labelsize=7)
    _draw_orientation_glyphs(ax0, d, color="black", axis_len=0.07, count=12, alpha=0.45)
    # Mark 3D points where any hardware utilization exceeds 1.0.
    _, _, _, _, viol_any = _hardware_utilization(d, max_t, max_w, max_tau)
    if np.any(viol_any):
        ax0.scatter(px[viol_any], py[viol_any], pz[viol_any], s=8, c="red", alpha=0.35, label="limit exceed")

    ax = sp(2)
    ax.plot(t, px, label="px", lw=1.0)
    ax.plot(t, py, label="py", lw=1.0)
    ax.plot(t, pz, label="pz", lw=1.0)
    _junction_lines(ax, junctions)
    ax.set_ylabel("pos [m]")
    ax.legend(fontsize=7, ncol=3, loc="upper right")
    ax.set_title("Position vs t (+ segment junctions)", fontsize=9)
    ax.grid(True, alpha=0.3)

    ax = sp(3)
    ax.plot(t, vx, color="C1", lw=1.2, label="|v|")
    ax.plot(t, d["vx"], ls=":", lw=0.8, alpha=0.7, label="vx")
    ax.plot(t, d["vy"], ls=":", lw=0.8, alpha=0.7, label="vy")
    ax.plot(t, d["vz"], ls=":", lw=0.8, alpha=0.7, label="vz")
    _junction_lines(ax, junctions)
    ax.set_ylabel("[m/s]")
    ax.legend(fontsize=7, ncol=2)
    ax.set_title("Velocity", fontsize=9)
    ax.grid(True, alpha=0.3)

    ax = sp(4)
    ax.plot(t, axm, label="|a|", lw=1.3, color="C0")
    ax.plot(t, jm,  label="|j|", lw=1.3, color="C1")
    ax.plot(t, sm,  label="|s|", lw=1.1, color="C2", alpha=0.8)
    ax.plot(t, d["ax"], ls=":", lw=0.8, alpha=0.65, color="C0", label="ax")
    ax.plot(t, d["ay"], ls=":", lw=0.8, alpha=0.65, color="C3", label="ay")
    ax.plot(t, d["az"], ls=":", lw=0.8, alpha=0.65, color="C4", label="az")
    _junction_lines(ax, junctions)
    ax.set_ylabel("[SI]")
    ax.legend(fontsize=7, ncol=3)
    ttl = "|acc|, |jerk|, |snap| (solid) + acc components (dotted)"
    if se3_zero:
        ttl += " — legacy: j/s zeros"
    elif meta.get("planner_kind") == "Se3Trajectory":
        ttl += " — SE(3): j,s from pos spline"
    ax.set_title(ttl, fontsize=9)
    ax.grid(True, alpha=0.3)

    ax = sp(5)
    ax.plot(t, d["yaw"], label="ψ", lw=1.0)
    ax.plot(t, d["yaw_dot"], label="ψ̇", lw=1.0)
    ax.plot(t, d["yaw_ddot"], label="ψ̈", lw=1.0)
    _junction_lines(ax, junctions)
    ax.set_ylabel("[rad, rad/s, rad/s²]")
    ax.legend(fontsize=7)
    ax.set_title("Flat yaw and derivatives", fontsize=9)
    ax.grid(True, alpha=0.3)

    ax = sp(6)
    ax.plot(t, d["euler_roll"], label="roll", lw=1.0)
    ax.plot(t, d["euler_pitch"], label="pitch", lw=1.0)
    ax.plot(t, d["euler_yaw"], label="yaw", lw=1.0)
    _junction_lines(ax, junctions)
    ax.set_ylabel("[rad]")
    ax.legend(fontsize=7)
    ax.set_title("Euler (singular near ±90° pitch — see quaternion row)", fontsize=9)
    ax.grid(True, alpha=0.3)

    ax = sp(7)
    if has_q:
        ax.plot(t, d["qw"], label="qw", lw=1.0)
        ax.plot(t, d["qx"], label="qx", lw=1.0)
        ax.plot(t, d["qy"], label="qy", lw=1.0)
        ax.plot(t, d["qz"], label="qz", lw=1.0)
        _junction_lines(ax, junctions)
        ax.set_ylabel("unit quaternion")
        ax.legend(fontsize=7, ncol=2)
        ax.set_title("Quaternion (body attitude, continuous)", fontsize=9)
    else:
        ax.text(0.5, 0.5, "No qw,qx,qy,qz in CSV (re-export reference)", ha="center", va="center", fontsize=10)
        ax.axis("off")
    if has_q:
        ax.grid(True, alpha=0.3)

    ax = sp(8)
    ax.plot(t, d["thrust_N"], color="C3", lw=1.2, label="thrust")
    ax.axhline(max_t, color="red", ls="--", lw=1.0, alpha=0.85, label=f"feas. max {max_t:.2f} N")
    ax.axhline(-max_t, color="red", ls="--", lw=0.7, alpha=0.45)
    _junction_lines(ax, junctions)
    ax.set_ylabel("N")
    ax.legend(fontsize=7)
    ax.set_title("Collective thrust vs feasibility band (same limits as Richter/SE(3) checks)", fontsize=9)
    ax.grid(True, alpha=0.3)

    ax = sp(9)
    ax.plot(t, om, color="C2", lw=1.2, label="|ω|")
    ax.plot(t, d["omega_x"], ls=":", lw=0.8, alpha=0.7, label="ωx")
    ax.plot(t, d["omega_y"], ls=":", lw=0.8, alpha=0.7, label="ωy")
    ax.plot(t, d["omega_z"], ls=":", lw=0.8, alpha=0.7, label="ωz")
    ax.axhline(max_w, color="red", ls="--", lw=1.0, alpha=0.85, label=f"feas. max {max_w:.1f} rad/s")
    _junction_lines(ax, junctions)
    ax.set_ylabel("[rad/s]")
    ax.legend(fontsize=7, ncol=2)
    ax.set_title("Body angular velocity vs feasibility |ω| limit", fontsize=9)
    ax.grid(True, alpha=0.3)

    ax = sp(10)
    ax.plot(t, am, color="C4", lw=1.1, label="|α| or |ω̇|")
    ax.plot(t, d["angacc_x"], ls=":", lw=0.7, alpha=0.6, label="x")
    ax.plot(t, d["angacc_y"], ls=":", lw=0.7, alpha=0.6, label="y")
    ax.plot(t, d["angacc_z"], ls=":", lw=0.7, alpha=0.6, label="z")
    _junction_lines(ax, junctions)
    ax.set_ylabel("[rad/s²]")
    ax.legend(fontsize=7, ncol=2)
    ax.set_title("Body angular acceleration", fontsize=9)
    ax.grid(True, alpha=0.3)

    ax = sp(11)
    ax.plot(t, d["torque_x"], label="τx", lw=1.0)
    ax.plot(t, d["torque_y"], label="τy", lw=1.0)
    ax.plot(t, d["torque_z"], label="τz", lw=1.0)
    _junction_lines(ax, junctions)
    ax.set_ylabel("N·m")
    ax.set_xlabel("t [s]")
    ax.legend(fontsize=7)
    pk = meta.get("planner_kind", "")
    if pk == "Se3Trajectory":
        tq = "Torque τ = Jα − (Jω)×ω (diagonal CF inertia)"
    elif pk in ("SplineTrajectory", "RichterTrajectory"):
        tq = "Torque from differential flatness (Faessler τ)"
    else:
        tq = "Torque"
    ax.set_title(tq, fontsize=9)
    ax.grid(True, alpha=0.3)

    ax = sp(12)
    ax.plot(px, py, lw=1.0, color="C0", label="path")
    if wps is not None and wps.size:
        ax.scatter(wps["px"], wps["py"], s=30, c="darkred", zorder=5, label="wps")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Top view (x–y)", fontsize=9)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = sp(13)
    ax.plot(px, pz, lw=1.0, color="C5", label="path")
    if wps is not None and wps.size:
        ax.scatter(wps["px"], wps["pz"], s=30, c="darkred", zorder=5, label="wps")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.set_title("Side view (x–z)", fontsize=9)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = sp(14)
    ax.plot(py, pz, lw=1.0, color="C6", label="path")
    if wps is not None and wps.size:
        ax.scatter(wps["py"], wps["pz"], s=30, c="darkred", zorder=5, label="wps")
    ax.set_xlabel("y [m]")
    ax.set_ylabel("z [m]")
    ax.set_title("Side view (y–z)", fontsize=9)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Hardware utilization summary (ratio > 1 means limit exceeded)
    u_t = np.abs(d["thrust_N"]) / max(max_t, 1e-6)
    u_w = om / max(max_w, 1e-6)
    tau_inf = np.maximum(np.abs(d["torque_x"]), np.maximum(np.abs(d["torque_y"]), np.abs(d["torque_z"])))
    u_tau = tau_inf / max(max_tau, 1e-9)
    u_max = np.maximum(u_t, np.maximum(u_w, u_tau))

    ax = sp(15)
    ax.plot(t, u_t, lw=1.0, label="|T| / T_max", color="C3")
    ax.plot(t, u_w, lw=1.0, label="|ω| / ω_max", color="C2")
    ax.plot(t, u_tau, lw=1.0, label="|τ|∞ / τ_axis_max", color="C4")
    ax.plot(t, u_max, lw=1.3, label="max utilization", color="black")
    ax.axhline(1.0, color="red", ls="--", lw=1.0, alpha=0.8, label="limit boundary")
    _junction_lines(ax, junctions)
    ax.set_ylabel("utilization ratio [-]")
    ax.set_xlabel("t [s]")
    ax.set_title("Hardware-limit utilization (crossing 1.0 = violation)", fontsize=9)
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    ax = sp(16)
    v_t = (u_t > 1.0).astype(float)
    v_w = (u_w > 1.0).astype(float)
    v_tau = (u_tau > 1.0).astype(float)
    # stack as 0/1 lanes for quick visual scan
    ax.step(t, v_t + 2.0, where="post", lw=1.0, label="thrust violation", color="C3")
    ax.step(t, v_w + 1.0, where="post", lw=1.0, label="omega violation", color="C2")
    ax.step(t, v_tau + 0.0, where="post", lw=1.0, label="torque violation", color="C4")
    _junction_lines(ax, junctions)
    ax.set_yticks([0.0, 1.0, 2.0, 3.0])
    ax.set_yticklabels(["τ lane", "ω lane", "T lane", "active"])
    ax.set_xlabel("t [s]")
    ax.set_title("Violation timeline (lane high means active limit crossing)", fontsize=9)
    ax.legend(fontsize=7, ncol=2, loc="upper right")
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(gs[nrow - 1, :])
    ax.axis("off")
    pct_t = 100.0 * float(np.mean(u_t > 1.0))
    pct_w = 100.0 * float(np.mean(u_w > 1.0))
    pct_tau = 100.0 * float(np.mean(u_tau > 1.0))
    peak_u = float(np.max(u_max))
    lines = [
        "planning_meta.txt",
        f"  max thrust (feasibility) = {max_t} N",
        f"  max |ω| (feasibility) = {max_w} rad/s",
        f"  max |τ_axis| (feasibility) = {max_tau} N·m",
        f"  planner_kind = {meta.get('planner_kind', '?')}",
        f"  SE(3) jerk/snap placeholder flag: {bool(se3_zero)}",
        f"  peak utilization max(|T/Tmax|, |ω/ωmax|, |τ|∞/τmax) = {peak_u:.3f}",
        f"  violation % of samples: thrust={pct_t:.2f}%  omega={pct_w:.2f}%  torque={pct_tau:.2f}%",
        "",
        "Sidecars:",
        "  waypoints.csv — QP / SLERP inputs",
        "  segment_junctions.csv — interior polynomial joins",
        "  feasibility_se3.txt — sampled |T|,|ω| vs limits (Mode 2)",
        "",
        meta.get("euler_note", "")[:80] + "…" if len(meta.get("euler_note", "")) > 80 else meta.get("euler_note", ""),
    ]
    if feas_txt:
        lines.extend(["", "feasibility.txt (Richter):", feas_txt[:200]])
    ax.text(0.02, 0.98, "\n".join(lines), va="top", ha="left", fontsize=7, family="monospace", wrap=True)

    plt.tight_layout(rect=(0, 0, 1, 0.99))
    out = os.path.join(folder, "overview.png")
    plt.savefig(out, dpi=140)
    plt.close()
    print(f"Saved {out}")

    _plot_trajectory_3d_orientation(folder, d, wps, mode_traj)
    _plot_frenet_sanity(folder, d, t, junctions, mode_traj, max_t, max_w, max_tau)
    _plot_kinematics_axes(folder, d, mode_traj, max_t, max_w, max_tau)
    _plot_state_axes(folder, d, mode_traj, max_t, max_w, max_tau)
    _plot_limits_dashboard(folder, d, t, junctions, mode_traj, meta)


def plot_cross_mode_comparisons() -> None:
    """Overlay Mode 0 / 1 / 2 / 3 / 4 planner references for the same trajectory shape."""
    shapes: Tuple[Tuple[str, Tuple[int, ...]], ...] = (
        ("circle",    (0, 1, 2, 3)),
        ("figure8",   (0, 1, 2, 3)),
        ("helix",     (0, 1, 2, 3)),
        ("corner",    (0, 1, 2, 3)),
        ("loop",      (0, 1, 2, 3)),
        ("flip",      (2,)),
        ("corkscrew", (0, 1, 2, 3)),
        ("roll",      (2,)),
        ("immelmann", (2, 3)),
        ("splits",    (2, 3)),
        ("screw",     (2, 3)),
    )
    colors = {0: "#1f77b4", 1: "#ff7f0e", 2: "#2ca02c", 3: "#9467bd"}
    labels = {
        0: "Mode 0 spline",
        1: "Mode 1 Richter",
        2: "Mode 2 SE(3)",
        3: "Mode 3 paper",
    }

    os.makedirs(CROSS_ROOT, exist_ok=True)

    for shape, modes in shapes:
        loaded: List[Tuple[int, np.ndarray]] = []
        for m in modes:
            csv_p = os.path.join(REF_ROOT, f"mode{m}", shape, "reference.csv")
            if os.path.isfile(csv_p):
                loaded.append((m, np.genfromtxt(csv_p, delimiter=",", names=True)))

        if len(loaded) < 2:
            continue

        fig = plt.figure(figsize=(13, 10))
        fig.suptitle(f"Planner reference — cross-mode comparison: {shape}", fontsize=12)

        ax3d = fig.add_subplot(2, 2, 1, projection="3d")
        for m, dat in loaded:
            ax3d.plot(dat["px"], dat["py"], dat["pz"], color=colors[m], lw=1.3, label=labels[m])
            _draw_orientation_glyphs(ax3d, dat, color=colors[m], axis_len=0.06, count=10, alpha=0.28)
        ax3d.set_xlabel("x [m]")
        ax3d.set_ylabel("y [m]")
        ax3d.set_zlabel("z [m]")
        ax3d.set_title("3-D path")
        ax3d.legend(fontsize=8)

        ax_t = fig.add_subplot(2, 2, 2)
        meta0 = _parse_meta(os.path.join(REF_ROOT, f"mode{loaded[0][0]}", shape))
        max_t = float(meta0.get("feasibility_max_thrust_N", "0.55"))
        for m, dat in loaded:
            ax_t.plot(dat["t"], dat["thrust_N"], color=colors[m], lw=1.1, label=labels[m])
        ax_t.axhline(max_t, color="red", ls="--", lw=0.9, alpha=0.7)
        ax_t.axhline(-max_t, color="red", ls="--", lw=0.6, alpha=0.4)
        ax_t.set_xlabel("t [s]")
        ax_t.set_ylabel("thrust [N]")
        ax_t.set_title("Thrust (feasibility limits: ± {:.2f} N)".format(max_t))
        ax_t.legend(fontsize=8)
        ax_t.grid(True, alpha=0.3)

        ax_w = fig.add_subplot(2, 2, 3)
        max_w = float(meta0.get("feasibility_max_omega_rad_s", "12.0"))
        for m, dat in loaded:
            omm = np.sqrt(dat["omega_x"] ** 2 + dat["omega_y"] ** 2 + dat["omega_z"] ** 2)
            ax_w.plot(dat["t"], omm, color=colors[m], lw=1.1, label=labels[m])
        ax_w.axhline(max_w, color="red", ls="--", lw=0.9, alpha=0.7)
        ax_w.set_xlabel("t [s]")
        ax_w.set_ylabel("|ω| [rad/s]")
        ax_w.set_title("|ω| vs feasibility max ({:.1f} rad/s)".format(max_w))
        ax_w.legend(fontsize=8)
        ax_w.grid(True, alpha=0.3)

        ax_v = fig.add_subplot(2, 2, 4)
        for m, dat in loaded:
            vm = np.sqrt(dat["vx"] ** 2 + dat["vy"] ** 2 + dat["vz"] ** 2)
            ax_v.plot(dat["t"], vm, color=colors[m], lw=1.1, label=labels[m])
        ax_v.set_xlabel("t [s]")
        ax_v.set_ylabel("|v| [m/s]")
        ax_v.set_title("Speed magnitude")
        ax_v.legend(fontsize=8)
        ax_v.grid(True, alpha=0.3)

        plt.tight_layout(rect=(0, 0, 1, 0.96))
        out_dir = os.path.join(CROSS_ROOT, shape)
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, "comparison.png")
        plt.savefig(out_path, dpi=140)
        plt.close()
        print(f"Saved cross-mode {out_path}")


def main() -> None:
    pattern = os.path.join(REF_ROOT, "mode*", "*", "reference.csv")
    paths = sorted(glob.glob(pattern))
    if not paths:
        print(
            "No reference.csv files found under",
            REF_ROOT,
            "\nRun: cargo run --release --bin planning_sim -- --export-reference-only",
            file=sys.stderr,
        )
        sys.exit(1)
    for p in paths:
        _plot_reference_csv(p)
    plot_cross_mode_comparisons()
    print(f"\nDone. Per-folder overviews: {len(paths)}; cross_mode in {CROSS_ROOT}")


if __name__ == "__main__":
    main()
