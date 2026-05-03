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
  reference/cross_mode/<traj>/comparison.png   (Modes 0/1/2 overlaid where available)

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

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REF_ROOT = os.path.join(_ROOT, "results", "planning_sim", "reference")
CROSS_ROOT = os.path.join(_ROOT, "results", "planning_sim", "cross_mode")


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


def _plot_frenet_sanity(folder: str, d: np.ndarray, t: np.ndarray, junctions: List[float], title: str) -> None:
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

    ax = axes[1, 2]
    ax.axis("off")
    ax.text(
        0.02,
        0.98,
        "Finite differences use numpy.gradient(..., t).\n"
        "Large spikes often occur at segment joins or very small |v|.\n"
        "Frenet scalars use analytic v,a from the CSV.",
        va="top",
        ha="left",
        fontsize=8,
        family="monospace",
    )

    plt.tight_layout()
    out = os.path.join(folder, "frenet_sanity.png")
    plt.savefig(out, dpi=140)
    plt.close()
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
    se3_zero = meta.get("se3_export_jerk_snap_zero", "0") == "1"

    px, py, pz = d["px"], d["py"], d["pz"]
    vx = np.sqrt(d["vx"] ** 2 + d["vy"] ** 2 + d["vz"] ** 2)
    axm = np.sqrt(d["ax"] ** 2 + d["ay"] ** 2 + d["az"] ** 2)
    jm = np.sqrt(d["jx"] ** 2 + d["jy"] ** 2 + d["jz"] ** 2)
    sm = np.sqrt(d["sx"] ** 2 + d["sy"] ** 2 + d["sz"] ** 2)
    om = np.sqrt(d["omega_x"] ** 2 + d["omega_y"] ** 2 + d["omega_z"] ** 2)
    am = np.sqrt(d["angacc_x"] ** 2 + d["angacc_y"] ** 2 + d["angacc_z"] ** 2)

    has_q = d.dtype.names and "qw" in d.dtype.names

    fig = plt.figure(figsize=(14, 25))
    fig.suptitle(f"Planner-only reference: {mode_traj}{title_extra}", fontsize=11, y=0.995)

    nrow, ncol = 8, 2

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

    ax = fig.add_subplot(gs[nrow - 1, :])
    ax.axis("off")
    lines = [
        "planning_meta.txt",
        f"  max thrust (feasibility) = {max_t} N",
        f"  max |ω| (feasibility) = {max_w} rad/s",
        f"  planner_kind = {meta.get('planner_kind', '?')}",
        f"  SE(3) jerk/snap placeholder flag: {bool(se3_zero)}",
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

    _plot_frenet_sanity(folder, d, t, junctions, mode_traj)


def plot_cross_mode_comparisons() -> None:
    """Overlay Mode 0 / 1 / 2 planner references for the same trajectory shape."""
    shapes: Tuple[Tuple[str, Tuple[int, ...]], ...] = (
        ("circle", (0, 1, 2)),
        ("figure8", (0, 1, 2)),
        ("helix", (0, 1, 2)),
        ("loop", (0, 1, 2)),
        ("flip", (2,)),
    )
    colors = {0: "#1f77b4", 1: "#ff7f0e", 2: "#2ca02c"}
    labels = {0: "Mode 0 spline", 1: "Mode 1 Richter", 2: "Mode 2 SE(3)"}

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
