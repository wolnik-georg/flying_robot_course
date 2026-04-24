#!/usr/bin/env python3
"""
Trajectory Validation Plots
============================
Evaluates planned Poly4D trajectories in physical time and produces two
figures per trajectory saved to trajectory_plots/<slug>/:

  <slug>_path.png     — 2D XY path + 3D perspective (large, uncluttered)
  <slug>_dynamics.png — 7 panels:
      Row 0: Speed/tilt/yaw overview · Position x/y/z · Velocity vx/vy/vz
      Row 1: Acceleration ax/ay/az  · Jerk jx/jy/jz  · Thrust feasibility
      Row 2: Angular velocity wx/wy/wz [deg/s]  (wide, full-width)
             Derived from differential flatness — matches expected gyro readings.

Usage:
    ~/.pyenv/versions/flying_robots/bin/python plot_trajectories.py
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3D projection)

GRAVITY    = 9.81   # m/s²
HOVER_Z    = 1.0    # m — nominal hover height for 3D z-offset
OUT_DIR    = "trajectory_plots"

# Crazyflie 2.1 physical limits
CF_MASS_KG      = 0.027   # kg
CF_MAX_THRUST_N = 0.60    # N  (~4 × 0.15 N)
CF_HOVER_N      = CF_MASS_KG * GRAVITY   # ≈ 0.265 N

# ---------------------------------------------------------------------------
# Trajectory data
# ---------------------------------------------------------------------------

# ── Circle (run_circle.py / run_fast_circle.py) ──────────────────────────────
# r=0.30m, 16 segments × 0.654s, total=10.47s, periodic=True
circle_poly4d = [
    [0.654498, -0.000000,  0.000000,  0.054000,  0.000000, -0.001621,  0.000003,  0.000016,  0.000001,  0.000000,  0.180000, -0.000000, -0.010800,  0.000002,  0.000190,  0.000006, -0.000004, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  1.570796, -0.600000,  0.000000,  0.000001, -0.000022,  0.000079, -0.000101,  0.000044],
    [0.654498,  0.022836,  0.068883,  0.049889, -0.004133, -0.001496,  0.000073,  0.000019, -0.000001,  0.114805,  0.166298, -0.020665, -0.009978,  0.000621,  0.000175, -0.000002, -0.000004, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  1.178097, -0.600000, -0.000000, -0.000000,  0.000000, -0.000001,  0.000001, -0.000001],
    [0.654498,  0.087868,  0.127279,  0.038184, -0.007637, -0.001146,  0.000137,  0.000015, -0.000002,  0.212132,  0.127279, -0.038184, -0.007637,  0.001144,  0.000142, -0.000019,  0.000001, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.785398, -0.600000,  0.000000,  0.000000, -0.000003,  0.000011, -0.000014,  0.000006],
    [0.654498,  0.185195,  0.166298,  0.020665, -0.009978, -0.000626,  0.000201, -0.000020,  0.000010,  0.277164,  0.068883, -0.049889, -0.004133,  0.001493,  0.000088, -0.000036,  0.000007, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.392699, -0.600000, -0.000000, -0.000000,  0.000002, -0.000009,  0.000012, -0.000005],
    [0.654498,  0.300000,  0.180000,  0.000000, -0.010800,  0.000002,  0.000186,  0.000011, -0.000006,  0.300000, -0.000000, -0.054000,  0.000000,  0.001626, -0.000022,  0.000008, -0.000012, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.600000, -0.000000,  0.000000, -0.000008,  0.000032, -0.000041,  0.000018],
    [0.654498,  0.414805,  0.166298, -0.020665, -0.009978,  0.000631,  0.000139,  0.000043, -0.000024,  0.277164, -0.068883, -0.049889,  0.004133,  0.001498, -0.000079, -0.000012, -0.000002, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.392699, -0.600000,  0.000000, -0.000000, -0.000001,  0.000001, -0.000001,  0.000000],
    [0.654498,  0.512132,  0.127279, -0.038184, -0.007637,  0.001139,  0.000161, -0.000043,  0.000012,  0.212132, -0.127279, -0.038184,  0.007637,  0.001146, -0.000140, -0.000010, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.785398, -0.600000, -0.000001,  0.000000,  0.000006, -0.000019,  0.000023, -0.000010],
    [0.654498,  0.577164,  0.068883, -0.049890, -0.004133,  0.001503,  0.000051,  0.000011, -0.000013,  0.114805, -0.166298, -0.020665,  0.009978,  0.000620, -0.000182, -0.000004,  0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -1.178097, -0.600000,  0.000001,  0.000000, -0.000005,  0.000015, -0.000018,  0.000008],
    [0.654498,  0.600000, -0.000000, -0.054000,  0.000000,  0.001620, -0.000001, -0.000018, -0.000001, -0.000000, -0.180000,  0.000000,  0.010800, -0.000003, -0.000184, -0.000013,  0.000007, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -1.570796, -0.600000, -0.000000, -0.000000,  0.000013, -0.000045,  0.000057, -0.000025],
    [0.654498,  0.577164, -0.068883, -0.049890,  0.004133,  0.001492, -0.000058, -0.000039,  0.000010, -0.114805, -0.166298,  0.020665,  0.009978, -0.000618, -0.000188,  0.000019, -0.000003, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -1.963495, -0.600000, -0.000000, -0.000000,  0.000002, -0.000008,  0.000010, -0.000004],
    [0.654498,  0.512132, -0.127279, -0.038184,  0.007637,  0.001142, -0.000126, -0.000028,  0.000008, -0.212132, -0.127279,  0.038184,  0.007637, -0.001144, -0.000142,  0.000019, -0.000001, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -2.356194, -0.600000, -0.000000,  0.000000,  0.000035, -0.000127,  0.000162, -0.000071],
    [0.654498,  0.414805, -0.166298, -0.020665,  0.009978,  0.000620, -0.000180, -0.000007,  0.000001, -0.277164, -0.068883,  0.049890,  0.004133, -0.001501, -0.000058, -0.000002,  0.000009, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -2.748894, -0.600000,  0.000001, -0.000000, -0.000012,  0.000040, -0.000050,  0.000022],
    [0.654498,  0.300000, -0.180000, -0.000000,  0.010800,  0.000001, -0.000196,  0.000002,  0.000001, -0.300000,  0.000000,  0.054000,  0.000000, -0.001619, -0.000003,  0.000024, -0.000002, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -3.141593, -0.600000, -0.000001, -0.000001,  0.000015, -0.000053,  0.000068, -0.000030],
    [0.654498,  0.185195, -0.166298,  0.020665,  0.009978, -0.000621, -0.000175,  0.000002,  0.000004, -0.277164,  0.068883,  0.049890, -0.004133, -0.001504,  0.000099, -0.000013,  0.000013, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -3.534292, -0.600001,  0.000000,  0.000001,  0.000012, -0.000043,  0.000053, -0.000023],
    [0.654498,  0.087868, -0.127279,  0.038184,  0.007637, -0.001145, -0.000139,  0.000015,  0.000000, -0.212132,  0.127279,  0.038184, -0.007637, -0.001146,  0.000139,  0.000011, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -3.926991, -0.600000,  0.000001, -0.000001,  0.000014, -0.000053,  0.000068, -0.000030],
    [0.654498,  0.022836, -0.068883,  0.049889,  0.004133, -0.001497, -0.000075,  0.000018,  0.000000, -0.114805,  0.166298,  0.020665, -0.009978, -0.000621,  0.000183,  0.000004,  0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -4.319690, -0.600000, -0.000001, -0.000000, -0.000031,  0.000117, -0.000151,  0.000066],
]

# ── Our min-snap figure-8 (run_figure8.py) ────────────────────────────────────
# x ∈ [-0.92, +0.92]m, y ∈ [-0.45, +0.45]m, total=7.28s, rest-to-rest
our_figure8_poly4d = [
    [1.050000, -0.000000, -0.000000,  0.000000,  0.000000,  0.920060, -0.422403, -0.330819,  0.184887,  0.000000,  0.000000, -0.000000,  0.000000, -1.744998,  1.466438,  0.107645, -0.241908, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [0.710000,  0.396058,  0.894217,  0.119978, -0.586456, -0.088417,  0.767407, -0.773189,  0.263120, -0.445604, -0.612846,  0.911715,  1.039294, -0.529370, -1.547065,  1.745710, -0.571617, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [0.620000,  0.922409,  0.425094, -0.600250, -0.163530, -0.012154,  0.048479,  0.110009, -0.056150, -0.291165,  0.907200,  0.475613, -0.850656,  0.203289,  0.237703, -0.290271,  0.089495, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [0.700000,  0.923174, -0.445424, -0.681107,  0.226650,  0.293565, -0.024591, -0.190025,  0.072759,  0.289869,  0.761427, -0.542422, -0.353518,  0.032877, -0.064582,  0.204906, -0.075639, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [0.560000,  0.405364, -0.824251,  0.146024,  0.236002, -0.316721, -0.076047,  0.161773, -0.039131,  0.450742, -0.405717, -0.938657,  0.192123,  0.406789,  0.021473, -0.156746,  0.042205, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [0.560000,  0.000000, -0.653540,  0.006420, -0.278385, -0.009358,  0.209809,  0.006890, -0.045898, -0.000000, -1.022634,  0.004823,  0.765421, -0.010510, -0.227116,  0.011757,  0.032847, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [0.700000, -0.402804, -0.819292, -0.153336,  0.224830,  0.327403, -0.067263, -0.177646,  0.076262, -0.449354, -0.405209,  0.927436,  0.184003, -0.387313,  0.024881,  0.149373, -0.070147, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [0.620000, -0.921641, -0.451349,  0.680038,  0.234193, -0.308640,  0.000361,  0.139849, -0.058210, -0.292459,  0.755025,  0.550873, -0.343333, -0.024645, -0.124468, -0.087936,  0.085862, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [0.710000, -0.923935,  0.421735,  0.602634, -0.164286, -0.084256,  0.261179, -0.536957,  0.263618,  0.288570,  0.912522, -0.470936, -0.858025, -0.016914, -0.156197,  1.086628, -0.567817, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [1.053185, -0.398611,  0.895363, -0.115272, -0.585975, -0.724664,  1.764668, -1.017567,  0.182051,  0.447039, -0.609322, -0.918168,  1.034657,  2.062702, -3.428890,  1.649625, -0.237630, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
]


# ---------------------------------------------------------------------------
# Polynomial evaluation helpers
# ---------------------------------------------------------------------------

def poly_eval(coeffs, t, n=0):
    """Evaluate nth derivative of polynomial at physical time t."""
    result = 0.0
    for i, c in enumerate(coeffs):
        if i < n:
            continue
        factor = 1
        for k in range(n):
            factor *= i - k
        tp = t ** (i - n) if i > n else 1.0
        result += c * factor * tp
    return result


def _eval_point(cx, cy, cz, cyaw, t_s, speed_scale):
    ss = speed_scale
    x  = poly_eval(cx, t_s)
    y  = poly_eval(cy, t_s)
    z  = poly_eval(cz, t_s)
    vx = poly_eval(cx, t_s, 1) * ss
    vy = poly_eval(cy, t_s, 1) * ss
    vz = poly_eval(cz, t_s, 1) * ss
    ax = poly_eval(cx, t_s, 2) * ss**2
    ay = poly_eval(cy, t_s, 2) * ss**2
    az = poly_eval(cz, t_s, 2) * ss**2
    jx = poly_eval(cx, t_s, 3) * ss**3
    jy = poly_eval(cy, t_s, 3) * ss**3
    jz = poly_eval(cz, t_s, 3) * ss**3
    f      = np.array([ax, ay, az + GRAVITY])
    tilt   = np.degrees(np.arccos(np.clip(GRAVITY / np.linalg.norm(f), -1, 1)))
    thrust = CF_MASS_KG * np.linalg.norm(f)
    yaw_d  = np.degrees(poly_eval(cyaw, t_s))
    yr_d   = np.degrees(poly_eval(cyaw, t_s, 1) * ss)
    return x, y, z, vx, vy, vz, ax, ay, az, jx, jy, jz, tilt, thrust, yaw_d, yr_d


def eval_traj(traj, speed_scale=1.0, pts_per_seg=80, hover_z=1.0):
    """Evaluate a Poly4D trajectory. Returns dict of arrays."""
    ts, xs, ys, zs   = [], [], [], []
    vxs, vys, vzs    = [], [], []
    axs, ays, azs    = [], [], []
    jxs, jys, jzs    = [], [], []
    tilts, thrusts   = [], []
    yaws, yaw_rates  = [], []
    seg_starts       = []
    t_global = 0.0

    for row in traj:
        dur_stored = row[0]
        dur_actual = dur_stored / speed_scale
        cx, cy, cz, cyaw = row[1:9], row[9:17], row[17:25], row[25:33]
        seg_starts.append(t_global)

        for t_local in np.linspace(0, dur_actual, pts_per_seg, endpoint=False):
            t_s = t_local * speed_scale
            x, y, z, vx, vy, vz, ax, ay, az, jx, jy, jz, tilt, thr, yaw_d, yr_d = \
                _eval_point(cx, cy, cz, cyaw, t_s, speed_scale)
            xs.append(x);   ys.append(y);   zs.append(hover_z + z)
            vxs.append(vx); vys.append(vy); vzs.append(vz)
            axs.append(ax); ays.append(ay); azs.append(az)
            jxs.append(jx); jys.append(jy); jzs.append(jz)
            tilts.append(tilt); thrusts.append(thr)
            yaws.append(yaw_d); yaw_rates.append(yr_d)
            ts.append(t_global + t_local)
        t_global += dur_actual

    # Final point
    row = traj[-1]; dur_s = row[0]
    cx, cy, cz, cyaw = row[1:9], row[9:17], row[17:25], row[25:33]
    x, y, z, vx, vy, vz, ax, ay, az, jx, jy, jz, tilt, thr, yaw_d, yr_d = \
        _eval_point(cx, cy, cz, cyaw, dur_s, speed_scale)
    xs.append(x);   ys.append(y);   zs.append(hover_z + z)
    vxs.append(vx); vys.append(vy); vzs.append(vz)
    axs.append(ax); ays.append(ay); azs.append(az)
    jxs.append(jx); jys.append(jy); jzs.append(jz)
    tilts.append(tilt); thrusts.append(thr)
    yaws.append(yaw_d); yaw_rates.append(yr_d)
    ts.append(t_global)
    seg_starts.append(t_global)

    return dict(
        t=np.array(ts), x=np.array(xs), y=np.array(ys), z=np.array(zs),
        vx=np.array(vxs), vy=np.array(vys), vz=np.array(vzs),
        ax=np.array(axs), ay=np.array(ays), az=np.array(azs),
        jx=np.array(jxs), jy=np.array(jys), jz=np.array(jzs),
        speed=np.sqrt(np.array(vxs)**2 + np.array(vys)**2 + np.array(vzs)**2),
        acc_h=np.sqrt(np.array(axs)**2 + np.array(ays)**2),
        jerk_h=np.sqrt(np.array(jxs)**2 + np.array(jys)**2),
        tilt=np.array(tilts),
        thrust_N=np.array(thrusts),
        yaw=np.array(yaws),
        yaw_rate=np.array(yaw_rates),
        seg_starts=np.array(seg_starts),
    )


def compute_angular_velocity(d):
    """Derive body-frame angular velocity [deg/s] from the flatness rotation matrices.

    Method: build R(t) = [x_B | y_B | z_B] at every sample using acc + yaw,
    then central-difference R to get Ṙ, and extract ω from R^T Ṙ (skew-symmetric).

    Convention (right-hand, body frame):
        wx = roll rate  = (R^T Ṙ)[2,1]
        wy = pitch rate = (R^T Ṙ)[0,2]
        wz = yaw rate   = (R^T Ṙ)[1,0]
    """
    n = len(d['t'])
    Rs = np.empty((n, 3, 3))

    for i in range(n):
        f_vec = np.array([d['ax'][i], d['ay'][i], d['az'][i] + GRAVITY])
        f_mag = np.linalg.norm(f_vec)
        z_B   = f_vec / max(f_mag, 1e-9)

        yaw_rad = np.radians(d['yaw'][i])
        x_C = np.array([np.cos(yaw_rad), np.sin(yaw_rad), 0.0])

        y_B_raw = np.cross(z_B, x_C)
        y_mag   = np.linalg.norm(y_B_raw)
        if y_mag < 1e-9:                   # gimbal lock: z_B ∥ x_C
            x_C     = np.array([0.0, 0.0, 1.0])
            y_B_raw = np.cross(z_B, x_C)
            y_mag   = np.linalg.norm(y_B_raw)
        y_B = y_B_raw / y_mag
        x_B = np.cross(y_B, z_B)

        Rs[i] = np.column_stack([x_B, y_B, z_B])   # columns = body axes in world frame

    # Central differences for Ṙ; forward/backward at endpoints
    wx = np.zeros(n); wy = np.zeros(n); wz = np.zeros(n)
    dt = np.diff(d['t'])

    for i in range(n):
        if i == 0:
            R_dot = (Rs[1] - Rs[0]) / dt[0]
        elif i == n - 1:
            R_dot = (Rs[-1] - Rs[-2]) / dt[-1]
        else:
            R_dot = (Rs[i+1] - Rs[i-1]) / (dt[i-1] + dt[i])

        Omega = Rs[i].T @ R_dot          # skew-symmetric body angular velocity matrix
        wx[i] = Omega[2, 1]              # roll rate
        wy[i] = Omega[0, 2]              # pitch rate
        wz[i] = Omega[1, 0]             # yaw rate

    return np.degrees(wx), np.degrees(wy), np.degrees(wz)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _seg_lines(ax, seg_t):
    for sv in seg_t:
        ax.axvline(sv, color='gray', lw=0.5, alpha=0.4)


def _save(fig, folder, filename):
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, filename)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Figure 1 — Path  (2D + 3D, large and uncluttered)
# ---------------------------------------------------------------------------

def plot_path(d, traj, name, speed_scale, slug):
    total_t = d['t'][-1]
    max_spd = d['speed'].max()
    seg_t   = d['seg_starts']
    gap     = np.hypot(d['x'][-1] - d['x'][0], d['y'][-1] - d['y'][0])

    fig, axes = plt.subplots(1, 2, figsize=(16, 7),
                             gridspec_kw={'width_ratios': [1, 1.1]})
    fig.suptitle(
        f"{name}  —  SPEED_SCALE={speed_scale}  |  {len(traj)} segments  |  "
        f"{total_t:.2f} s  |  gap={gap*1000:.2f} mm",
        fontsize=13, fontweight='bold',
    )

    # ── 2D XY ────────────────────────────────────────────────────────────────
    ax = axes[0]
    sc = ax.scatter(d['x'], d['y'], c=d['speed'], cmap='plasma',
                    s=5, zorder=3, vmin=0, vmax=max_spd)
    fig.colorbar(sc, ax=ax, label='speed [m/s]', fraction=0.046, pad=0.04)
    for sv in seg_t[:-1]:
        idx = np.argmin(np.abs(d['t'] - sv))
        ax.plot(d['x'][idx], d['y'][idx], 'w+', ms=6, mew=1.2, zorder=4)
    ax.plot(d['x'][0],  d['y'][0],  'go', ms=10, zorder=5, label='start')
    ax.plot(d['x'][-1], d['y'][-1], 'rs', ms=9,  zorder=5, label='end')
    ax.set_xlabel('x [m]', fontsize=11); ax.set_ylabel('y [m]', fontsize=11)
    ax.set_title(f'XY Path  (max speed = {max_spd:.2f} m/s)', fontsize=11)
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3); ax.legend(fontsize=9)

    # ── 3D ───────────────────────────────────────────────────────────────────
    ax3 = fig.add_subplot(1, 2, 2, projection='3d')
    axes[1].remove()
    for i in range(len(d['x']) - 1):
        colour = plt.cm.plasma(d['speed'][i] / max(max_spd, 1e-6))
        ax3.plot(d['x'][i:i+2], d['y'][i:i+2], d['z'][i:i+2], color=colour, lw=2.0)
    ax3.scatter([d['x'][0]],  [d['y'][0]],  [d['z'][0]],  color='green', s=80, zorder=5)
    ax3.scatter([d['x'][-1]], [d['y'][-1]], [d['z'][-1]], color='red', marker='s', s=70, zorder=5)
    ax3.set_xlabel('x [m]'); ax3.set_ylabel('y [m]'); ax3.set_zlabel('z [m]')
    ax3.set_title('3D Path  (colour = speed, blue→yellow)', fontsize=11)
    ax3.view_init(elev=25, azim=-60)

    folder = os.path.join(OUT_DIR, slug)
    _save(fig, folder, f"{slug}_path.png")
    return fig


# ---------------------------------------------------------------------------
# Figure 2 — Dynamics  (2×3 panels + 1 wide angular-velocity panel)
# ---------------------------------------------------------------------------

def plot_dynamics(d, traj, name, speed_scale, slug):
    total_t  = d['t'][-1]
    max_tilt = d['tilt'].max()
    max_acc  = d['acc_h'].max()
    seg_t    = d['seg_starts']
    feasible = bool(d['thrust_N'].max() <= CF_MAX_THRUST_N)

    wx_dps, wy_dps, wz_dps = compute_angular_velocity(d)
    max_w = max(np.abs(wx_dps).max(), np.abs(wy_dps).max(), np.abs(wz_dps).max())

    fig = plt.figure(figsize=(18, 14))
    fig.suptitle(
        f"{name}  —  Dynamics  |  SPEED_SCALE={speed_scale}  |  {total_t:.2f} s  |  "
        f"thrust {'OK' if feasible else 'OVER LIMIT'}",
        fontsize=13, fontweight='bold',
        color='black' if feasible else 'red',
    )
    gs = fig.add_gridspec(3, 3, hspace=0.48, wspace=0.42,
                          height_ratios=[1, 1, 0.85])

    # ── Row 0, Panel 0: Overview (speed / tilt / yaw / yaw_rate / horiz-accel) ──
    ax1 = fig.add_subplot(gs[0, 0])
    ax1b = ax1.twinx()
    ax1c = ax1.twinx()
    ax1c.spines['right'].set_position(('axes', 1.16))
    l1, = ax1.plot(d['t'], d['speed'],    color='tab:blue',   lw=1.4, label='speed [m/s]')
    l2, = ax1.plot(d['t'], d['tilt'],     color='tab:orange', lw=1.2, ls='--', label='tilt [°]')
    l3, = ax1b.plot(d['t'], d['acc_h'],   color='tab:red',    lw=1.0, ls=':',  label='|a_xy| [m/s²]')
    l4, = ax1c.plot(d['t'], d['yaw'],     color='tab:purple', lw=1.0,          label='yaw [°]')
    l5, = ax1c.plot(d['t'], d['yaw_rate'],color='tab:pink',   lw=0.9, ls='--', label='yaw_rate [°/s]')
    _seg_lines(ax1, seg_t)
    ax1.set_xlabel('time [s]')
    ax1.set_ylabel('speed [m/s] / tilt [°]')
    ax1b.set_ylabel('|a_xy| [m/s²]', color='tab:red')
    ax1c.set_ylabel('yaw [°] / rate [°/s]', color='tab:purple')
    ax1.set_title(f'Overview  (max tilt={max_tilt:.1f}°  |a_xy|={max_acc:.2f} m/s²)')
    ax1.legend([l1,l2,l3,l4,l5], [l.get_label() for l in [l1,l2,l3,l4,l5]], fontsize=7)
    ax1.grid(True, alpha=0.3)

    # ── Row 0, Panel 1: Position ──────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(d['t'], d['x'], color='tab:blue',   lw=1.3, label='x')
    ax2.plot(d['t'], d['y'], color='tab:orange', lw=1.3, label='y')
    ax2.plot(d['t'], d['z'], color='tab:green',  lw=1.3, label='z')
    _seg_lines(ax2, seg_t)
    ax2.set_xlabel('time [s]'); ax2.set_ylabel('position [m]')
    ax2.set_title('Position vs Time')
    ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)

    # ── Row 0, Panel 2: Velocity ──────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(d['t'], d['vx'],    color='tab:blue',   lw=1.3, label='vx')
    ax3.plot(d['t'], d['vy'],    color='tab:orange', lw=1.3, label='vy')
    ax3.plot(d['t'], d['vz'],    color='tab:green',  lw=1.3, label='vz')
    ax3.plot(d['t'], d['speed'], color='k',          lw=1.0, ls='--', label='|v|')
    _seg_lines(ax3, seg_t)
    ax3.set_xlabel('time [s]'); ax3.set_ylabel('velocity [m/s]')
    ax3.set_title('Velocity vs Time')
    ax3.legend(fontsize=8); ax3.grid(True, alpha=0.3)

    # ── Row 1, Panel 0: Acceleration ─────────────────────────────────────────
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.plot(d['t'], d['ax'],    color='tab:blue',   lw=1.3, label='ax')
    ax4.plot(d['t'], d['ay'],    color='tab:orange', lw=1.3, label='ay')
    ax4.plot(d['t'], d['az'],    color='tab:green',  lw=1.3, label='az')
    ax4.plot(d['t'], d['acc_h'], color='k',          lw=1.0, ls='--', label='|a_xy|')
    _seg_lines(ax4, seg_t)
    ax4.set_xlabel('time [s]'); ax4.set_ylabel('acceleration [m/s²]')
    ax4.set_title('Acceleration vs Time')
    ax4.legend(fontsize=8); ax4.grid(True, alpha=0.3)

    # ── Row 1, Panel 1: Jerk ─────────────────────────────────────────────────
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.plot(d['t'], d['jx'],     color='tab:blue',   lw=1.3, label='jx')
    ax5.plot(d['t'], d['jy'],     color='tab:orange', lw=1.3, label='jy')
    ax5.plot(d['t'], d['jz'],     color='tab:green',  lw=1.3, label='jz')
    ax5.plot(d['t'], d['jerk_h'], color='k',          lw=1.0, ls='--', label='|j_xy|')
    _seg_lines(ax5, seg_t)
    ax5.set_xlabel('time [s]'); ax5.set_ylabel('jerk [m/s³]')
    ax5.set_title('Jerk vs Time  (spikes at segment joins = smoothness quality)')
    ax5.legend(fontsize=8); ax5.grid(True, alpha=0.3)

    # ── Row 1, Panel 2: Thrust feasibility ───────────────────────────────────
    ax6 = fig.add_subplot(gs[1, 2])
    ax6.plot(d['t'], d['thrust_N'], color='tab:blue', lw=1.4, label='required thrust')
    ax6.axhline(CF_MAX_THRUST_N, color='red',  lw=1.2, ls='--',
                label=f'max ({CF_MAX_THRUST_N:.2f} N)')
    ax6.axhline(CF_HOVER_N,      color='gray', lw=1.0, ls=':',
                label=f'hover ({CF_HOVER_N:.3f} N)')
    ax6.fill_between(d['t'], d['thrust_N'], CF_MAX_THRUST_N,
                     where=d['thrust_N'] > CF_MAX_THRUST_N,
                     color='red', alpha=0.25, label='over limit')
    margin_pct = 100 * (1 - d['thrust_N'].max() / CF_MAX_THRUST_N)
    _seg_lines(ax6, seg_t)
    ax6.set_xlabel('time [s]'); ax6.set_ylabel('thrust [N]')
    status = f"margin={margin_pct:.1f}%" if feasible else "OVER LIMIT"
    ax6.set_title(f'Thrust Feasibility  ({status})',
                  color='black' if feasible else 'red')
    ax6.set_ylim(bottom=0)
    ax6.legend(fontsize=8); ax6.grid(True, alpha=0.3)

    # ── Row 2, wide: Angular velocity (body frame) ────────────────────────────
    ax7 = fig.add_subplot(gs[2, :])   # spans all 3 columns
    ax7.plot(d['t'], wx_dps, color='tab:blue',   lw=1.3, label='ωx  roll rate')
    ax7.plot(d['t'], wy_dps, color='tab:orange', lw=1.3, label='ωy  pitch rate')
    ax7.plot(d['t'], wz_dps, color='tab:green',  lw=1.3, label='ωz  yaw rate')
    ax7.axhline(0, color='k', lw=0.6, alpha=0.4)
    _seg_lines(ax7, seg_t)
    ax7.set_xlabel('time [s]', fontsize=10)
    ax7.set_ylabel('angular velocity [°/s]', fontsize=10)
    ax7.set_title(
        f'Body-Frame Angular Velocity  (from differential flatness)  '
        f'max |ω| = {max_w:.1f} °/s',
        fontsize=10,
    )
    ax7.legend(fontsize=9, ncol=3, loc='upper right')
    ax7.grid(True, alpha=0.3)

    folder = os.path.join(OUT_DIR, slug)
    _save(fig, folder, f"{slug}_dynamics.png")
    return fig


# ---------------------------------------------------------------------------
# Main — define trajectories and run
# ---------------------------------------------------------------------------

# Add new trajectories here: (data, display_name, speed_scale, folder_slug)
TRAJECTORIES = [
    (circle_poly4d,      "Circle (r=0.30m)",  1.0, "circle"),
    (circle_poly4d,      "Fast Circle (2×)",  2.0, "fast_circle"),
    (our_figure8_poly4d, "Figure-8",          1.0, "figure8"),
]

print("\n── Trajectory Summary ─────────────────────────────────────────────────────────────")
print(f"  {'Name':<22}  {'Duration':>9}  {'Max spd':>8}  {'Max tilt':>9}  "
      f"{'Max thrust':>11}  {'Max |ω|':>9}  {'Gap':>8}")
print(f"  {'-'*22}  {'-'*9}  {'-'*8}  {'-'*9}  {'-'*11}  {'-'*9}  {'-'*8}")

figs = []
for traj, name, ss, slug in TRAJECTORIES:
    d   = eval_traj(traj, speed_scale=ss, hover_z=HOVER_Z)
    gap = np.hypot(d['x'][-1] - d['x'][0], d['y'][-1] - d['y'][0])
    ok  = "OK" if d['thrust_N'].max() <= CF_MAX_THRUST_N else "OVER"
    wx_dps, wy_dps, wz_dps = compute_angular_velocity(d)
    max_w = max(np.abs(wx_dps).max(), np.abs(wy_dps).max(), np.abs(wz_dps).max())
    print(f"  {name:<22}  {d['t'][-1]:>8.2f}s  {d['speed'].max():>7.2f}m/s"
          f"  {d['tilt'].max():>8.1f}°  {d['thrust_N'].max():>8.3f}N {ok:<4}"
          f"  {max_w:>8.1f}°/s  {gap*1000:>6.2f}mm")
    figs.append(plot_path(d, traj, name, ss, slug))
    figs.append(plot_dynamics(d, traj, name, ss, slug))

print()
plt.show()
