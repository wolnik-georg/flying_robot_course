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

# ── Fast Figure-8 (run_fast_figure8.py) ────────────────────────────────────────
# Same waypoints as our_figure8_poly4d but every segment duration halved.
# QP re-optimised for 2× speed (3.641 s total). Generated by:
#   cargo run --release --bin export_poly4d fast_figure8
fast_figure8_poly4d = [
    [0.525000,  0.000000,  0.000000, -0.000000,  0.000000, 14.721231, -13.518221, -21.170134, 23.664181,  0.000000,  0.000000,  0.000000,  0.000000, -27.921423, 46.933582,  6.875987, -30.956657, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [0.355000,  0.396058,  1.788435,  0.479912, -4.691644, -1.414906, 24.558617, -49.487778, 33.682228, -0.445604, -1.225691,  3.646858,  8.314348, -8.469375, -49.510471, 111.736870, -73.175804, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [0.310000,  0.922409,  0.850188, -2.400999, -1.308238, -0.194434,  1.550995,  7.041512, -7.188192, -0.291165,  1.814400,  1.902450, -6.805248,  3.252759,  7.605334, -18.574106, 11.452249, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [0.350000,  0.923174, -0.890848, -2.724426,  1.813201,  4.697038, -0.786906, -12.161487,  9.313063,  0.289869,  1.522855, -2.169687, -2.828149,  0.526008, -2.066455, 13.113787, -9.681414, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [0.280000,  0.405364, -1.648502,  0.584094,  1.888018, -5.067538, -2.433504, 10.353470, -5.008752,  0.450742, -0.811435, -3.754631,  1.536981,  6.508108,  0.691915, -10.045361,  5.417976, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [0.280000, -0.000000, -1.307081,  0.025680, -2.227083, -0.149873,  6.715198,  0.436747, -5.870491, -0.000000, -2.045268,  0.019294,  6.123369, -0.167976, -7.269342,  0.757535,  4.199692, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [0.350000, -0.402804, -1.638585, -0.613343,  1.798643,  5.238427, -2.152333, -11.369637,  9.761742, -0.449354, -0.810418,  3.709744,  1.472023, -6.197081,  0.796588,  9.558938, -8.978198, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [0.310000, -0.921641, -0.902698,  2.720152,  1.873546, -4.938235,  0.011555,  8.950330, -7.450928, -0.292459,  1.510049,  2.203491, -2.746664, -0.394144, -3.984396, -5.624136, 10.986733, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [0.355000, -0.923935,  0.843471,  2.410536, -1.314290, -1.348082,  8.357662, -34.365170, 33.743046,  0.288570,  1.825044, -1.883745, -6.864203, -0.270524, -4.999114, 69.546700, -72.682564, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
    [0.526592, -0.398611,  1.790726, -0.461090, -4.687801, -11.594625, 56.469376, -65.124260, 23.302589,  0.447039, -1.218644, -3.672671,  8.277253, 33.003574, -109.726562, 105.579918, -30.419226, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.000000, -0.000000, -0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000],
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
# Flip preview (separate from Poly4D trajectories — uses angle polynomial)
# ---------------------------------------------------------------------------

# Flip segments — generated by: cargo run --release --bin export_poly4d flip
# Two segments via QP planner (3 waypoints: 0 → π → 2π rad)
# Each row: [T_seg, c0..c7]; t_local resets to 0 at each segment boundary.
_FLIP_SEGS = [
    [0.350000, -0.000000247, +0.000003508, +0.000009778, +0.000002773,
     +470.858642578, +113.301620483, -4221.361328125, +5036.835937500],
    [0.350000, +3.141595602, +21.038593292, +0.000606730, -176.377578735,
     -470.869232178, +4206.153320312, -8119.797363281, +5037.533691406],
]
_T_FLIP      = sum(row[0] for row in _FLIP_SEGS)
_FLIP_HEIGHT = HOVER_Z + 0.2   # 1.2 m during flip

# Fast flip segments — generated by: cargo run --release --bin export_poly4d fast_flip
# T=0.45 s (2× faster), peak ω ≈ 1876 °/s
_FAST_FLIP_SEGS = [
    [0.225000, +0.000516237, +0.002698161, +0.004064898, +0.003826565,
     +2753.500488281, +1058.499389648, -59866.273437500, +111039.554687500],
    [0.225000, +3.140736580, +32.718929291, +0.148125976, -663.908203125,
     -2762.185791016, +38330.464843750, -115059.164062500, +111003.046875000],
]
_T_FAST_FLIP      = sum(row[0] for row in _FAST_FLIP_SEGS)
_FAST_FLIP_HEIGHT = HOVER_Z + 0.3   # 1.3 m — extra buffer for faster motion


def _flip_eval(segs, t_global, deriv=0):
    """Evaluate flip angle trajectory at global time t_global."""
    t = t_global
    for seg in segs:
        dur, coeffs = seg[0], seg[1:]
        if t <= dur + 1e-9:
            t_local = min(t, dur)
            if deriv == 0:
                return sum(c * t_local ** i for i, c in enumerate(coeffs))
            else:
                return sum(i * c * t_local ** (i - 1) for i, c in enumerate(coeffs) if i > 0)
        t -= dur
    dur, coeffs = segs[-1][0], segs[-1][1:]
    if deriv == 0:
        return sum(c * dur ** i for i, c in enumerate(coeffs))
    else:
        return sum(i * c * dur ** (i - 1) for i, c in enumerate(coeffs) if i > 0)


def plot_flip_path():
    """Flip orientation visualisation: stroboscopic side-view + pitch angle timeline.

    A Crazyflie flip is an IN-PLACE rotation — the drone centre of mass stays at
    roughly (x0, y0, FLIP_HEIGHT) throughout.  Only the ATTITUDE changes: pitch
    goes 0° → 360° over T_FLIP seconds.

    Left panel: stroboscopic XZ side-view — drone body drawn as a cross (+) at
        8 equally-spaced time steps so you can read the rotation directly.
        Ground and hover height are marked for spatial context.
    Right panel: pitch angle θ(t) and angular rate ω(t) vs time — the actual
        planned trajectory output from the QP motion planner.
    """
    N        = 500
    ts       = np.linspace(0, _T_FLIP, N)
    theta    = np.array([_flip_eval(_FLIP_SEGS, t)          for t in ts])
    omega_ds = np.degrees([_flip_eval(_FLIP_SEGS, t, deriv=1) for t in ts])
    omega_abs = np.abs(omega_ds)
    omega_max = omega_abs.max()

    # Drone centre stays fixed in space
    CX, CZ = 0.0, _FLIP_HEIGHT     # world position of drone centre [m]
    ARM    = 0.10                   # display arm length (exaggerated for clarity)
    ARM_CF = 0.046                  # real CF arm [m] shown in annotation

    # Body x-axis (forward) in world: [cos(θ), 0, −sin(θ)]   (pitch around y)
    # Body z-axis (up)      in world: [sin(θ), 0,  cos(θ)]

    folder = os.path.join(OUT_DIR, "flip")
    os.makedirs(folder, exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=(16, 7),
                             gridspec_kw={'width_ratios': [1.2, 1]})
    fig.suptitle(
        f"Flip Manoeuvre  —  in-place 360° pitch rotation  |  "
        f"T = {_T_FLIP:.3f} s  |  peak ω = {omega_max:.0f} °/s  |  "
        f"hover height = {_FLIP_HEIGHT:.1f} m",
        fontsize=13, fontweight='bold',
    )

    # ── Left: stroboscopic side view ─────────────────────────────────────────
    ax = axes[0]

    # Shaded background bands
    ax.axhspan(0,            0.05,          color='saddlebrown', alpha=0.25, label='ground')
    ax.axhspan(_FLIP_HEIGHT - 0.35,
               _FLIP_HEIGHT + 0.35, color='lightblue',    alpha=0.15, label='flight zone')
    ax.axhline(CZ, color='gray', lw=0.8, ls='--', alpha=0.5)

    # 8 stroboscopic snapshots evenly spread over the full 360°
    n_snaps  = 8
    snap_idx = np.linspace(0, N - 1, n_snaps, dtype=int)
    cmap     = plt.cm.plasma

    for k, i in enumerate(snap_idx):
        th   = theta[i]
        frac = k / (n_snaps - 1)
        col  = cmap(frac)
        t_s  = ts[i]
        deg  = np.degrees(th) % 360

        # Forward/rear arm along body x-axis
        fx =  ARM * np.cos(th);  fz = CZ - ARM * np.sin(th)
        rx = -ARM * np.cos(th);  rz = CZ + ARM * np.sin(th)
        # Up/down arm along body z-axis
        ux =  ARM * np.sin(th);  uz = CZ + ARM * np.cos(th)
        dx = -ARM * np.sin(th);  dz = CZ - ARM * np.cos(th)

        ax.plot([rx, fx], [rz, fz], color=col, lw=3,   alpha=0.85, zorder=3)
        ax.plot([dx, ux], [dz, uz], color=col, lw=1.5, alpha=0.55, zorder=3, ls='--')
        # Arrow showing forward direction
        ax.annotate('', xy=(fx, fz), xytext=(fx - 0.015*np.cos(th),
                                              fz + 0.015*np.sin(th)),
                    arrowprops=dict(arrowstyle='->', color=col, lw=1.8),
                    zorder=4)
        # Label each snapshot with angle and time
        lx = fx + 0.025 * np.cos(th)
        lz = fz - 0.025 * np.sin(th)
        ax.text(lx, lz, f"{deg:.0f}°\nt={t_s:.2f}s",
                fontsize=6.5, color=col, ha='center', va='center', zorder=5)

    # Drone centre dot
    ax.plot(CX, CZ, 'k+', ms=14, mew=2.5, zorder=6, label='drone centre (fixed)')

    # Colourbar for time progression
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, _T_FLIP))
    sm.set_array([])
    fig.colorbar(sm, ax=ax, label='time [s]', fraction=0.046, pad=0.04)

    ax.set_xlim(-ARM * 1.6, ARM * 1.6)
    ax.set_ylim(-0.05, _FLIP_HEIGHT + ARM * 1.6)
    ax.set_xlabel('x [m]  (forward ←→ backward)', fontsize=10)
    ax.set_ylabel('z [m]  (height)',               fontsize=10)
    ax.set_title(
        'Stroboscopic side view (XZ plane)\n'
        'Solid bar = body x-axis (forward); dashed = body z-axis (up)\n'
        f'Drone centre stays fixed at z = {_FLIP_HEIGHT:.1f} m throughout',
        fontsize=9,
    )
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8, loc='upper right')

    # ── Right: pitch angle + rate vs time ────────────────────────────────────
    ax2  = axes[1]
    ax2b = ax2.twinx()

    l1, = ax2.plot(ts, np.degrees(theta), color='tab:blue',   lw=2,   label='θ pitch [°]')
    l2, = ax2b.plot(ts, omega_ds,         color='tab:orange', lw=1.5, label='ω pitch [°/s]', ls='--')

    for deg_mark, label in [(90, '90° (nose down)'), (180, '180° (inverted)'),
                             (270, '270° (nose up)'), (360, '360° (upright)')]:
        ax2.axhline(deg_mark, color='gray', lw=0.7, ls=':', alpha=0.6)
        ax2.text(_T_FLIP * 0.01, deg_mark + 5, label, fontsize=7, color='gray')

    # Mark segment boundary
    ax2.axvline(_T_FLIP / 2, color='gray', lw=0.8, alpha=0.4, label='segment boundary')

    ax2.set_xlabel('time [s]', fontsize=10)
    ax2.set_ylabel('pitch angle [°]', fontsize=10, color='tab:blue')
    ax2b.set_ylabel('pitch rate [°/s]', fontsize=10, color='tab:orange')
    ax2.set_ylim(-10, 380)
    ax2.set_title(f'QP-planned angle trajectory\npeak ω = {omega_max:.0f} °/s  at  '
                  f't ≈ {ts[np.argmax(omega_abs)]:.3f} s', fontsize=9)
    ax2.legend([l1, l2], [l.get_label() for l in [l1, l2]], fontsize=9)
    ax2.grid(True, alpha=0.3)

    _save(fig, folder, "flip_path.png")
    return fig


def plot_flip_preview():
    """4-panel preview: pitch angle, angular rate, thrust, z-setpoint."""
    ts = np.linspace(0, _T_FLIP, 300)
    theta_deg = np.degrees([_flip_eval(_FLIP_SEGS, t) for t in ts])
    omega_dps = np.degrees([_flip_eval(_FLIP_SEGS, t, deriv=1) for t in ts])
    omega_max = np.abs(omega_dps).max()

    folder = os.path.join(OUT_DIR, "flip")
    os.makedirs(folder, exist_ok=True)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(
        f"Flip Preview  —  T={_T_FLIP:.3f} s  |  peak rate={omega_max:.0f} °/s  |  "
        f"height={_FLIP_HEIGHT:.1f} m",
        fontsize=12, fontweight='bold',
    )

    # Panel 1: Pitch angle 0 → 360°
    ax = axes[0, 0]
    ax.plot(ts, theta_deg, color='tab:blue', lw=2)
    ax.axhline(180, color='gray', lw=0.8, ls='--', alpha=0.6, label='inverted (180°)')
    ax.axhline(360, color='green', lw=0.8, ls='--', alpha=0.6, label='upright (360°)')
    ax.set_xlabel('time [s]'); ax.set_ylabel('pitch angle [°]')
    ax.set_title('Pitch Angle θ(t)  (0 → 360°)')
    ax.set_ylim(-5, 370); ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    # Panel 2: Angular rate
    ax = axes[0, 1]
    ax.plot(ts, omega_dps, color='tab:orange', lw=2)
    ax.axhline(0, color='k', lw=0.5, alpha=0.4)
    ax.set_xlabel('time [s]'); ax.set_ylabel('pitch rate [°/s]')
    ax.set_title(f'Angular Rate ω(t)  (peak = {omega_max:.0f} °/s)')
    ax.grid(True, alpha=0.3)

    # Panel 3: Thrust profile (constant hover)
    ax = axes[1, 0]
    thrust_line = np.full_like(ts, CF_HOVER_N)
    ax.plot(ts, thrust_line, color='tab:blue', lw=2, label=f'setpoint ({CF_HOVER_N:.3f} N)')
    ax.axhline(CF_MAX_THRUST_N, color='red', lw=1.2, ls='--',
               label=f'max ({CF_MAX_THRUST_N:.2f} N)')
    ax.fill_between(ts, CF_HOVER_N, CF_MAX_THRUST_N, alpha=0.08, color='green',
                    label=f'{100*(1-CF_HOVER_N/CF_MAX_THRUST_N):.0f}% margin')
    ax.set_xlabel('time [s]'); ax.set_ylabel('thrust setpoint [N]')
    ax.set_title('Thrust Setpoint  (held at hover during flip)')
    ax.set_ylim(0, CF_MAX_THRUST_N * 1.15)
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    # Panel 4: Z setpoint
    ax = axes[1, 1]
    z_line = np.full_like(ts, _FLIP_HEIGHT)
    ax.plot(ts, z_line, color='tab:green', lw=2, label=f'setpoint = {_FLIP_HEIGHT:.1f} m')
    ax.axhspan(0, 0.1, color='red', alpha=0.2, label='ground (<0.1 m)')
    ax.set_xlabel('time [s]'); ax.set_ylabel('z setpoint [m]')
    ax.set_title('Z Setpoint  (held constant; actual z may dip ~0.1–0.3 m when inverted)')
    ax.set_ylim(0, _FLIP_HEIGHT * 1.3)
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    _save(fig, folder, "flip_preview.png")
    return fig


# ---------------------------------------------------------------------------
# Fast Flip preview — mirrors plot_flip_path/preview with faster profile
# ---------------------------------------------------------------------------

def plot_fast_flip_path():
    """Stroboscopic side-view + pitch angle timeline for the fast flip (T=0.45 s)."""
    N        = 500
    ts       = np.linspace(0, _T_FAST_FLIP, N)
    theta    = np.array([_flip_eval(_FAST_FLIP_SEGS, t)          for t in ts])
    omega_ds = np.degrees([_flip_eval(_FAST_FLIP_SEGS, t, deriv=1) for t in ts])
    omega_abs = np.abs(omega_ds)
    omega_max = omega_abs.max()

    CX, CZ = 0.0, _FAST_FLIP_HEIGHT
    ARM    = 0.10

    folder = os.path.join(OUT_DIR, "fast_flip")
    os.makedirs(folder, exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=(16, 7),
                             gridspec_kw={'width_ratios': [1.2, 1]})
    fig.suptitle(
        f"Fast Flip  —  in-place 360° pitch  |  T = {_T_FAST_FLIP:.3f} s  |  "
        f"peak ω = {omega_max:.0f} °/s  |  height = {_FAST_FLIP_HEIGHT:.1f} m",
        fontsize=13, fontweight='bold',
    )

    ax = axes[0]
    ax.axhspan(0, 0.05, color='saddlebrown', alpha=0.25, label='ground')
    ax.axhspan(_FAST_FLIP_HEIGHT - 0.35, _FAST_FLIP_HEIGHT + 0.35,
               color='lightblue', alpha=0.15, label='flight zone')
    ax.axhline(CZ, color='gray', lw=0.8, ls='--', alpha=0.5)

    n_snaps  = 8
    snap_idx = np.linspace(0, N - 1, n_snaps, dtype=int)
    cmap     = plt.cm.plasma

    for k, i in enumerate(snap_idx):
        th  = theta[i]
        col = cmap(k / (n_snaps - 1))
        t_s = ts[i]
        deg = np.degrees(th) % 360
        fx =  ARM * np.cos(th);  fz = CZ - ARM * np.sin(th)
        rx = -ARM * np.cos(th);  rz = CZ + ARM * np.sin(th)
        ux =  ARM * np.sin(th);  uz = CZ + ARM * np.cos(th)
        dx = -ARM * np.sin(th);  dz = CZ - ARM * np.cos(th)
        ax.plot([rx, fx], [rz, fz], color=col, lw=3,   alpha=0.85, zorder=3)
        ax.plot([dx, ux], [dz, uz], color=col, lw=1.5, alpha=0.55, zorder=3, ls='--')
        ax.annotate('', xy=(fx, fz), xytext=(fx - 0.015*np.cos(th), fz + 0.015*np.sin(th)),
                    arrowprops=dict(arrowstyle='->', color=col, lw=1.8), zorder=4)
        lx = fx + 0.025 * np.cos(th)
        lz = fz - 0.025 * np.sin(th)
        ax.text(lx, lz, f"{deg:.0f}°\nt={t_s:.2f}s",
                fontsize=6.5, color=col, ha='center', va='center', zorder=5)

    ax.plot(CX, CZ, 'k+', ms=14, mew=2.5, zorder=6, label='drone centre (fixed)')
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, _T_FAST_FLIP))
    sm.set_array([])
    fig.colorbar(sm, ax=ax, label='time [s]', fraction=0.046, pad=0.04)
    ax.set_xlim(-ARM * 1.6, ARM * 1.6)
    ax.set_ylim(-0.05, _FAST_FLIP_HEIGHT + ARM * 1.6)
    ax.set_xlabel('x [m]  (forward ←→ backward)', fontsize=10)
    ax.set_ylabel('z [m]  (height)',               fontsize=10)
    ax.set_title(
        'Stroboscopic side view (XZ plane)  [FAST]\n'
        f'Drone centre stays fixed at z = {_FAST_FLIP_HEIGHT:.1f} m', fontsize=9)
    ax.set_aspect('equal'); ax.grid(True, alpha=0.25); ax.legend(fontsize=8, loc='upper right')

    ax2  = axes[1]
    ax2b = ax2.twinx()
    l1, = ax2.plot(ts, np.degrees(theta), color='tab:blue',   lw=2,   label='θ pitch [°]')
    l2, = ax2b.plot(ts, omega_ds,         color='tab:orange', lw=1.5, label='ω pitch [°/s]', ls='--')
    for deg_mark, label in [(90, '90°'), (180, '180° (inv)'), (270, '270°'), (360, '360°')]:
        ax2.axhline(deg_mark, color='gray', lw=0.7, ls=':', alpha=0.6)
        ax2.text(_T_FAST_FLIP * 0.01, deg_mark + 5, label, fontsize=7, color='gray')
    ax2.axvline(_T_FAST_FLIP / 2, color='gray', lw=0.8, alpha=0.4)
    ax2.set_xlabel('time [s]', fontsize=10)
    ax2.set_ylabel('pitch angle [°]', fontsize=10, color='tab:blue')
    ax2b.set_ylabel('pitch rate [°/s]', fontsize=10, color='tab:orange')
    ax2.set_ylim(-10, 380)
    ax2.set_title(f'QP-planned angle  (FAST T={_T_FAST_FLIP:.3f}s)\npeak ω = {omega_max:.0f} °/s', fontsize=9)
    ax2.legend([l1, l2], [l.get_label() for l in [l1, l2]], fontsize=9)
    ax2.grid(True, alpha=0.3)

    _save(fig, folder, "fast_flip_path.png")
    return fig


def plot_fast_flip_preview():
    """4-panel preview for fast flip (T=0.45 s)."""
    ts = np.linspace(0, _T_FAST_FLIP, 300)
    theta_deg = np.degrees([_flip_eval(_FAST_FLIP_SEGS, t) for t in ts])
    omega_dps = np.degrees([_flip_eval(_FAST_FLIP_SEGS, t, deriv=1) for t in ts])
    omega_max = np.abs(omega_dps).max()

    folder = os.path.join(OUT_DIR, "fast_flip")
    os.makedirs(folder, exist_ok=True)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(
        f"Fast Flip Preview  —  T={_T_FAST_FLIP:.3f} s  |  peak={omega_max:.0f} °/s  |  "
        f"height={_FAST_FLIP_HEIGHT:.1f} m",
        fontsize=12, fontweight='bold',
    )

    ax = axes[0, 0]
    ax.plot(ts, theta_deg, color='tab:blue', lw=2)
    ax.axhline(180, color='gray', lw=0.8, ls='--', alpha=0.6, label='inverted (180°)')
    ax.axhline(360, color='green', lw=0.8, ls='--', alpha=0.6, label='upright (360°)')
    ax.set_xlabel('time [s]'); ax.set_ylabel('pitch angle [°]')
    ax.set_title('Pitch Angle θ(t)  [FAST — 0 → 360°]')
    ax.set_ylim(-5, 370); ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(ts, omega_dps, color='tab:orange', lw=2)
    ax.axhline(0, color='k', lw=0.5, alpha=0.4)
    ax.set_xlabel('time [s]'); ax.set_ylabel('pitch rate [°/s]')
    ax.set_title(f'Angular Rate ω(t)  (peak = {omega_max:.0f} °/s)')
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    thrust_line = np.full_like(ts, CF_HOVER_N)
    ax.plot(ts, thrust_line, color='tab:blue', lw=2, label=f'setpoint ({CF_HOVER_N:.3f} N)')
    ax.axhline(CF_MAX_THRUST_N, color='red', lw=1.2, ls='--', label=f'max ({CF_MAX_THRUST_N:.2f} N)')
    ax.fill_between(ts, CF_HOVER_N, CF_MAX_THRUST_N, alpha=0.08, color='green',
                    label=f'{100*(1-CF_HOVER_N/CF_MAX_THRUST_N):.0f}% margin')
    ax.set_xlabel('time [s]'); ax.set_ylabel('thrust setpoint [N]')
    ax.set_title('Thrust Setpoint  (hover during flip)')
    ax.set_ylim(0, CF_MAX_THRUST_N * 1.15); ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    z_line = np.full_like(ts, _FAST_FLIP_HEIGHT)
    ax.plot(ts, z_line, color='tab:green', lw=2, label=f'setpoint = {_FAST_FLIP_HEIGHT:.1f} m')
    ax.axhspan(0, 0.1, color='red', alpha=0.2, label='ground (<0.1 m)')
    ax.set_xlabel('time [s]'); ax.set_ylabel('z setpoint [m]')
    ax.set_title('Z Setpoint  (held constant)')
    ax.set_ylim(0, _FAST_FLIP_HEIGHT * 1.3); ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    _save(fig, folder, "fast_flip_preview.png")
    return fig


# ---------------------------------------------------------------------------
# Fast Roll preview
# ---------------------------------------------------------------------------

_FAST_ROLL_SEGS   = _FAST_FLIP_SEGS
_T_FAST_ROLL      = _T_FAST_FLIP
_FAST_ROLL_HEIGHT = _FAST_FLIP_HEIGHT


def plot_fast_roll_path():
    """YZ stroboscopic side view + angle/rate timeline for the fast right roll (T=0.45 s)."""
    N        = 500
    ts       = np.linspace(0, _T_FAST_ROLL, N)
    theta    = np.array([_flip_eval(_FAST_ROLL_SEGS, t)          for t in ts])
    omega_ds = np.degrees([_flip_eval(_FAST_ROLL_SEGS, t, deriv=1) for t in ts])
    omega_abs = np.abs(omega_ds)
    omega_max = omega_abs.max()

    ARM = 0.10
    folder = os.path.join(OUT_DIR, "fast_roll")
    os.makedirs(folder, exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=(16, 7),
                             gridspec_kw={'width_ratios': [1.2, 1]})
    fig.suptitle(
        f"Fast Roll  —  in-place 360° right roll  |  T = {_T_FAST_ROLL:.3f} s  |  "
        f"peak ω = {omega_max:.0f} °/s  |  height = {_FAST_ROLL_HEIGHT:.1f} m",
        fontsize=13, fontweight='bold',
    )

    ax = axes[0]
    ax.axhspan(0, 0.05, color='saddlebrown', alpha=0.25, label='ground')
    ax.axhspan(_FAST_ROLL_HEIGHT - 0.35, _FAST_ROLL_HEIGHT + 0.35,
               color='lightblue', alpha=0.15, label='flight zone')
    ax.axhline(_FAST_ROLL_HEIGHT, color='gray', lw=0.8, ls='--', alpha=0.5)

    n_snaps  = 8
    snap_idx = np.linspace(0, N - 1, n_snaps, dtype=int)
    cmap     = plt.cm.plasma

    for k, i in enumerate(snap_idx):
        th   = theta[i]
        col  = cmap(k / (n_snaps - 1))
        t_s  = ts[i]
        deg  = np.degrees(th) % 360
        fy = ARM * np.cos(th);   fz_w = _FAST_ROLL_HEIGHT + ARM * np.sin(th)
        ry = -ARM * np.cos(th);  rz_w = _FAST_ROLL_HEIGHT - ARM * np.sin(th)
        uy = -ARM * np.sin(th);  uz_w = _FAST_ROLL_HEIGHT + ARM * np.cos(th)
        dy =  ARM * np.sin(th);  dz_w = _FAST_ROLL_HEIGHT - ARM * np.cos(th)
        ax.plot([ry, fy], [rz_w, fz_w], color=col, lw=3,   alpha=0.85, zorder=3)
        ax.plot([dy, uy], [dz_w, uz_w], color=col, lw=1.5, alpha=0.55, ls='--', zorder=3)
        ax.annotate('', xy=(fy, fz_w), xytext=(fy - 0.015*np.cos(th), fz_w - 0.015*np.sin(th)),
                    arrowprops=dict(arrowstyle='->', color=col, lw=1.8), zorder=4)
        lx = fy + 0.025 * np.cos(th)
        lz = fz_w + 0.025 * np.sin(th)
        ax.text(lx, lz, f"{deg:.0f}°\nt={t_s:.2f}s",
                fontsize=6.5, color=col, ha='center', va='center', zorder=5)

    ax.plot(0, _FAST_ROLL_HEIGHT, 'k+', ms=14, mew=2.5, zorder=6, label='drone centre (fixed)')
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, _T_FAST_ROLL))
    sm.set_array([])
    fig.colorbar(sm, ax=ax, label='time [s]', fraction=0.046, pad=0.04)
    ax.set_xlim(-ARM * 1.6, ARM * 1.6)
    ax.set_ylim(-0.05, _FAST_ROLL_HEIGHT + ARM * 1.6)
    ax.set_xlabel('y [m]  (left ←→ right)', fontsize=10)
    ax.set_ylabel('z [m]  (height)',         fontsize=10)
    ax.set_title(
        'Stroboscopic front view (YZ plane)  [FAST]\n'
        f'Drone centre stays fixed at z = {_FAST_ROLL_HEIGHT:.1f} m', fontsize=9)
    ax.set_aspect('equal'); ax.grid(True, alpha=0.25); ax.legend(fontsize=8, loc='upper right')

    ax2  = axes[1]
    ax2b = ax2.twinx()
    l1, = ax2.plot(ts, np.degrees(theta), color='tab:blue',   lw=2,   label='θ roll [°]')
    l2, = ax2b.plot(ts, omega_ds,         color='tab:orange', lw=1.5, label='ω roll [°/s]', ls='--')
    for deg_mark, label in [(90, '90°'), (180, '180° (inv)'), (270, '270°'), (360, '360°')]:
        ax2.axhline(deg_mark, color='gray', lw=0.7, ls=':', alpha=0.6)
        ax2.text(_T_FAST_ROLL * 0.01, deg_mark + 5, label, fontsize=7, color='gray')
    ax2.axvline(_T_FAST_ROLL / 2, color='gray', lw=0.8, alpha=0.4)
    ax2.set_xlabel('time [s]', fontsize=10)
    ax2.set_ylabel('roll angle [°]', fontsize=10, color='tab:blue')
    ax2b.set_ylabel('roll rate [°/s]', fontsize=10, color='tab:orange')
    ax2.set_ylim(-10, 380)
    ax2.set_title(f'QP-planned angle  (FAST T={_T_FAST_ROLL:.3f}s)\npeak ω = {omega_max:.0f} °/s', fontsize=9)
    ax2.legend([l1, l2], [l.get_label() for l in [l1, l2]], fontsize=9)
    ax2.grid(True, alpha=0.3)

    _save(fig, folder, "fast_roll_path.png")
    return fig


def plot_fast_roll_preview():
    """4-panel preview for fast roll (T=0.45 s)."""
    ts = np.linspace(0, _T_FAST_ROLL, 300)
    theta_deg = np.degrees([_flip_eval(_FAST_ROLL_SEGS, t) for t in ts])
    omega_dps = np.degrees([_flip_eval(_FAST_ROLL_SEGS, t, deriv=1) for t in ts])
    omega_max = np.abs(omega_dps).max()

    folder = os.path.join(OUT_DIR, "fast_roll")
    os.makedirs(folder, exist_ok=True)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(
        f"Fast Roll Preview  —  T={_T_FAST_ROLL:.3f} s  |  peak={omega_max:.0f} °/s  |  "
        f"height={_FAST_ROLL_HEIGHT:.1f} m",
        fontsize=12, fontweight='bold',
    )

    ax = axes[0, 0]
    ax.plot(ts, theta_deg, color='tab:blue', lw=2)
    ax.axhline(90,  color='gray', lw=0.8, ls='--', alpha=0.6, label='right side down (90°)')
    ax.axhline(180, color='gray', lw=0.8, ls='--', alpha=0.6, label='inverted (180°)')
    ax.axhline(360, color='green', lw=0.8, ls='--', alpha=0.6, label='upright (360°)')
    ax.set_xlabel('time [s]'); ax.set_ylabel('roll angle [°]')
    ax.set_title('Roll Angle θ(t)  [FAST — 0 → 360°, right roll]')
    ax.set_ylim(-5, 375); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(ts, omega_dps, color='tab:orange', lw=2)
    ax.axhline(0, color='k', lw=0.5, alpha=0.4)
    ax.set_xlabel('time [s]'); ax.set_ylabel('roll rate [°/s]')
    ax.set_title(f'Angular Rate ωx(t)  (peak = {omega_max:.0f} °/s)')
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    thrust_line = np.full_like(ts, CF_HOVER_N)
    ax.plot(ts, thrust_line, color='tab:blue', lw=2, label=f'setpoint ({CF_HOVER_N:.3f} N)')
    ax.axhline(CF_MAX_THRUST_N, color='red', lw=1.2, ls='--', label=f'max ({CF_MAX_THRUST_N:.2f} N)')
    ax.fill_between(ts, CF_HOVER_N, CF_MAX_THRUST_N, alpha=0.08, color='green',
                    label=f'{100*(1-CF_HOVER_N/CF_MAX_THRUST_N):.0f}% margin')
    ax.set_xlabel('time [s]'); ax.set_ylabel('thrust setpoint [N]')
    ax.set_title('Thrust Setpoint  (held at hover)')
    ax.set_ylim(0, CF_MAX_THRUST_N * 1.15); ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    z_line = np.full_like(ts, _FAST_ROLL_HEIGHT)
    ax.plot(ts, z_line, color='tab:green', lw=2, label=f'setpoint = {_FAST_ROLL_HEIGHT:.1f} m')
    ax.axhspan(0, 0.1, color='red', alpha=0.2, label='ground (<0.1 m)')
    ax.set_xlabel('time [s]'); ax.set_ylabel('z setpoint [m]')
    ax.set_title('Z Setpoint  (held constant)')
    ax.set_ylim(0, _FAST_ROLL_HEIGHT * 1.3); ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    _save(fig, folder, "fast_roll_preview.png")
    return fig


# ---------------------------------------------------------------------------
# Main — define trajectories and run
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Roll preview — stroboscopic YZ side view (rotation around body x-axis)
# ---------------------------------------------------------------------------

# Reuses same QP coefficients as flip — only axis changes
_ROLL_SEGS  = _FLIP_SEGS
_T_ROLL     = _T_FLIP
_ROLL_HEIGHT = _FLIP_HEIGHT


def plot_roll_path():
    """YZ stroboscopic side view + angle/rate timeline for a right roll."""
    N        = 500
    ts       = np.linspace(0, _T_ROLL, N)
    theta    = np.array([_flip_eval(_ROLL_SEGS, t)          for t in ts])
    omega_ds = np.degrees([_flip_eval(_ROLL_SEGS, t, deriv=1) for t in ts])
    omega_abs = np.abs(omega_ds)
    omega_max = omega_abs.max()

    # Pure roll around body x-axis.
    # Body y-axis (left wing) in world: Rx(θ)*[0,1,0] = [0, cos(θ), sin(θ)]
    # Body z-axis (up)        in world: Rx(θ)*[0,0,1] = [0,-sin(θ), cos(θ)]
    ARM = 0.10   # exaggerated display arm

    folder = os.path.join(OUT_DIR, "roll")
    os.makedirs(folder, exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=(16, 7),
                             gridspec_kw={'width_ratios': [1.2, 1]})
    fig.suptitle(
        f"Roll Manoeuvre  —  in-place 360° right roll  |  "
        f"T = {_T_ROLL:.3f} s  |  peak ω = {omega_max:.0f} °/s  |  "
        f"height = {_ROLL_HEIGHT:.1f} m",
        fontsize=13, fontweight='bold',
    )

    # ── Left: stroboscopic YZ front view ─────────────────────────────────────
    ax = axes[0]
    ax.axhspan(0, 0.05, color='saddlebrown', alpha=0.25, label='ground')
    ax.axhspan(_ROLL_HEIGHT - 0.35, _ROLL_HEIGHT + 0.35,
               color='lightblue', alpha=0.15, label='flight zone')
    ax.axhline(_ROLL_HEIGHT, color='gray', lw=0.8, ls='--', alpha=0.5)

    n_snaps  = 8
    snap_idx = np.linspace(0, N - 1, n_snaps, dtype=int)
    cmap     = plt.cm.plasma

    for k, i in enumerate(snap_idx):
        th   = theta[i]
        col  = cmap(k / (n_snaps - 1))
        t_s  = ts[i]
        deg  = np.degrees(th) % 360

        # Body y-axis (wing tip) in YZ: [cos(θ), sin(θ)]
        # Body z-axis (top)      in YZ: [-sin(θ), cos(θ)]
        fy = ARM * np.cos(th);   fz_w = _ROLL_HEIGHT + ARM * np.sin(th)   # right tip
        ry = -ARM * np.cos(th);  rz_w = _ROLL_HEIGHT - ARM * np.sin(th)   # left tip
        uy = -ARM * np.sin(th);  uz_w = _ROLL_HEIGHT + ARM * np.cos(th)   # top
        dy =  ARM * np.sin(th);  dz_w = _ROLL_HEIGHT - ARM * np.cos(th)   # bottom

        ax.plot([ry, fy], [rz_w, fz_w], color=col, lw=3,   alpha=0.85, zorder=3)
        ax.plot([dy, uy], [dz_w, uz_w], color=col, lw=1.5, alpha=0.55, ls='--', zorder=3)
        ax.annotate('', xy=(fy, fz_w), xytext=(fy - 0.015*np.cos(th),
                                                fz_w - 0.015*np.sin(th)),
                    arrowprops=dict(arrowstyle='->', color=col, lw=1.8), zorder=4)
        lx = fy + 0.025 * np.cos(th)
        lz = fz_w + 0.025 * np.sin(th)
        ax.text(lx, lz, f"{deg:.0f}°\nt={t_s:.2f}s",
                fontsize=6.5, color=col, ha='center', va='center', zorder=5)

    ax.plot(0, _ROLL_HEIGHT, 'k+', ms=14, mew=2.5, zorder=6, label='drone centre (fixed)')
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, _T_ROLL))
    sm.set_array([])
    fig.colorbar(sm, ax=ax, label='time [s]', fraction=0.046, pad=0.04)

    ax.set_xlim(-ARM * 1.6, ARM * 1.6)
    ax.set_ylim(-0.05, _ROLL_HEIGHT + ARM * 1.6)
    ax.set_xlabel('y [m]  (left ←→ right)', fontsize=10)
    ax.set_ylabel('z [m]  (height)',         fontsize=10)
    ax.set_title(
        'Stroboscopic front view (YZ plane)\n'
        'Solid bar = wing axis (left-right); dashed = body z-axis (up)\n'
        f'Drone centre stays fixed at z = {_ROLL_HEIGHT:.1f} m throughout',
        fontsize=9,
    )
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8, loc='upper right')

    # ── Right: roll angle + rate vs time ─────────────────────────────────────
    ax2  = axes[1]
    ax2b = ax2.twinx()
    l1, = ax2.plot(ts, np.degrees(theta), color='tab:blue',   lw=2,   label='θ roll [°]')
    l2, = ax2b.plot(ts, omega_ds,         color='tab:orange', lw=1.5, label='ω roll [°/s]', ls='--')
    for deg_mark, label in [(90, '90° (right side down)'), (180, '180° (inverted)'),
                             (270, '270° (left side down)'), (360, '360° (upright)')]:
        ax2.axhline(deg_mark, color='gray', lw=0.7, ls=':', alpha=0.6)
        ax2.text(_T_ROLL * 0.01, deg_mark + 5, label, fontsize=7, color='gray')
    ax2.axvline(_T_ROLL / 2, color='gray', lw=0.8, alpha=0.4)
    ax2.set_xlabel('time [s]', fontsize=10)
    ax2.set_ylabel('roll angle [°]', fontsize=10, color='tab:blue')
    ax2b.set_ylabel('roll rate [°/s]', fontsize=10, color='tab:orange')
    ax2.set_ylim(-10, 380)
    ax2.set_title(f'QP-planned angle trajectory\npeak ω = {omega_max:.0f} °/s', fontsize=9)
    ax2.legend([l1, l2], [l.get_label() for l in [l1, l2]], fontsize=9)
    ax2.grid(True, alpha=0.3)

    _save(fig, folder, "roll_path.png")
    return fig


def plot_roll_preview():
    """4-panel preview: roll angle, angular rate, thrust, z-setpoint."""
    ts = np.linspace(0, _T_ROLL, 300)
    theta_deg = np.degrees([_flip_eval(_ROLL_SEGS, t) for t in ts])
    omega_dps = np.degrees([_flip_eval(_ROLL_SEGS, t, deriv=1) for t in ts])
    omega_max = np.abs(omega_dps).max()

    folder = os.path.join(OUT_DIR, "roll")
    os.makedirs(folder, exist_ok=True)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(
        f"Roll Preview  —  T={_T_ROLL:.3f} s  |  peak rate={omega_max:.0f} °/s  |  "
        f"height={_ROLL_HEIGHT:.1f} m",
        fontsize=12, fontweight='bold',
    )

    # Panel 1: Roll angle 0 → 360°
    ax = axes[0, 0]
    ax.plot(ts, theta_deg, color='tab:blue', lw=2)
    ax.axhline(90,  color='gray', lw=0.8, ls='--', alpha=0.6, label='right side down (90°)')
    ax.axhline(180, color='gray', lw=0.8, ls='--', alpha=0.6, label='inverted (180°)')
    ax.axhline(270, color='gray', lw=0.8, ls='--', alpha=0.6, label='left side down (270°)')
    ax.axhline(360, color='green', lw=0.8, ls='--', alpha=0.6, label='upright (360°)')
    ax.set_xlabel('time [s]'); ax.set_ylabel('roll angle [°]')
    ax.set_title('Roll Angle θ(t)  (0 → 360°, right roll)')
    ax.set_ylim(-5, 375); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # Panel 2: Angular rate
    ax = axes[0, 1]
    ax.plot(ts, omega_dps, color='tab:orange', lw=2)
    ax.axhline(0, color='k', lw=0.5, alpha=0.4)
    ax.set_xlabel('time [s]'); ax.set_ylabel('roll rate [°/s]')
    ax.set_title(f'Angular Rate ωx(t)  (peak = {omega_max:.0f} °/s)')
    ax.grid(True, alpha=0.3)

    # Panel 3: Thrust profile (constant hover — no collective change during in-place roll)
    ax = axes[1, 0]
    thrust_line = np.full_like(ts, CF_HOVER_N)
    ax.plot(ts, thrust_line, color='tab:blue', lw=2, label=f'setpoint ({CF_HOVER_N:.3f} N)')
    ax.axhline(CF_MAX_THRUST_N, color='red', lw=1.2, ls='--',
               label=f'max ({CF_MAX_THRUST_N:.2f} N)')
    ax.fill_between(ts, CF_HOVER_N, CF_MAX_THRUST_N, alpha=0.08, color='green',
                    label=f'{100*(1-CF_HOVER_N/CF_MAX_THRUST_N):.0f}% margin')
    ax.set_xlabel('time [s]'); ax.set_ylabel('thrust setpoint [N]')
    ax.set_title('Thrust Setpoint  (held at hover; motor differential provides roll torque)')
    ax.set_ylim(0, CF_MAX_THRUST_N * 1.15)
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    # Panel 4: Z setpoint
    ax = axes[1, 1]
    z_line = np.full_like(ts, _ROLL_HEIGHT)
    ax.plot(ts, z_line, color='tab:green', lw=2, label=f'setpoint = {_ROLL_HEIGHT:.1f} m')
    ax.axhspan(0, 0.1, color='red', alpha=0.2, label='ground (<0.1 m)')
    ax.set_xlabel('time [s]'); ax.set_ylabel('z setpoint [m]')
    ax.set_title('Z Setpoint  (held constant; actual z may dip ~0.1–0.3 m when inverted)')
    ax.set_ylim(0, _ROLL_HEIGHT * 1.3)
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    _save(fig, folder, "roll_preview.png")
    return fig


# ---------------------------------------------------------------------------
# Helix preview — 3D spiral path + height/speed profile
# ---------------------------------------------------------------------------

_HELIX_RADIUS = 0.30   # m
_HELIX_DZ     = 0.40   # m
_LAP_TIME     = 10.5   # s
_OMEGA_C      = 2 * np.pi / _LAP_TIME
_VZ_UP        = _HELIX_DZ / _LAP_TIME
_T_HELIX      = 2.0 * _LAP_TIME


def plot_helix_path():
    """3D helix spiral + height and speed profile."""
    N    = 800
    ts   = np.linspace(0, _T_HELIX, N)
    ph0  = np.pi   # start phase (drone begins at hover position = leftmost point)
    ph   = ph0 + _OMEGA_C * ts

    xs   = HOVER_Z + _HELIX_RADIUS * np.cos(ph)   # offset so centre is visible
    ys   = _HELIX_RADIUS * np.sin(ph)
    zs   = np.where(ts <= _LAP_TIME,
                    HOVER_Z + _VZ_UP * ts,
                    HOVER_Z + _HELIX_DZ - _VZ_UP * (ts - _LAP_TIME))

    vxs  = -_HELIX_RADIUS * _OMEGA_C * np.sin(ph)
    vys  =  _HELIX_RADIUS * _OMEGA_C * np.cos(ph)
    vzs  = np.where(ts <= _LAP_TIME, _VZ_UP, -_VZ_UP)
    speed = np.sqrt(vxs**2 + vys**2 + vzs**2)
    max_spd = speed.max()

    folder = os.path.join(OUT_DIR, "helix")
    os.makedirs(folder, exist_ok=True)

    fig = plt.figure(figsize=(16, 7))
    fig.suptitle(
        f"Helix Path  —  r={_HELIX_RADIUS:.2f}m  dz={_HELIX_DZ:.2f}m  "
        f"lap={_LAP_TIME:.1f}s  ×2  |  max speed={max_spd:.2f} m/s",
        fontsize=13, fontweight='bold',
    )

    # ── 3D helix ─────────────────────────────────────────────────────────────
    ax3 = fig.add_subplot(1, 2, 1, projection='3d')
    for i in range(N - 1):
        colour = plt.cm.plasma(speed[i] / max(max_spd, 1e-6))
        ax3.plot(xs[i:i+2], ys[i:i+2], zs[i:i+2], color=colour, lw=2.0)
    ax3.scatter([xs[0]],  [ys[0]],  [zs[0]],  color='green', s=80, zorder=5, label='start')
    ax3.scatter([xs[-1]], [ys[-1]], [zs[-1]], color='red',   s=70, marker='s', zorder=5, label='end')
    # Mark apex (top of helix)
    apex_idx = np.argmax(zs)
    ax3.scatter([xs[apex_idx]], [ys[apex_idx]], [zs[apex_idx]],
                color='gold', s=90, marker='^', zorder=5, label=f'apex ({zs[apex_idx]:.2f}m)')
    ax3.set_xlabel('x [m]'); ax3.set_ylabel('y [m]'); ax3.set_zlabel('z [m]')
    ax3.set_title('3D Helix  (colour = speed, blue→yellow)', fontsize=11)
    ax3.view_init(elev=25, azim=-50)
    ax3.legend(fontsize=8)

    # ── 2D profiles ──────────────────────────────────────────────────────────
    ax2 = fig.add_subplot(1, 2, 2)
    ax2b = ax2.twinx()

    l1, = ax2.plot(ts, zs,    color='tab:green',  lw=2,   label='z [m]')
    l2, = ax2b.plot(ts, speed, color='tab:blue',   lw=1.5, label='speed [m/s]', ls='--')
    ax2.axhline(HOVER_Z,              color='gray', lw=0.8, ls=':', alpha=0.5, label=f'hover z={HOVER_Z:.1f}m')
    ax2.axhline(HOVER_Z + _HELIX_DZ, color='gold', lw=0.8, ls=':', alpha=0.7, label=f'apex z={HOVER_Z+_HELIX_DZ:.1f}m')
    ax2.axvline(_LAP_TIME, color='gray', lw=1, ls='--', alpha=0.5, label='lap 1/2 boundary')

    ax2.set_xlabel('time [s]', fontsize=10)
    ax2.set_ylabel('height z [m]', fontsize=10, color='tab:green')
    ax2b.set_ylabel('speed [m/s]',  fontsize=10, color='tab:blue')
    ax2.set_title(f'Height & Speed Profile\n(speed = const {max_spd:.2f} m/s in XY, +vz during ascent)',
                  fontsize=9)
    lines = [l1, l2]
    ax2.legend(lines, [l.get_label() for l in lines], fontsize=8)
    ax2.grid(True, alpha=0.3)

    _save(fig, folder, "helix_path.png")
    return fig


def plot_helix_dynamics():
    """7-panel dynamics figure for helix — mirrors plot_dynamics() for circle/figure-8."""
    _G    = 9.81
    _MASS = 0.031
    N     = 800
    ts    = np.linspace(0, _T_HELIX, N)
    ph    = np.pi + _OMEGA_C * ts

    # Position
    xs  = _HELIX_RADIUS * np.cos(ph)
    ys  = _HELIX_RADIUS * np.sin(ph)
    zs  = np.where(ts <= _LAP_TIME,
                   HOVER_Z + _VZ_UP * ts,
                   HOVER_Z + _HELIX_DZ - _VZ_UP * (ts - _LAP_TIME))

    # Velocity
    vxs = -_HELIX_RADIUS * _OMEGA_C * np.sin(ph)
    vys =  _HELIX_RADIUS * _OMEGA_C * np.cos(ph)
    vzs = np.where(ts <= _LAP_TIME, _VZ_UP, -_VZ_UP)
    spd = np.sqrt(vxs**2 + vys**2 + vzs**2)

    # Acceleration (centripetal only; az_traj=0)
    axs = -_HELIX_RADIUS * _OMEGA_C**2 * np.cos(ph)
    ays = -_HELIX_RADIUS * _OMEGA_C**2 * np.sin(ph)
    azs = np.zeros_like(ts)
    acc_h = np.sqrt(axs**2 + ays**2)

    # Jerk (az jerk: delta-function at lap boundary, ignore for vis)
    jxs =  _HELIX_RADIUS * _OMEGA_C**3 * np.sin(ph)
    jys = -_HELIX_RADIUS * _OMEGA_C**3 * np.cos(ph)
    jzs = np.zeros_like(ts)
    jerk_h = np.sqrt(jxs**2 + jys**2)

    # Tilt and thrust via flatness
    fxs = axs; fys = ays; fzs = np.full_like(ts, _G)  # f = acc + g*ez
    fn  = np.sqrt(fxs**2 + fys**2 + fzs**2)
    tilt_deg = np.degrees(np.arctan2(np.sqrt(axs**2 + ays**2), _G))
    thrust_N = _MASS * fn

    # Body-frame angular velocity (flatness, yaw=0)
    # yc = [0,1,0]; xb_raw = yc × f = [fz, 0, -fx]
    xbxr = fzs; xbyr = np.zeros_like(ts); xbzr = -fxs
    xbn  = np.sqrt(xbxr**2 + xbzr**2)
    xBx  = xbxr / xbn; xBy  = xbyr / xbn; xBz  = xbzr / xbn
    # zB = f / fn
    zBx = fxs / fn; zBy = fys / fn; zBz = fzs / fn
    # yB = zB × xB
    yBx = zBy*xBz - zBz*xBy
    yBy = zBz*xBx - zBx*xBz
    yBz = zBx*xBy - zBy*xBx
    # ω
    wx_dps = np.degrees((yBx*jxs + yBy*jys) / fn)
    wy_dps = np.degrees(-(xBx*jxs + xBy*jys) / fn)
    wz_dps = np.zeros_like(ts)
    max_w  = max(np.abs(wx_dps).max(), np.abs(wy_dps).max())

    feasible  = bool(thrust_N.max() <= CF_MAX_THRUST_N)
    margin_pc = 100.0 * (1.0 - thrust_N.max() / CF_MAX_THRUST_N)

    folder = os.path.join(OUT_DIR, "helix")
    os.makedirs(folder, exist_ok=True)

    fig = plt.figure(figsize=(18, 14))
    fig.suptitle(
        f"Helix  —  Dynamics  |  r={_HELIX_RADIUS:.2f}m  dz={_HELIX_DZ:.2f}m  "
        f"lap={_LAP_TIME:.1f}s ×2  |  thrust {'OK' if feasible else 'OVER LIMIT'}",
        fontsize=13, fontweight='bold',
        color='black' if feasible else 'red',
    )
    gs = fig.add_gridspec(3, 3, hspace=0.48, wspace=0.42, height_ratios=[1, 1, 0.85])

    # Row 0, Panel 0: Overview
    ax = fig.add_subplot(gs[0, 0])
    axb = ax.twinx()
    l1, = ax.plot(ts, spd,      color='tab:blue',   lw=1.4, label='speed [m/s]')
    l2, = ax.plot(ts, tilt_deg, color='tab:orange', lw=1.2, ls='--', label='tilt [°]')
    l3, = axb.plot(ts, acc_h,   color='tab:red',    lw=1.0, ls=':',  label='|a_xy| [m/s²]')
    ax.axvline(_LAP_TIME, color='gray', lw=0.8, ls='--', alpha=0.5)
    ax.set_xlabel('time [s]'); ax.set_ylabel('speed [m/s] / tilt [°]')
    axb.set_ylabel('|a_xy| [m/s²]', color='tab:red')
    ax.set_title(f'Overview  (max tilt={tilt_deg.max():.2f}°  |a_xy|={acc_h.max():.3f} m/s²)')
    ax.legend([l1,l2,l3], [l.get_label() for l in [l1,l2,l3]], fontsize=7)
    ax.grid(True, alpha=0.3)

    # Row 0, Panel 1: Position
    ax = fig.add_subplot(gs[0, 1])
    ax.plot(ts, xs, color='tab:blue',   lw=1.3, label='x')
    ax.plot(ts, ys, color='tab:orange', lw=1.3, label='y')
    ax.plot(ts, zs, color='tab:green',  lw=1.3, label='z')
    ax.axvline(_LAP_TIME, color='gray', lw=0.8, ls='--', alpha=0.5)
    ax.set_xlabel('time [s]'); ax.set_ylabel('position [m]')
    ax.set_title('Position vs Time')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # Row 0, Panel 2: Velocity
    ax = fig.add_subplot(gs[0, 2])
    ax.plot(ts, vxs, color='tab:blue',   lw=1.3, label='vx')
    ax.plot(ts, vys, color='tab:orange', lw=1.3, label='vy')
    ax.plot(ts, vzs, color='tab:green',  lw=1.3, label='vz')
    ax.plot(ts, spd, color='k',          lw=1.0, ls='--', label='|v|')
    ax.axvline(_LAP_TIME, color='gray', lw=0.8, ls='--', alpha=0.5)
    ax.set_xlabel('time [s]'); ax.set_ylabel('velocity [m/s]')
    ax.set_title('Velocity vs Time')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # Row 1, Panel 0: Acceleration
    ax = fig.add_subplot(gs[1, 0])
    ax.plot(ts, axs,  color='tab:blue',   lw=1.3, label='ax')
    ax.plot(ts, ays,  color='tab:orange', lw=1.3, label='ay')
    ax.plot(ts, azs,  color='tab:green',  lw=1.3, label='az')
    ax.plot(ts, acc_h, color='k',         lw=1.0, ls='--', label='|a_xy|')
    ax.axvline(_LAP_TIME, color='gray', lw=0.8, ls='--', alpha=0.5)
    ax.set_xlabel('time [s]'); ax.set_ylabel('acceleration [m/s²]')
    ax.set_title('Acceleration vs Time  (centripetal only, az=0)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # Row 1, Panel 1: Jerk
    ax = fig.add_subplot(gs[1, 1])
    ax.plot(ts, jxs,   color='tab:blue',   lw=1.3, label='jx')
    ax.plot(ts, jys,   color='tab:orange', lw=1.3, label='jy')
    ax.plot(ts, jzs,   color='tab:green',  lw=1.3, label='jz')
    ax.plot(ts, jerk_h, color='k',         lw=1.0, ls='--', label='|j_xy|')
    ax.axvline(_LAP_TIME, color='gray', lw=0.8, ls='--', alpha=0.5)
    ax.set_xlabel('time [s]'); ax.set_ylabel('jerk [m/s³]')
    ax.set_title('Jerk vs Time  (analytic — constant magnitude for constant ω)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # Row 1, Panel 2: Thrust feasibility
    ax = fig.add_subplot(gs[1, 2])
    ax.plot(ts, thrust_N, color='tab:blue', lw=1.4, label='required thrust')
    ax.axhline(CF_MAX_THRUST_N, color='red',  lw=1.2, ls='--',
               label=f'max ({CF_MAX_THRUST_N:.2f} N)')
    ax.axhline(CF_HOVER_N,      color='gray', lw=1.0, ls=':',
               label=f'hover ({CF_HOVER_N:.3f} N)')
    ax.fill_between(ts, thrust_N, CF_MAX_THRUST_N,
                    where=thrust_N > CF_MAX_THRUST_N,
                    color='red', alpha=0.25, label='over limit')
    ax.axvline(_LAP_TIME, color='gray', lw=0.8, ls='--', alpha=0.5)
    ax.set_xlabel('time [s]'); ax.set_ylabel('thrust [N]')
    ax.set_title(f'Thrust Feasibility  (margin={margin_pc:.1f}%)',
                 color='black' if feasible else 'red')
    ax.set_ylim(bottom=0)
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # Row 2 wide: Angular velocity
    ax = fig.add_subplot(gs[2, :])
    ax.plot(ts, wx_dps, color='tab:blue',   lw=1.3, label='ωx  roll rate')
    ax.plot(ts, wy_dps, color='tab:orange', lw=1.3, label='ωy  pitch rate')
    ax.plot(ts, wz_dps, color='tab:green',  lw=1.3, label='ωz  yaw rate')
    ax.axhline(0, color='k', lw=0.6, alpha=0.4)
    ax.axvline(_LAP_TIME, color='gray', lw=0.8, ls='--', alpha=0.5, label='lap 1/2 boundary')
    ax.set_xlabel('time [s]', fontsize=10)
    ax.set_ylabel('angular velocity [°/s]', fontsize=10)
    ax.set_title(
        f'Body-Frame Angular Velocity  (from differential flatness)  '
        f'max |ω| = {max_w:.1f} °/s',
        fontsize=10,
    )
    ax.legend(fontsize=9, ncol=4, loc='upper right')
    ax.grid(True, alpha=0.3)

    _save(fig, folder, "helix_dynamics.png")
    return fig


# Add new trajectories here: (data, display_name, speed_scale, folder_slug)
TRAJECTORIES = [
    (circle_poly4d,      "Circle (r=0.30m)",  1.0, "circle"),
    (circle_poly4d,      "Fast Circle (2×)",  2.0, "fast_circle"),
    (our_figure8_poly4d, "Figure-8",          1.0, "figure8"),
    (fast_figure8_poly4d, "Fast Figure-8 (2×)", 1.0, "fast_figure8"),
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

# Flip summary row (angle-only trajectory — no XY/Z dynamics)
omega_max_dps = np.degrees(max(abs(_flip_eval(_FLIP_SEGS, t, deriv=1))
                               for t in np.linspace(0, _T_FLIP, 500)))
print(f"  {'Flip (360° pitch)':<22}  {_T_FLIP:>8.2f}s  {'N/A':>7}  "
      f"  {'360.0':>8}°  {'N/A':>8} {'OK':<4}  {omega_max_dps:>8.0f}°/s  {'N/A':>8}")

figs.append(plot_flip_path())
figs.append(plot_flip_preview())

# Fast Flip
omega_fast_flip_dps = np.degrees(max(abs(_flip_eval(_FAST_FLIP_SEGS, t, deriv=1))
                                     for t in np.linspace(0, _T_FAST_FLIP, 500)))
print(f"  {'Fast Flip (2×)':<22}  {_T_FAST_FLIP:>8.2f}s  {'N/A':>7}  "
      f"  {'360.0':>8}°  {'N/A':>8} {'OK':<4}  {omega_fast_flip_dps:>8.0f}°/s  {'N/A':>8}")
figs.append(plot_fast_flip_path())
figs.append(plot_fast_flip_preview())

# Roll
omega_roll_dps = np.degrees(max(abs(_flip_eval(_ROLL_SEGS, t, deriv=1))
                                for t in np.linspace(0, _T_ROLL, 500)))
print(f"  {'Roll (360° right)':<22}  {_T_ROLL:>8.2f}s  {'N/A':>7}  "
      f"  {'360.0':>8}°  {'N/A':>8} {'OK':<4}  {omega_roll_dps:>8.0f}°/s  {'N/A':>8}")
figs.append(plot_roll_path())
figs.append(plot_roll_preview())

# Fast Roll
omega_fast_roll_dps = np.degrees(max(abs(_flip_eval(_FAST_ROLL_SEGS, t, deriv=1))
                                     for t in np.linspace(0, _T_FAST_ROLL, 500)))
print(f"  {'Fast Roll (2×)':<22}  {_T_FAST_ROLL:>8.2f}s  {'N/A':>7}  "
      f"  {'360.0':>8}°  {'N/A':>8} {'OK':<4}  {omega_fast_roll_dps:>8.0f}°/s  {'N/A':>8}")
figs.append(plot_fast_roll_path())
figs.append(plot_fast_roll_preview())

# Helix
helix_speed  = np.sqrt((_HELIX_RADIUS * _OMEGA_C)**2 + _VZ_UP**2)
helix_tilt   = np.degrees(np.arctan(_HELIX_RADIUS * _OMEGA_C**2 / 9.81))
helix_thrust = 0.031 * np.sqrt((_HELIX_RADIUS * _OMEGA_C**2)**2 + 9.81**2)
helix_om     = 0.031 * (_HELIX_RADIUS * _OMEGA_C**3) / (0.031 * 9.81)   # ~wy peak [rad/s]
helix_om_dps = np.degrees(helix_om)
print(f"  {'Helix (2 laps)':<22}  {_T_HELIX:>8.2f}s  {helix_speed:>7.2f}m/s"
      f"  {helix_tilt:>8.2f}°  {helix_thrust:>8.3f}N {'OK':<4}"
      f"  {helix_om_dps:>8.1f}°/s  {'~0mm':>8}")
figs.append(plot_helix_path())
figs.append(plot_helix_dynamics())

print()
plt.show()
