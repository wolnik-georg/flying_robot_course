#!/usr/bin/env python3
"""
Post-flight trajectory analysis
================================
Loads a CSV from FlightLogger (run_spline_circle.py or run_spline_figure8.py),
evaluates the planned Poly4D trajectory at the same timestamps, and plots:

  Panel 1 — XY path: actual (blue) vs planned (red dashed)
  Panel 2 — Position vs time per axis (actual vs planned)
  Panel 3 — Position error magnitude vs time + RMSE
  Panel 4 — Attitude (roll/pitch/yaw) vs time

Usage:
    python analyze_flight.py --csv logs/circle_20260422_120000.csv --type circle
    python analyze_flight.py --csv logs/figure8_20260422_120000.csv --type figure8
    python analyze_flight.py --csv logs/autonomous_20260422_120000.csv --type autonomous
    python analyze_flight.py          # auto-pick latest CSV, infer type from filename
"""

import argparse
import csv
import glob
import os
import sys

import numpy as np
import matplotlib.pyplot as plt

# ── Trajectory parameters — must match the flight scripts ──────────────────
# Circle (run_spline_circle.py) — 16 segments × 0.654s, tangent yaw
CIRCLE_SPEED_SCALE = 1.0  # update if you change it in the flight script
CIRCLE_XY_SCALE = 1.0
CIRCLE_IS_LOOP = True  # periodic: wraps at end

# Figure-8 our min-snap (run_spline_figure8.py)
FIGURE8_SPEED_SCALE = 1.0  # update if you change it in the flight script
FIGURE8_XY_SCALE = 1.0
FIGURE8_IS_LOOP = False  # point-to-point: clamp at end

# Autonomous — professor's figure-8 (autonomous_sequence_high_level.py)
AUTONOMOUS_SPEED_SCALE = 1.0  # time_scale=1.0 (no scaling)
AUTONOMOUS_XY_SCALE = 1.0  # SCALE=0.7 in the script
AUTONOMOUS_IS_LOOP = False

# ── Poly4D data — copied from the flight scripts ────────────────────────────
# If you regenerate trajectories, update these to match the flight scripts.

circle_poly4d = [
    [
        0.654498,
        -0.000000,
        0.000000,
        0.054000,
        0.000000,
        -0.001621,
        0.000003,
        0.000016,
        0.000001,
        0.000000,
        0.180000,
        -0.000000,
        -0.010800,
        0.000002,
        0.000190,
        0.000006,
        -0.000004,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        1.570796,
        -0.600000,
        0.000000,
        0.000001,
        -0.000022,
        0.000079,
        -0.000101,
        0.000044,
    ],
    [
        0.654498,
        0.022836,
        0.068883,
        0.049889,
        -0.004133,
        -0.001496,
        0.000073,
        0.000019,
        -0.000001,
        0.114805,
        0.166298,
        -0.020665,
        -0.009978,
        0.000621,
        0.000175,
        -0.000002,
        -0.000004,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        1.178097,
        -0.600000,
        -0.000000,
        -0.000000,
        0.000000,
        -0.000001,
        0.000001,
        -0.000001,
    ],
    [
        0.654498,
        0.087868,
        0.127279,
        0.038184,
        -0.007637,
        -0.001146,
        0.000137,
        0.000015,
        -0.000002,
        0.212132,
        0.127279,
        -0.038184,
        -0.007637,
        0.001144,
        0.000142,
        -0.000019,
        0.000001,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.785398,
        -0.600000,
        0.000000,
        0.000000,
        -0.000003,
        0.000011,
        -0.000014,
        0.000006,
    ],
    [
        0.654498,
        0.185195,
        0.166298,
        0.020665,
        -0.009978,
        -0.000626,
        0.000201,
        -0.000020,
        0.000010,
        0.277164,
        0.068883,
        -0.049889,
        -0.004133,
        0.001493,
        0.000088,
        -0.000036,
        0.000007,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.392699,
        -0.600000,
        -0.000000,
        -0.000000,
        0.000002,
        -0.000009,
        0.000012,
        -0.000005,
    ],
    [
        0.654498,
        0.300000,
        0.180000,
        0.000000,
        -0.010800,
        0.000002,
        0.000186,
        0.000011,
        -0.000006,
        0.300000,
        -0.000000,
        -0.054000,
        0.000000,
        0.001626,
        -0.000022,
        0.000008,
        -0.000012,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -0.000000,
        -0.600000,
        -0.000000,
        0.000000,
        -0.000008,
        0.000032,
        -0.000041,
        0.000018,
    ],
    [
        0.654498,
        0.414805,
        0.166298,
        -0.020665,
        -0.009978,
        0.000631,
        0.000139,
        0.000043,
        -0.000024,
        0.277164,
        -0.068883,
        -0.049889,
        0.004133,
        0.001498,
        -0.000079,
        -0.000012,
        -0.000002,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -0.392699,
        -0.600000,
        0.000000,
        -0.000000,
        -0.000001,
        0.000001,
        -0.000001,
        0.000000,
    ],
    [
        0.654498,
        0.512132,
        0.127279,
        -0.038184,
        -0.007637,
        0.001139,
        0.000161,
        -0.000043,
        0.000012,
        0.212132,
        -0.127279,
        -0.038184,
        0.007637,
        0.001146,
        -0.000140,
        -0.000010,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -0.785398,
        -0.600000,
        -0.000001,
        0.000000,
        0.000006,
        -0.000019,
        0.000023,
        -0.000010,
    ],
    [
        0.654498,
        0.577164,
        0.068883,
        -0.049890,
        -0.004133,
        0.001503,
        0.000051,
        0.000011,
        -0.000013,
        0.114805,
        -0.166298,
        -0.020665,
        0.009978,
        0.000620,
        -0.000182,
        -0.000004,
        0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -1.178097,
        -0.600000,
        0.000001,
        0.000000,
        -0.000005,
        0.000015,
        -0.000018,
        0.000008,
    ],
    [
        0.654498,
        0.600000,
        -0.000000,
        -0.054000,
        0.000000,
        0.001620,
        -0.000001,
        -0.000018,
        -0.000001,
        -0.000000,
        -0.180000,
        0.000000,
        0.010800,
        -0.000003,
        -0.000184,
        -0.000013,
        0.000007,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -1.570796,
        -0.600000,
        -0.000000,
        -0.000000,
        0.000013,
        -0.000045,
        0.000057,
        -0.000025,
    ],
    [
        0.654498,
        0.577164,
        -0.068883,
        -0.049890,
        0.004133,
        0.001492,
        -0.000058,
        -0.000039,
        0.000010,
        -0.114805,
        -0.166298,
        0.020665,
        0.009978,
        -0.000618,
        -0.000188,
        0.000019,
        -0.000003,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -1.963495,
        -0.600000,
        -0.000000,
        -0.000000,
        0.000002,
        -0.000008,
        0.000010,
        -0.000004,
    ],
    [
        0.654498,
        0.512132,
        -0.127279,
        -0.038184,
        0.007637,
        0.001142,
        -0.000126,
        -0.000028,
        0.000008,
        -0.212132,
        -0.127279,
        0.038184,
        0.007637,
        -0.001144,
        -0.000142,
        0.000019,
        -0.000001,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -2.356194,
        -0.600000,
        -0.000000,
        0.000000,
        0.000035,
        -0.000127,
        0.000162,
        -0.000071,
    ],
    [
        0.654498,
        0.414805,
        -0.166298,
        -0.020665,
        0.009978,
        0.000620,
        -0.000180,
        -0.000007,
        0.000001,
        -0.277164,
        -0.068883,
        0.049890,
        0.004133,
        -0.001501,
        -0.000058,
        -0.000002,
        0.000009,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -2.748894,
        -0.600000,
        0.000001,
        -0.000000,
        -0.000012,
        0.000040,
        -0.000050,
        0.000022,
    ],
    [
        0.654498,
        0.300000,
        -0.180000,
        -0.000000,
        0.010800,
        0.000001,
        -0.000196,
        0.000002,
        0.000001,
        -0.300000,
        0.000000,
        0.054000,
        0.000000,
        -0.001619,
        -0.000003,
        0.000024,
        -0.000002,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -3.141593,
        -0.600000,
        -0.000001,
        -0.000001,
        0.000015,
        -0.000053,
        0.000068,
        -0.000030,
    ],
    [
        0.654498,
        0.185195,
        -0.166298,
        0.020665,
        0.009978,
        -0.000621,
        -0.000175,
        0.000002,
        0.000004,
        -0.277164,
        0.068883,
        0.049890,
        -0.004133,
        -0.001504,
        0.000099,
        -0.000013,
        0.000013,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -3.534292,
        -0.600001,
        0.000000,
        0.000001,
        0.000012,
        -0.000043,
        0.000053,
        -0.000023,
    ],
    [
        0.654498,
        0.087868,
        -0.127279,
        0.038184,
        0.007637,
        -0.001145,
        -0.000139,
        0.000015,
        0.000000,
        -0.212132,
        0.127279,
        0.038184,
        -0.007637,
        -0.001146,
        0.000139,
        0.000011,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -3.926991,
        -0.600000,
        0.000001,
        -0.000001,
        0.000014,
        -0.000053,
        0.000068,
        -0.000030,
    ],
    [
        0.654498,
        0.022836,
        -0.068883,
        0.049889,
        0.004133,
        -0.001497,
        -0.000075,
        0.000018,
        0.000000,
        -0.114805,
        0.166298,
        0.020665,
        -0.009978,
        -0.000621,
        0.000183,
        0.000004,
        0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -4.319690,
        -0.600000,
        -0.000001,
        -0.000000,
        -0.000031,
        0.000117,
        -0.000151,
        0.000066,
    ],
]

figure8_poly4d = [
    [
        1.050000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.920060,
        -0.422403,
        -0.330819,
        0.184887,
        0.000000,
        0.000000,
        -0.000000,
        0.000000,
        -1.744998,
        1.466438,
        0.107645,
        -0.241908,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        0.710000,
        0.396058,
        0.894217,
        0.119978,
        -0.586456,
        -0.088417,
        0.767407,
        -0.773189,
        0.263120,
        -0.445604,
        -0.612846,
        0.911715,
        1.039294,
        -0.529370,
        -1.547065,
        1.745710,
        -0.571617,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        0.620000,
        0.922409,
        0.425094,
        -0.600250,
        -0.163530,
        -0.012154,
        0.048479,
        0.110009,
        -0.056150,
        -0.291165,
        0.907200,
        0.475613,
        -0.850656,
        0.203289,
        0.237703,
        -0.290271,
        0.089495,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        0.700000,
        0.923174,
        -0.445424,
        -0.681107,
        0.226650,
        0.293565,
        -0.024591,
        -0.190025,
        0.072759,
        0.289869,
        0.761427,
        -0.542422,
        -0.353518,
        0.032877,
        -0.064582,
        0.204906,
        -0.075639,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        0.560000,
        0.405364,
        -0.824251,
        0.146024,
        0.236002,
        -0.316721,
        -0.076047,
        0.161773,
        -0.039131,
        0.450742,
        -0.405717,
        -0.938657,
        0.192123,
        0.406789,
        0.021473,
        -0.156746,
        0.042205,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        0.560000,
        0.000000,
        -0.653540,
        0.006420,
        -0.278385,
        -0.009358,
        0.209809,
        0.006890,
        -0.045898,
        -0.000000,
        -1.022634,
        0.004823,
        0.765421,
        -0.010510,
        -0.227116,
        0.011757,
        0.032847,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        0.700000,
        -0.402804,
        -0.819292,
        -0.153336,
        0.224830,
        0.327403,
        -0.067263,
        -0.177646,
        0.076262,
        -0.449354,
        -0.405209,
        0.927436,
        0.184003,
        -0.387313,
        0.024881,
        0.149373,
        -0.070147,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        0.620000,
        -0.921641,
        -0.451349,
        0.680038,
        0.234193,
        -0.308640,
        0.000361,
        0.139849,
        -0.058210,
        -0.292459,
        0.755025,
        0.550873,
        -0.343333,
        -0.024645,
        -0.124468,
        -0.087936,
        0.085862,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        0.710000,
        -0.923935,
        0.421735,
        0.602634,
        -0.164286,
        -0.084256,
        0.261179,
        -0.536957,
        0.263618,
        0.288570,
        0.912522,
        -0.470936,
        -0.858025,
        -0.016914,
        -0.156197,
        1.086628,
        -0.567817,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        1.053185,
        -0.398611,
        0.895363,
        -0.115272,
        -0.585975,
        -0.724664,
        1.764668,
        -1.017567,
        0.182051,
        0.447039,
        -0.609322,
        -0.918168,
        1.034657,
        2.062702,
        -3.428890,
        1.649625,
        -0.237630,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        -0.000000,
        -0.000000,
        -0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
]

# Professor's figure-8 (autonomous_sequence_high_level.py, SCALE=0.7, time_scale=1.0)
autonomous_poly4d = [
    [
        1.050000,
        0.000000,
        -0.000000,
        0.000000,
        -0.000000,
        0.830443,
        -0.276140,
        -0.384219,
        0.180493,
        -0.000000,
        0.000000,
        -0.000000,
        0.000000,
        -1.356107,
        0.688430,
        0.587426,
        -0.329106,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        0.710000,
        0.396058,
        0.918033,
        0.128965,
        -0.773546,
        0.339704,
        0.034310,
        -0.026417,
        -0.030049,
        -0.445604,
        -0.684403,
        0.888433,
        1.493630,
        -1.361618,
        -0.139316,
        0.158875,
        0.095799,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        0.620000,
        0.922409,
        0.405715,
        -0.582968,
        -0.092188,
        -0.114670,
        0.101046,
        0.075834,
        -0.037926,
        -0.291165,
        0.967514,
        0.421451,
        -1.086348,
        0.545211,
        0.030109,
        -0.050046,
        -0.068177,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        0.700000,
        0.923174,
        -0.431533,
        -0.682975,
        0.177173,
        0.319468,
        -0.043852,
        -0.111269,
        0.023166,
        0.289869,
        0.724722,
        -0.512011,
        -0.209623,
        -0.218710,
        0.108797,
        0.128756,
        -0.055461,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        0.560000,
        0.405364,
        -0.834716,
        0.158939,
        0.288175,
        -0.373738,
        -0.054995,
        0.036090,
        0.078627,
        0.450742,
        -0.385534,
        -0.954089,
        0.128288,
        0.442620,
        0.055630,
        -0.060142,
        -0.076163,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        0.560000,
        0.001062,
        -0.646270,
        -0.012560,
        -0.324065,
        0.125327,
        0.119738,
        0.034567,
        -0.063130,
        0.001593,
        -1.031457,
        0.015159,
        0.820816,
        -0.152665,
        -0.130729,
        -0.045679,
        0.080444,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        0.700000,
        -0.402804,
        -0.820508,
        -0.132914,
        0.236278,
        0.235164,
        -0.053551,
        -0.088687,
        0.031253,
        -0.449354,
        -0.411507,
        0.902946,
        0.185335,
        -0.239125,
        -0.041696,
        0.016857,
        0.016709,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        0.620000,
        -0.921641,
        -0.464596,
        0.661875,
        0.286582,
        -0.228921,
        -0.051987,
        0.004669,
        0.038463,
        -0.292459,
        0.777682,
        0.565788,
        -0.432472,
        -0.060568,
        -0.082048,
        -0.009439,
        0.041158,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        0.710000,
        -0.923935,
        0.447832,
        0.627381,
        -0.259808,
        -0.042325,
        -0.032258,
        0.001420,
        0.005294,
        0.288570,
        0.873350,
        -0.515586,
        -0.730207,
        -0.026023,
        0.288755,
        0.215678,
        -0.148061,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
    [
        1.053185,
        -0.398611,
        0.850510,
        -0.144007,
        -0.485368,
        -0.079781,
        0.176330,
        0.234482,
        -0.153567,
        0.447039,
        -0.532729,
        -0.855023,
        0.878509,
        0.775168,
        -0.391051,
        -0.713519,
        0.391628,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
        0.000000,
    ],
]

TRAJECTORIES = {
    "circle": (circle_poly4d, CIRCLE_SPEED_SCALE, CIRCLE_XY_SCALE, CIRCLE_IS_LOOP),
    "figure8": (figure8_poly4d, FIGURE8_SPEED_SCALE, FIGURE8_XY_SCALE, FIGURE8_IS_LOOP),
    "autonomous": (
        autonomous_poly4d,
        AUTONOMOUS_SPEED_SCALE,
        AUTONOMOUS_XY_SCALE,
        AUTONOMOUS_IS_LOOP,
    ),
}


GRAVITY = 9.81  # m/s²

# ── Trajectory evaluation ───────────────────────────────────────────────────


def _poly_eval_nth(coeffs, t, n):
    """Evaluate nth derivative of polynomial (coeffs[i] = c_i for c_i*t^i) at time t."""
    result = 0.0
    for i, c in enumerate(coeffs):
        if i < n:
            continue
        factor = 1
        for k in range(n):
            factor *= i - k
        result += c * factor * (t ** (i - n) if i > n else 1.0)
    return result


def _find_seg(segs, t_poly):
    """Return (seg, local_t) for segment containing t_poly."""
    for seg in segs:
        if t_poly <= seg[0] + 1e-9:
            return seg, min(t_poly, seg[0])
        t_poly -= seg[0]
    seg = segs[-1]
    return seg, seg[0]


def compute_planned_attitude(segs, t_elapsed, speed_scale, xy_scale, loop=False):
    """
    Compute planned roll/pitch/yaw from differential flatness of Poly4D.

    Physical acceleration = speed_scale² × poly_2nd_derivative.
    Desired thrust vector → rotation matrix (Lee et al.) → ZYX Euler angles.
    Returns (roll_deg, pitch_deg, yaw_deg).
    """
    total_planned = sum(s[0] for s in segs)
    t_poly = t_elapsed * speed_scale
    if loop:
        t_poly = t_poly % total_planned

    seg, t = _find_seg(segs, t_poly)

    # Physical acceleration from 2nd derivative (speed_scale² converts poly-domain to real)
    ax = speed_scale**2 * _poly_eval_nth(seg[1:9], t, 2) * xy_scale
    ay = speed_scale**2 * _poly_eval_nth(seg[9:17], t, 2) * xy_scale
    az = speed_scale**2 * _poly_eval_nth(seg[17:25], t, 2)

    # Desired thrust vector (world frame, gravity-compensated)
    f = np.array([ax, ay, az + GRAVITY])
    f_norm = np.linalg.norm(f)
    if f_norm < 1e-9:
        return 0.0, 0.0, 0.0

    zb = f / f_norm  # desired body z

    # Desired yaw from poly4d (coeffs 25-32; all-zero for our trajectories → yaw_d=0)
    yaw_d = _poly_eval_nth(seg[25:33], t, 0)
    xc = np.array([np.cos(yaw_d), np.sin(yaw_d), 0.0])

    # Lee et al. rotation matrix columns
    zcxc = np.cross(zb, xc)
    zcxc_n = np.linalg.norm(zcxc)
    yb = zcxc / zcxc_n if zcxc_n > 1e-9 else np.array([0.0, 1.0, 0.0])
    xb = np.cross(yb, zb)

    R = np.column_stack([xb, yb, zb])  # R = [xb | yb | zb]

    # ZYX Euler angles
    roll_rad = np.arctan2(R[2, 1], R[2, 2])
    pitch_rad = np.arcsin(-float(np.clip(R[2, 0], -1.0, 1.0)))
    yaw_rad = np.arctan2(R[1, 0], R[0, 0])

    return np.degrees(roll_rad), np.degrees(pitch_rad), np.degrees(yaw_rad)


def _eval_one(segs, t_poly, xy_scale):
    """Evaluate Poly4D segments at polynomial-domain time t_poly."""
    for seg in segs:
        seg_dur = seg[0]
        if t_poly <= seg_dur + 1e-9:
            t = min(t_poly, seg_dur)
            x = sum(c * t**i for i, c in enumerate(seg[1:9])) * xy_scale
            y = sum(c * t**i for i, c in enumerate(seg[9:17])) * xy_scale
            z = sum(c * t**i for i, c in enumerate(seg[17:25]))
            return x, y, z
        t_poly -= seg_dur
    # Past end — clamp to last point
    seg = segs[-1]
    t = seg[0]
    return (
        sum(c * t**i for i, c in enumerate(seg[1:9])) * xy_scale,
        sum(c * t**i for i, c in enumerate(seg[9:17])) * xy_scale,
        sum(c * t**i for i, c in enumerate(seg[17:25])),
    )


def eval_poly4d(segs, t_elapsed, speed_scale, xy_scale, loop=False):
    """
    Evaluate at real elapsed time t_elapsed.
    speed_scale < 1 → slower. time_scale (cflib) = 1/speed_scale.
    Polynomial domain time: t_poly = t_elapsed * speed_scale
    (drone at real t is at polynomial position for t * speed_scale).
    """
    total_planned = sum(s[0] for s in segs)
    t_poly = t_elapsed * speed_scale
    if loop:
        t_poly = t_poly % total_planned
    return _eval_one(segs, t_poly, xy_scale)


# ── CSV loading ─────────────────────────────────────────────────────────────


def load_csv(path):
    rows = {
        c: []
        for c in [
            "time_s",
            "x",
            "y",
            "z",
            "vx",
            "vy",
            "vz",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "thrust",
        ]
    }
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k in rows:
                try:
                    rows[k].append(float(row[k]))
                except (KeyError, ValueError):
                    rows[k].append(float("nan"))
    return {k: np.array(v) for k, v in rows.items()}


# ── Plotting ────────────────────────────────────────────────────────────────


def plot_analysis(data, segs, traj_type, speed_scale, xy_scale, loop, csv_path):
    times = data["time_s"]

    # Evaluate planned trajectory at each logged timestamp
    plan = np.array([eval_poly4d(segs, t, speed_scale, xy_scale, loop) for t in times])

    # Planned attitude from differential flatness
    planned_att = np.array(
        [compute_planned_attitude(segs, t, speed_scale, xy_scale, loop) for t in times]
    )
    p_roll, p_pitch, p_yaw = planned_att[:, 0], planned_att[:, 1], planned_att[:, 2]
    px, py, pz = plan[:, 0], plan[:, 1], plan[:, 2]

    # Errors
    err_3d = np.sqrt(
        (data["x"] - px) ** 2 + (data["y"] - py) ** 2 + (data["z"] - pz) ** 2
    )
    err_xy = np.sqrt((data["x"] - px) ** 2 + (data["y"] - py) ** 2)
    err_z = np.abs(data["z"] - pz)

    # Planned full path for reference overlay
    t_plan_full = np.linspace(0, times[-1], 500)
    plan_full = np.array(
        [eval_poly4d(segs, t, speed_scale, xy_scale, loop) for t in t_plan_full]
    )

    fig, axes = plt.subplots(2, 2, figsize=(13, 10))
    fig.suptitle(
        f"Trajectory Analysis — {traj_type}  "
        f"(SPEED_SCALE={speed_scale}, XY_SCALE={xy_scale})\n"
        f"{os.path.basename(csv_path)}",
        fontsize=12,
    )

    # ── Panel 1: XY path ────────────────────────────────────────────────────
    ax = axes[0, 0]
    ax.plot(plan_full[:, 0], plan_full[:, 1], "r--", lw=1.5, label="Planned", alpha=0.7)
    ax.plot(data["x"], data["y"], "b-", lw=1.5, label="Actual", alpha=0.9)
    ax.plot(data["x"][0], data["y"][0], "go", ms=8, label="Start", zorder=5)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("XY Path")
    ax.legend(fontsize=9)
    ax.grid(True)
    ax.set_aspect("equal")

    # ── Panel 2: Position vs time ────────────────────────────────────────────
    ax = axes[0, 1]
    for axis, actual, planned, col in [
        ("x", data["x"], px, "tab:blue"),
        ("y", data["y"], py, "tab:orange"),
        ("z", data["z"], pz, "tab:green"),
    ]:
        ax.plot(times, actual, color=col, lw=1.5, label=f"{axis} actual")
        ax.plot(
            times, planned, color=col, lw=1.0, ls="--", alpha=0.6, label=f"{axis} plan"
        )
    ax.set_xlabel("time [s]")
    ax.set_ylabel("position [m]")
    ax.set_title("Position vs Time")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True)

    # ── Panel 3: Position error ──────────────────────────────────────────────
    ax = axes[1, 0]
    ax.plot(times, err_3d * 100, "k-", lw=1.5, label="3D error", alpha=0.9)
    ax.plot(times, err_xy * 100, "b--", lw=1.2, label="XY error")
    ax.plot(times, err_z * 100, color="g", lw=1.0, ls=":", label="Z error")
    rmse_3d = np.sqrt(np.nanmean(err_3d**2))
    rmse_xy = np.sqrt(np.nanmean(err_xy**2))
    ax.axhline(
        rmse_3d * 100,
        color="k",
        lw=0.8,
        ls="--",
        alpha=0.5,
        label=f"RMSE 3D={rmse_3d*100:.1f} cm",
    )
    ax.axhline(
        rmse_xy * 100,
        color="b",
        lw=0.8,
        ls="--",
        alpha=0.5,
        label=f"RMSE XY={rmse_xy*100:.1f} cm",
    )
    ax.set_xlabel("time [s]")
    ax.set_ylabel("error [cm]")
    ax.set_title("Position Error")
    ax.legend(fontsize=8)
    ax.grid(True)

    # ── Panel 4: Attitude — actual vs planned (differential flatness) ─────────
    ax = axes[1, 1]
    for att, col, key, pdata in [
        ("roll_deg", "tab:blue", "roll", p_roll),
        ("pitch_deg", "tab:orange", "pitch", p_pitch),
        ("yaw_deg", "tab:red", "yaw", p_yaw),
    ]:
        vals = data[att]
        mask = ~np.isnan(vals)
        ax.plot(times[mask], vals[mask], color=col, lw=1.2, label=f"{key} actual")
        ax.plot(
            times, pdata, color=col, lw=1.0, ls="--", alpha=0.65, label=f"{key} planned"
        )
    ax.set_xlabel("time [s]")
    ax.set_ylabel("angle [deg]")
    ax.set_title("Attitude — actual vs planned (flatness)")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True)

    plt.tight_layout()

    # Print statistics
    n = np.sum(~np.isnan(err_3d))
    print("\n── Tracking Error Statistics ─────────────────────────────────")
    print(f"  Samples  : {n}  ({n / max(len(times), 1) * 100:.0f}%)")
    print(f"  RMSE 3D  : {rmse_3d*100:.1f} cm")
    print(f"  RMSE XY  : {rmse_xy*100:.1f} cm")
    print(f"  RMSE Z   : {np.sqrt(np.nanmean(err_z**2))*100:.1f} cm")
    print(f"  Max 3D   : {np.nanmax(err_3d)*100:.1f} cm")
    print(f"  Max XY   : {np.nanmax(err_xy)*100:.1f} cm")
    print(f"  Duration : {times[-1]:.1f} s")

    out_path = os.path.splitext(csv_path)[0] + "_analysis.png"
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"  Plot     : {out_path}")
    plt.show()


# ── Entry point ─────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(description="Post-flight trajectory analysis")
    parser.add_argument(
        "--csv", default=None, help="Path to flight CSV (default: latest in logs/)"
    )
    parser.add_argument(
        "--type",
        choices=["circle", "figure8", "autonomous"],
        default=None,
        help="Trajectory type (default: inferred from filename)",
    )
    args = parser.parse_args()

    # Auto-detect CSV
    csv_path = args.csv
    if csv_path is None:
        candidates = sorted(glob.glob("logs/*.csv"))
        if not candidates:
            print("No CSV files found in logs/. Run a flight script first.")
            sys.exit(1)
        csv_path = candidates[-1]
        print(f"Auto-selected: {csv_path}")

    # Infer trajectory type from filename
    traj_type = args.type
    if traj_type is None:
        base = os.path.basename(csv_path).lower()
        if "circle" in base:
            traj_type = "circle"
        elif "figure8" in base or "fig8" in base:
            traj_type = "figure8"
        elif "autonomous" in base:
            traj_type = "autonomous"
        else:
            print(
                "Cannot infer trajectory type from filename. Use --type circle|figure8|autonomous"
            )
            sys.exit(1)
        print(f"Inferred trajectory type: {traj_type}")

    segs, speed_scale, xy_scale, loop = TRAJECTORIES[traj_type]
    data = load_csv(csv_path)

    if len(data["time_s"]) == 0:
        print("CSV is empty.")
        sys.exit(1)

    print(
        f'Loaded {len(data["time_s"])} rows, t=[{data["time_s"][0]:.2f}, {data["time_s"][-1]:.2f}] s'
    )
    plot_analysis(data, segs, traj_type, speed_scale, xy_scale, loop, csv_path)


if __name__ == "__main__":
    main()
