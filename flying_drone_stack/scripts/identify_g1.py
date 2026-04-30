#!/usr/bin/env python3
"""Offline G1 identification from flight CSV logs.

G1 (control effectiveness) maps torque increment → angular acceleration:
    α[k] = G1 · δu[k]

The CSV does not log raw motor torques, so we use two proxy signals:
  - α [rad/s²]  — differentiated gyro (columns gyro_x/y/z, logged in deg/s)
  - δu [units]  — differentiated torque-proxy column (see --proxy flag)

Proxy options:
  "roll_cmd"     — our_roll_cmd  [deg]  (attitude RPYT command, Mode A shadow)
  "pitch_cmd"    — our_pitch_cmd [deg]  (attitude RPYT command, Mode A shadow)
  "thrust"       — our_thrust    [N]    (total thrust, useful for thrust-stand data)

Because our_roll/pitch_cmd are DEGREES, G1 comes out in [rad/s²/deg].
Multiply by (180/π) to get [rad/s²/rad].  For INDI (which needs [rad/s²/Nm])
the user must additionally divide by the firmware attitude-PID gain KR.

Usage:
  python scripts/identify_g1.py runs/<file>.csv
  python scripts/identify_g1.py runs/<file>.csv --proxy roll_cmd --min_du 0.05 --plot

The script filters out rows where |δu| < min_du to avoid noise-dominated windows
and rows where thrust < min_thrust (on the ground / landing transient).
"""

import sys
import argparse
import numpy as np

# ── known column index for CSVs written by main.rs ─────────────────────────
# New format (49 cols):
# time_ms=0  pos_x=1 ... gyro_x=12 gyro_y=13 gyro_z=14 ... thrust=10 ...
# our_ref_x=30 our_ref_y=31 our_ref_z=32
# our_thrust=33 our_roll_cmd=34 our_pitch_cmd=35 our_yaw_rate_cmd=36
GYRO_COLS   = {"x": "gyro_x",   "y": "gyro_y",   "z": "gyro_z"}
PROXY_COLS  = {
    "roll_cmd":  "our_roll_cmd",
    "pitch_cmd": "our_pitch_cmd",
    "yaw_cmd":   "our_yaw_rate_cmd",
    "thrust":    "our_thrust",
}
TIME_COL    = "time_ms"
THRUST_COL  = "our_thrust"

# CF 2.1 inertia [kg·m²] — for reference / cross-check against J-based G1 = 1/J
J_XX = 16.571710e-6
J_YY = 16.655602e-6
J_ZZ = 29.261652e-6


def load_csv(path: str) -> dict[str, np.ndarray]:
    """Load CSV, return dict of column_name → numpy array."""
    with open(path) as f:
        header = f.readline().strip().split(",")
    data = np.genfromtxt(path, delimiter=",", skip_header=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return {col: data[:, i] for i, col in enumerate(header)}


def finite_diff(arr: np.ndarray, dt: np.ndarray) -> np.ndarray:
    """Centred finite difference; edges use forward/backward."""
    d = np.empty_like(arr)
    d[0]  = (arr[1]  - arr[0])  / dt[1]
    d[-1] = (arr[-1] - arr[-2]) / dt[-1]
    d[1:-1] = (arr[2:] - arr[:-2]) / (dt[1:-1] + dt[2:])
    return d


def identify_axis(alpha: np.ndarray, delta_u: np.ndarray, min_du: float) -> float:
    """Least-squares G1 = α / δu, windowed to |δu| > min_du.

    Returns G1 in the same units as alpha/delta_u (no unit conversion here).
    """
    mask = np.abs(delta_u) > min_du
    if mask.sum() < 10:
        print(f"  WARNING: only {mask.sum()} samples with |δu| > {min_du} — G1 unreliable")
        mask = np.ones_like(mask, dtype=bool)
    u_fit = delta_u[mask].reshape(-1, 1)
    a_fit = alpha[mask]
    g1, _, _, _ = np.linalg.lstsq(u_fit, a_fit, rcond=None)
    return float(g1[0])


def main():
    parser = argparse.ArgumentParser(description="Offline G1 identification from flight CSV")
    parser.add_argument("csv", help="Path to flight CSV (from main.rs runs/)")
    parser.add_argument("--proxy", choices=list(PROXY_COLS.keys()), default="roll_cmd",
                        help="Torque proxy column (default: roll_cmd)")
    parser.add_argument("--min_du", type=float, default=0.1,
                        help="Minimum |δu| for regression window (default: 0.1 deg)")
    parser.add_argument("--min_thrust", type=float, default=0.05,
                        help="Minimum thrust [N] to include rows (default: 0.05)")
    parser.add_argument("--plot", action="store_true", help="Show scatter plot of α vs δu")
    args = parser.parse_args()

    print(f"Loading {args.csv}")
    cols = load_csv(args.csv)

    required = [TIME_COL, THRUST_COL] + list(GYRO_COLS.values()) + [PROXY_COLS[args.proxy]]
    for c in required:
        if c not in cols:
            print(f"ERROR: column '{c}' not found. Available: {list(cols.keys())}")
            sys.exit(1)

    t_ms    = cols[TIME_COL]
    thrust  = cols[THRUST_COL]

    # ── Filter: in-flight rows only ──────────────────────────────────────────
    in_flight = thrust > args.min_thrust
    if in_flight.sum() < 20:
        print("WARNING: very few in-flight rows — check min_thrust")
    print(f"  Rows total: {len(t_ms)}   in-flight: {in_flight.sum()}")

    # dt array [s] (forward difference for each row)
    dt_ms = np.diff(t_ms, prepend=t_ms[0] - (t_ms[1] - t_ms[0] if len(t_ms) > 1 else 50))
    dt_s  = np.abs(dt_ms) / 1000.0
    dt_s  = np.clip(dt_s, 1e-4, 0.5)  # clip outliers (packet gaps)

    # ── Proxy torque signal and its increment ────────────────────────────────
    u       = cols[PROXY_COLS[args.proxy]]
    delta_u = finite_diff(u, dt_s)

    # ── Gyro angular acceleration (deg/s → rad/s, then differentiate) ────────
    DEG2RAD = np.pi / 180.0
    gx      = cols[GYRO_COLS["x"]] * DEG2RAD
    gy      = cols[GYRO_COLS["y"]] * DEG2RAD
    gz      = cols[GYRO_COLS["z"]] * DEG2RAD

    alpha_x = finite_diff(gx, dt_s)
    alpha_y = finite_diff(gy, dt_s)
    alpha_z = finite_diff(gz, dt_s)

    # ── Restrict to in-flight ────────────────────────────────────────────────
    du_f  = delta_u[in_flight]
    ax_f  = alpha_x[in_flight]
    ay_f  = alpha_y[in_flight]
    az_f  = alpha_z[in_flight]

    # ── Fit G1 per axis ──────────────────────────────────────────────────────
    # For roll_cmd / pitch_cmd proxy, G1 has units [rad/s²/deg].
    # The INDI law needs G1 in [rad/s²/Nm].  For that conversion:
    #   G1_nm = G1_deg × (180/π) / KR_firmware
    # where KR_firmware is the firmware's attitude P-gain in [Nm/rad] (e.g. 0.010).
    # This file only computes G1_deg; leave KR conversion to the user.

    print(f"\n  Proxy: {PROXY_COLS[args.proxy]}  min_du={args.min_du}")
    g1x = identify_axis(ax_f, du_f, args.min_du)
    g1y = identify_axis(ay_f, du_f, args.min_du)
    g1z = identify_axis(az_f, du_f, args.min_du)

    print(f"\n  G1x = {g1x:.4f} rad/s²/{'deg' if 'cmd' in args.proxy else 'unit'}")
    print(f"  G1y = {g1y:.4f} rad/s²/{'deg' if 'cmd' in args.proxy else 'unit'}")
    print(f"  G1z = {g1z:.4f} rad/s²/{'deg' if 'cmd' in args.proxy else 'unit'}")

    # ── Reference: theoretical G1 = 1/J ─────────────────────────────────────
    print("\n  Reference (theoretical G1 = 1/J_axis):")
    print(f"    G1x_theory = 1/J_xx = {1/J_XX:.1f} rad/s²/Nm")
    print(f"    G1y_theory = 1/J_yy = {1/J_YY:.1f} rad/s²/Nm")
    print(f"    G1z_theory = 1/J_zz = {1/J_ZZ:.1f} rad/s²/Nm")

    print("\n  For INDI IndiController::new(...) replace g1 = Vec3::new(1/Jxx, 1/Jyy, 1/Jzz)")
    print("  with measured values after converting to [rad/s²/Nm].")
    print("  Conversion: G1_nm = G1_deg × (180/π) / KR_firmware")
    print("  Using KR_firmware = 0.010 [Nm/rad] (Block N/active):")
    if "cmd" in args.proxy:
        KR = 0.010
        print(f"    G1x_nm = {g1x * (180/np.pi) / KR:.1f}")
        print(f"    G1y_nm = {g1y * (180/np.pi) / KR:.1f}")
        print(f"    G1z_nm = {g1z * (180/np.pi) / KR:.1f}")

    # ── Optional scatter plot ────────────────────────────────────────────────
    if args.plot:
        try:
            import matplotlib.pyplot as plt
            fig, axes = plt.subplots(1, 3, figsize=(14, 4), sharey=False)
            for ax, alpha_vals, g1, label in zip(
                axes,
                [ax_f, ay_f, az_f],
                [g1x, g1y, g1z],
                ["roll (α_x)", "pitch (α_y)", "yaw (α_z)"],
            ):
                ax.scatter(du_f, alpha_vals, s=2, alpha=0.4, label="data")
                u_line = np.linspace(du_f.min(), du_f.max(), 100)
                ax.plot(u_line, g1 * u_line, "r-", lw=2,
                        label=f"G1={g1:.4f}")
                ax.set_xlabel(f"δu [{PROXY_COLS[args.proxy]}]")
                ax.set_ylabel("α [rad/s²]")
                ax.set_title(label)
                ax.legend()
            plt.suptitle(f"G1 identification — {args.csv.split('/')[-1]}")
            plt.tight_layout()
            plt.show()
        except ImportError:
            print("matplotlib not available — skipping plot")

    return 0


if __name__ == "__main__":
    sys.exit(main())
