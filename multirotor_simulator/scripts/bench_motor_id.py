#!/usr/bin/env python3
"""
Simple bench motor identification script.
Usage:
  python3 scripts/bench_motor_id.py path/to/step_response.csv --dt 0.005 --plot

CSV input: header or no header. Must contain 3 columns (in order):
  time_s, cmd (unitless PWM or commanded thrust), measured_thrust_N

Output: prints fitted time constant tau (s), steady-state gain K (N/command),
and recommended thrust-rate-limit R (N/s) with a safety margin. Optionally
writes a JSON summary with --out.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

try:
    from scipy.optimize import curve_fit

    SCIPY_AVAILABLE = True
except Exception:
    SCIPY_AVAILABLE = False


def load_csv(path):
    # Try numpy genfromtxt; skip non-numeric header lines
    data = np.genfromtxt(path, delimiter=",", comments="#")
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 3:
        raise ValueError(
            "CSV must have at least 3 columns: time_s, cmd, measured_thrust_N"
        )
    t = data[:, 0].astype(float)
    cmd = data[:, 1].astype(float)
    y = data[:, 2].astype(float)
    return t, cmd, y


def find_step_region(cmd, t):
    # Find first index where cmd changes significantly (relative to median noise)
    dif = np.abs(np.diff(cmd))
    if dif.size == 0:
        return None
    thr = max(1e-6, np.median(dif) * 5.0)
    idx = np.where(dif > thr)[0]
    if idx.size == 0:
        # fallback: use earliest change above tiny threshold
        idx = np.where(dif > 1e-4)[0]
        if idx.size == 0:
            return None
    k = idx[0]
    t0 = t[k + 1]
    return t0, k + 1


def step_model(t, y_inf, y0, t0, tau):
    # y(t) = y_inf + (y0 - y_inf)*exp(-(t - t0)/tau) for t>=t0
    y = np.array(t)
    y_out = np.empty_like(y)
    after = y >= t0
    y_out[~after] = y0
    y_out[after] = y_inf + (y0 - y_inf) * np.exp(-(y[after] - t0) / tau)
    return y_out


def fit_step(t, y, t0_guess=None):
    # Use curve_fit if available, otherwise linearize
    # Estimate y0 (pre-step mean) and y_inf (post-step mean)
    # Choose windows: 5% of data before/after
    n = len(t)
    w = max(1, int(n * 0.05))
    y0 = np.mean(y[:w])
    y_inf = np.mean(y[-w:])
    if t0_guess is None:
        # approximate step time: where y leaves neighborhood of y0
        dif = np.abs(y - y0)
        thr = max(1e-6, np.std(y[:w]) * 4.0)
        idx = np.where(dif > thr)[0]
        if idx.size == 0:
            t0 = t[0]
        else:
            t0 = t[idx[0]]
    else:
        t0 = t0_guess

    if SCIPY_AVAILABLE:
        # Fit y_inf, tau keeping y0 fixed for robustness
        def model_fit(tt, y_inf_fit, tau_fit):
            return step_model(tt, y_inf_fit, y0, t0, np.abs(tau_fit))

        try:
            popt, pcov = curve_fit(
                model_fit, t, y, p0=[y_inf, (t[-1] - t0) / 3], maxfev=20000
            )
            y_inf_fit, tau_fit = popt
            tau_fit = float(abs(tau_fit))
            y0_fit = float(y0)
            return {
                "y0": y0_fit,
                "y_inf": float(y_inf_fit),
                "t0": float(t0),
                "tau": float(tau_fit),
            }
        except Exception:
            pass
    # Fallback linearization: for t>t0, ln(1 - (y - y_inf)/(y0 - y_inf)) = -(t - t0)/tau
    mask = t > t0
    if np.sum(mask) < 3:
        # not enough points
        return {
            "y0": float(y0),
            "y_inf": float(y_inf),
            "t0": float(t0),
            "tau": float(max(1e-3, (t[-1] - t[0]) / 3)),
        }
    y_mask = y[mask]
    t_mask = t[mask]
    # estimate y_inf as last window mean
    y_inf_est = np.mean(y[-w:])
    denom = y0 - y_inf_est
    # protect against divide by zero
    if abs(denom) < 1e-9:
        tau_est = (t[-1] - t0) / 3.0
        return {
            "y0": float(y0),
            "y_inf": float(y_inf_est),
            "t0": float(t0),
            "tau": float(tau_est),
        }
    val = 1.0 - (y_mask - y_inf_est) / denom
    # clamp values inside (0,1) to avoid log problems
    eps = 1e-9
    val = np.clip(val, eps, 1.0 - eps)
    ln = np.log(val)
    # linear fit ln = -(t - t0)/tau => slope = -1/tau
    A = np.vstack([-(t_mask - t0), np.ones_like(t_mask)]).T
    slope, _ = np.linalg.lstsq(A, ln, rcond=None)[0]
    if slope == 0:
        tau_est = (t[-1] - t0) / 3.0
    else:
        tau_est = 1.0 / slope
    return {
        "y0": float(y0),
        "y_inf": float(y_inf_est),
        "t0": float(t0),
        "tau": float(abs(tau_est)),
    }


def recommend_R(t, y, tau, safety=1.2):
    # Compute empirical max slope of measured thrust (N/s)
    dt = np.diff(t)
    dy = np.diff(y)
    slopes = dy / np.where(dt == 0, 1e-9, dt)
    max_slope = float(np.max(np.abs(slopes)))
    if not np.isfinite(max_slope) or max_slope <= 0:
        # fallback: approximate from delta over tau
        delta = np.abs(y[-1] - y[0])
        max_slope = delta / max(1e-6, tau)
    R = safety * max_slope
    return R, max_slope


def main():
    p = argparse.ArgumentParser(
        description="Bench motor identification: fit first-order time constant from step response CSV"
    )
    p.add_argument("csv", help="CSV file: time_s, cmd, measured_thrust_N")
    p.add_argument(
        "--dt",
        type=float,
        default=0.005,
        help="controller dt (s) to report per-step limits",
    )
    p.add_argument("--out", help="optional JSON output file to write summary")
    p.add_argument("--plot", action="store_true", help="show diagnostic plot")
    p.add_argument(
        "--safety", type=float, default=1.2, help="safety factor for recommended R"
    )
    args = p.parse_args()

    if not os.path.exists(args.csv):
        print("CSV not found:", args.csv)
        sys.exit(1)
    t, cmd, y = load_csv(args.csv)
    # normalize time to start at zero
    t = t - t[0]
    step_info = find_step_region(cmd, t)
    if step_info is None:
        print(
            "Could not detect step in command series. Make sure the CSV contains a clear step change in column 2 (cmd)."
        )
        # still try to fit
        t0_guess = t[0]
    else:
        t0_guess = step_info[0]

    fit = fit_step(t, y, t0_guess)
    tau = float(fit["tau"])
    y0 = float(fit["y0"])
    y_inf = float(fit["y_inf"])
    t0 = float(fit["t0"])

    # estimate gain K relative to command step magnitude
    # find command step magnitude by median before/after
    n = len(t)
    w = max(1, int(n * 0.05))
    cmd0 = np.mean(cmd[:w])
    cmd1 = np.mean(cmd[-w:])
    delta_cmd = cmd1 - cmd0
    delta_y = y_inf - y0
    if abs(delta_cmd) < 1e-9:
        K = float("nan")
    else:
        K = float(delta_y / delta_cmd)

    R, max_slope = recommend_R(t, y, tau, safety=args.safety)
    per_dt = R * args.dt

    summary = {
        "file": os.path.abspath(args.csv),
        "tau_s": float(tau),
        "y0_N": float(y0),
        "y_inf_N": float(y_inf),
        "t0_s": float(t0),
        "gain_K_N_per_cmd": float(K) if np.isfinite(K) else None,
        "empirical_max_slope_N_per_s": float(max_slope),
        "recommended_R_N_per_s": float(R),
        "recommended_delta_per_dt_N": float(per_dt),
        "controller_dt_s": float(args.dt),
    }

    print("\nBench motor ID results:")
    print(" CSV:           ", summary["file"])
    print(" tau (s):       ", f"{summary['tau_s']:.4f}")
    print(" y0 (N):        ", f"{summary['y0_N']:.4f}")
    print(" y_inf (N):     ", f"{summary['y_inf_N']:.4f}")
    print(
        " gain K (N/cmd):",
        (
            f"{summary['gain_K_N_per_cmd']:.4f}"
            if summary["gain_K_N_per_cmd"] is not None
            else "N/A"
        ),
    )
    print(" max slope (N/s):", f"{summary['empirical_max_slope_N_per_s']:.3f}")
    print(" recommended R (N/s):", f"{summary['recommended_R_N_per_s']:.3f}")
    print(
        " recommended delta per dt:",
        f"{summary['recommended_delta_per_dt_N']:.4f} N (for dt={args.dt}s)",
    )

    if args.out:
        with open(args.out, "w") as f:
            json.dump(summary, f, indent=2)
        print("Wrote summary to", args.out)

    if args.plot:
        try:
            import matplotlib.pyplot as plt

            plt.figure(figsize=(8, 4))
            plt.plot(t, y, label="measured_thrust_N")
            plt.plot(t, cmd, label="cmd", alpha=0.6)
            tt = np.linspace(t[0], t[-1], 500)
            yy = step_model(tt, y_inf, y0, t0, tau)
            plt.plot(tt, yy, "--", label=f"fit tau={tau:.3f}s")
            plt.axvline(t0, color="k", lw=0.5)
            plt.xlabel("time (s)")
            plt.legend()
            plt.title("Bench motor ID: step response")
            plt.grid(True)
            plt.show()
        except Exception as e:
            print("Plotting failed (matplotlib may be missing):", e)


if __name__ == "__main__":
    main()
