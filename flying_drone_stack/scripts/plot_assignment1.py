#!/usr/bin/env python3
"""
Assignment 1 Visualization: Integration Method Comparison

Visualises two scenarios produced by assignment1.rs:

  Scenario 1 — Vertical takeoff (no rotation)
    Shows positional accuracy differences between Euler, RK4, ExpEuler, ExpRK4.
    All methods handle zero-rotation dynamics similarly; RK4 is ~1 cm more
    accurate than Euler after 2 s.

  Scenario 2 — Spinning hover (constant yaw torque)
    The drone spins at increasing angular velocity (0 → 8.5 rad/s in 5 s).
    All methods maintain |q| = 1 by construction (exponential-map or normalise).
    The ATTITUDE ACCURACY differs: Euler/ExpEuler accumulate ~2.4° more yaw
    than RK4/ExpRK4 over the 5-second spin, because they use the end-of-step
    angular velocity for orientation integration rather than the midpoint.
    This demonstrates why higher-order methods matter for fast rotations.

Usage: python plot_assignment1.py
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os


def load_trajectory_data(filename):
    """Load trajectory data from CSV file"""
    filepath = f"results/assignment1/data/{filename}"
    if not os.path.exists(filepath):
        print(f"Warning: {filepath} not found")
        return None

    df = pd.read_csv(filepath)
    return df


def plot_position_comparison():
    """Plot position vs time for all integration methods"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle(
        "Assignment 1: Integration Method Comparison - Vertical Takeoff", fontsize=16
    )

    methods = ["Euler", "RK4", "ExpEuler", "ExpRK4"]
    colors = ["red", "blue", "green", "orange"]

    for method, color in zip(methods, colors):
        filename = f"trajectory_modular_{method}.csv"
        df = load_trajectory_data(filename)

        if df is not None:
            axes[0].plot(
                df["time"],
                df["z"],
                label=f'{method} (z={df["z"].iloc[-1]:.3f}m)',
                color=color,
                linewidth=2,
            )
            axes[1].plot(df["time"], df["vz"], label=method, color=color, linewidth=2)
            axes[2].plot(
                df["time"],
                df["z"] - df["z"].iloc[0],
                label=method,
                color=color,
                linewidth=2,
            )

    axes[0].set_ylabel("Height (m)")
    axes[0].set_title("Position (Z-axis)")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel("Velocity (m/s)")
    axes[1].set_title("Velocity (Z-axis)")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("Displacement (m)")
    axes[2].set_title("Displacement from Initial Position")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_accuracy_analysis():
    """Plot accuracy differences between integration methods"""
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle("Assignment 1: Integration Accuracy Analysis", fontsize=16)

    # Load RK4 as reference (most accurate)
    rk4_df = load_trajectory_data("trajectory_modular_RK4.csv")
    if rk4_df is None:
        return None

    methods = ["Euler", "ExpEuler", "ExpRK4"]
    colors = ["red", "green", "orange"]

    for method, color in zip(methods, colors):
        filename = f"trajectory_modular_{method}.csv"
        df = load_trajectory_data(filename)

        if df is not None:
            # Position error relative to RK4
            pos_error = df["z"] - rk4_df["z"]
            vel_error = df["vz"] - rk4_df["vz"]

            axes[0].plot(
                df["time"],
                pos_error,
                label=f"{method} vs RK4",
                color=color,
                linewidth=2,
            )
            axes[1].plot(
                df["time"],
                vel_error,
                label=f"{method} vs RK4",
                color=color,
                linewidth=2,
            )

    axes[0].set_ylabel("Position Error (m)")
    axes[0].set_title("Position Error Relative to RK4")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Velocity Error (m/s)")
    axes[1].set_title("Velocity Error Relative to RK4")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def print_statistics():
    """Print final statistics for all methods (vertical takeoff scenario)."""
    print("\n" + "=" * 60)
    print("ASSIGNMENT 1 STATISTICS: Integration Method Comparison")
    print("=" * 60)
    print("Scenario 1 — Vertical takeoff (dt=0.01 s, 2 s simulation)")
    print()

    methods = ["Euler", "RK4", "ExpEuler", "ExpRK4"]

    # Load RK4 as reference
    rk4_df = load_trajectory_data("trajectory_modular_RK4.csv")

    print(f"  {'Method':<12} {'Final Z (m)':<14} {'Final Vz (m/s)':<16} {'ΔZ vs RK4 (mm)'}")
    print("  " + "-" * 58)

    for method in methods:
        filename = f"trajectory_modular_{method}.csv"
        df = load_trajectory_data(filename)

        if df is not None:
            final_pos = df["z"].iloc[-1]
            final_vel = df["vz"].iloc[-1]
            delta_z_mm = (final_pos - rk4_df["z"].iloc[-1]) * 1000 if rk4_df is not None else 0.0
            marker = "(ref)" if method == "RK4" else f"{delta_z_mm:+.2f} mm"
            print(f"  {method:<12} {final_pos:<14.4f} {final_vel:<16.4f} {marker}")

    print()
    print("  Runtime (from 'cargo run --bin assignment1' benchmark):")
    print("    Euler    ~31 ns/step   (1× baseline)")
    print("    RK4     ~111 ns/step   (~3.6× — 4 derivative evals)")
    print("    ExpEuler ~37 ns/step   (~1.2× — exp map replaces linear quat update)")
    print("    ExpRK4  ~158 ns/step   (~5.1× — 4 evals + exp maps)")
    print()
    print("  Conclusion: Euler/ExpEuler are ~9.7 mm higher than RK4 after 2 s")
    print("  (1st-order error; ExpRK4 matches RK4 exactly). With zero rotation")
    print("  the difference is small. See Scenario 2 (spinning hover) for the")
    print("  more significant attitude accuracy difference (~2.4° over 5 s).")


def quat_to_yaw_deg(qw, qx, qy, qz):
    """Convert quaternion to yaw angle in degrees."""
    return np.degrees(np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2)))


def plot_spin_comparison():
    """Scenario 2: spinning hover — attitude accuracy and quaternion norm."""
    methods = ["Euler", "RK4", "ExpEuler", "ExpRK4"]
    colors  = ["red", "blue", "green", "orange"]
    linestyles = ["-", "--", ":", "-."]

    spin_data = {}
    for m in methods:
        path = f"results/assignment1/data/trajectory_modular_{m}_spin.csv"
        if os.path.exists(path):
            spin_data[m] = pd.read_csv(path)

    if not spin_data:
        print("  No spin CSV files found — skipping spin comparison plot.")
        return None

    fig, axes = plt.subplots(3, 1, figsize=(12, 11))
    fig.suptitle(
        "Assignment 1: Spinning Hover — Integration Method Comparison\n"
        "(Hover thrust + constant yaw torque → ω_z grows 0→8.5 rad/s in 5 s)",
        fontsize=14,
    )

    # Reference: use RK4 (most accurate translational + 4th-order orientation)
    ref = spin_data.get("RK4")

    # ── Panel 1: unwrapped yaw angle ────────────────────────────────────────
    ax = axes[0]
    for m, color, ls in zip(methods, colors, linestyles):
        if m not in spin_data:
            continue
        df = spin_data[m]
        yaw_wrapped = quat_to_yaw_deg(df.qw, df.qx, df.qy, df.qz)
        yaw_total   = np.degrees(np.unwrap(np.radians(yaw_wrapped.values)))
        ax.plot(df["time"], yaw_total, label=m, color=color, linestyle=ls, linewidth=2)
    ax.set_ylabel("Total yaw (°)")
    ax.set_title("Accumulated yaw angle  (all methods spin same amount — zoom in for difference)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ── Panel 2: yaw error relative to RK4 ─────────────────────────────────
    ax = axes[1]
    if ref is not None:
        ref_yaw = np.degrees(np.unwrap(np.radians(
            quat_to_yaw_deg(ref.qw, ref.qx, ref.qy, ref.qz).values)))
        for m, color, ls in zip(methods, colors, linestyles):
            if m == "RK4" or m not in spin_data:
                continue
            df = spin_data[m]
            yaw = np.degrees(np.unwrap(np.radians(
                quat_to_yaw_deg(df.qw, df.qx, df.qy, df.qz).values)))
            ax.plot(df["time"], yaw - ref_yaw,
                    label=f"{m} vs RK4", color=color, linestyle=ls, linewidth=2)
        ax.axhline(0, color="blue", linestyle="--", linewidth=1, alpha=0.5, label="RK4 (reference)")
        ax.set_ylabel("Yaw error vs RK4 (°)")
        ax.set_title(
            "Attitude accuracy: Euler/ExpEuler accumulate ~2.4° more yaw than RK4/ExpRK4\n"
            "(because they use end-of-step ω vs RK4 midpoint ω for orientation integration)"
        )
        ax.legend()
        ax.grid(True, alpha=0.3)

    # ── Panel 3: quaternion norm (should stay = 1.0 for all methods) ────────
    ax = axes[2]
    for m, color, ls in zip(methods, colors, linestyles):
        if m not in spin_data:
            continue
        df = spin_data[m]
        qnorm = np.sqrt(df.qw**2 + df.qx**2 + df.qy**2 + df.qz**2)
        ax.plot(df["time"], qnorm - 1.0,
                label=f"{m} |q|−1", color=color, linestyle=ls, linewidth=1.5)
    ax.axhline(0, color="black", linestyle="-", linewidth=0.8, alpha=0.4)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("|q| − 1  (deviation from unit norm)")
    ax.set_title(
        "Quaternion norm: all methods maintain |q| ≈ 1 by construction\n"
        "(exponential map / axis-angle; deviations are floating-point noise < 1e-6)"
    )
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_convergence():
    """Log-log convergence plot: position error vs step size.

    Euler/ExpEuler show O(dt¹) — error halves each time dt halves.
    RK4/ExpRK4 hit the float32 precision floor (~1e-5 m) at all dt values
    because constant-acceleration dynamics are reproduced exactly by RK4
    (the Taylor expansion of the solution terminates at O(dt²), which RK4
    captures with zero truncation error; residual ~1e-5 m is floating-point
    rounding noise accumulated over 200–32 000 steps).
    """
    methods    = ["Euler", "RK4", "ExpEuler", "ExpRK4"]
    colors     = ["red", "blue", "green", "orange"]
    markers    = ["o", "s", "^", "D"]
    linestyles = ["-", "--", ":", "-."]

    fig, ax = plt.subplots(figsize=(10, 7))
    fig.suptitle(
        "Assignment 1: Convergence Analysis — Position Error vs Step Size\n"
        "(Vertical takeoff, 2 s;  reference = RK4 at dt=0.0005 s, motor dynamics bypassed)",
        fontsize=13,
    )

    euler_dt  = None
    euler_err = None

    for method, color, marker, ls in zip(methods, colors, markers, linestyles):
        path = f"results/assignment1/data/convergence_{method}.csv"
        if not os.path.exists(path):
            continue
        df = pd.read_csv(path)
        ax.loglog(
            df["dt"], df["pos_error"],
            color=color, marker=marker, linestyle=ls,
            linewidth=2, markersize=7, label=method,
        )
        if method == "Euler":
            euler_dt  = df["dt"].values
            euler_err = df["pos_error"].values

    # O(dt¹) reference line — anchored to Euler's midpoint value
    if euler_dt is not None:
        mid = len(euler_dt) // 2
        c1  = euler_err[mid] / euler_dt[mid]
        dt_line = np.array([euler_dt[0] * 1.5, euler_dt[-1] / 1.5])
        ax.loglog(dt_line, c1 * dt_line,
                  "k--", linewidth=1.4, alpha=0.6, label="O(dt¹) slope")

        # O(dt⁴) reference line — anchored just above float32 floor for context
        c4 = 2e-3 / (euler_dt[0] ** 4)
        ax.loglog(dt_line, c4 * dt_line ** 4,
                  color="gray", linestyle=":", linewidth=1.4, alpha=0.6,
                  label="O(dt⁴) slope (reference)")

    # Annotate float32 floor
    ax.axhline(1.5e-5, color="steelblue", linestyle="-.", linewidth=0.9,
               alpha=0.5, label="float32 precision floor (~1.5×10⁻⁵ m)")

    ax.set_xlabel("Step size  dt  (s)", fontsize=12)
    ax.set_ylabel("|z_sim − z_ref|  (m)", fontsize=12)
    ax.set_title(
        "Euler/ExpEuler: O(dt) — halving dt halves error, costs 2× compute\n"
        "RK4/ExpRK4: at float32 floor (~10⁻⁵ m) for all dt — "
        "no benefit from reducing dt further",
        fontsize=10,
    )
    ax.legend(fontsize=10)
    ax.grid(True, which="both", alpha=0.3)

    plt.tight_layout()
    return fig


def print_convergence_statistics():
    """Print convergence table and measured slopes."""
    methods = ["Euler", "RK4", "ExpEuler", "ExpRK4"]
    print("\n" + "=" * 60)
    print("ASSIGNMENT 1: Convergence Analysis")
    print("=" * 60)
    print("  Reference: RK4 at dt=0.0005 s, motor dynamics bypassed")
    print()
    print(f"  {'Method':<10}  {'dt=0.04':>9}  {'dt=0.01':>9}  {'dt=0.00125':>11}  {'slope (log-log)'}")
    print("  " + "-" * 58)
    for method in methods:
        path = f"results/assignment1/data/convergence_{method}.csv"
        if not os.path.exists(path):
            continue
        df = pd.read_csv(path)
        e_coarse = float(df[df["dt"] == 0.04]["pos_error"].iloc[0])   if 0.04   in df["dt"].values else float("nan")
        e_mid    = float(df[df["dt"] == 0.01]["pos_error"].iloc[0])   if 0.01   in df["dt"].values else float("nan")
        e_fine   = float(df[df["dt"] == 0.00125]["pos_error"].iloc[0]) if 0.00125 in df["dt"].values else float("nan")
        # Measure slope over the two coarsest points (where truncation error dominates)
        dt_arr  = df["dt"].values
        err_arr = df["pos_error"].values
        if len(dt_arr) >= 2 and err_arr[0] > 0 and err_arr[1] > 0:
            slope = np.log(err_arr[0] / err_arr[1]) / np.log(dt_arr[0] / dt_arr[1])
        else:
            slope = float("nan")
        slope_str = f"{slope:.2f}" if not np.isnan(slope) else "N/A"
        print(f"  {method:<10}  {e_coarse:>9.2e}  {e_mid:>9.2e}  {e_fine:>11.2e}  {slope_str}")
    print()
    print("  Euler/ExpEuler slope ≈ 1.0  → first-order  (halving dt halves error)")
    print("  RK4/ExpRK4    slope ≈ 0.0  → at float32 precision floor ~1e-5 m")
    print("  (For non-quadratic dynamics RK4 would show slope ≈ 4.0 before the floor.)")


def print_spin_statistics():
    """Print yaw accuracy statistics for the spin scenario."""
    methods = ["Euler", "RK4", "ExpEuler", "ExpRK4"]
    print("\n" + "=" * 60)
    print("ASSIGNMENT 1: Spinning Hover — Attitude Accuracy")
    print("=" * 60)

    # Pre-compute RK4 reference so Euler (which comes first) can be compared
    ref_total = None
    rk4_path = "results/assignment1/data/trajectory_modular_RK4_spin.csv"
    if os.path.exists(rk4_path):
        rk4_df = pd.read_csv(rk4_path)
        rk4_yaw = quat_to_yaw_deg(rk4_df.qw, rk4_df.qx, rk4_df.qy, rk4_df.qz)
        ref_total = np.degrees(np.unwrap(np.radians(rk4_yaw.values)))[-1]

    for m in methods:
        path = f"results/assignment1/data/trajectory_modular_{m}_spin.csv"
        if not os.path.exists(path):
            continue
        df = pd.read_csv(path)
        yaw  = quat_to_yaw_deg(df.qw, df.qx, df.qy, df.qz)
        total = np.degrees(np.unwrap(np.radians(yaw.values)))[-1]
        qnorm_drift = float(abs(np.sqrt(df.qw**2+df.qx**2+df.qy**2+df.qz**2) - 1.0).max())
        if m == "RK4":
            err = "(reference)"
        elif ref_total is not None:
            err = f"{total - ref_total:+.3f}°"
        else:
            err = "N/A"
        print(f"  {m:10s}: total yaw = {total:8.2f}°   vs RK4: {err:12s}   "
              f"max |q|−1 = {qnorm_drift:.2e}")
    print("\nConclusion:")
    print("  • All methods keep |q| = 1 (no norm drift — axis-angle/exp-map normalise).")
    print("  • Euler/ExpEuler overshoot by ~2.4° because they use the updated (larger)")
    print("    end-of-step ω for orientation integration instead of the midpoint ω.")
    print("  • RK4/ExpRK4 use the 4th-order weighted-average ω → more accurate attitude.")
    print("  • For fast rotation (>5 rad/s), use RK4 or ExpRK4 to avoid attitude drift.")


def main():
    """Main plotting function"""
    print("Assignment 1 Visualization: Integration Method Comparison")
    print("=" * 60)

    # make sure output directories exist so plots can be written
    os.makedirs("results/assignment1/data", exist_ok=True)
    os.makedirs("results/assignment1/images", exist_ok=True)

    # Check if data files exist
    data_files = [
        f"results/assignment1/data/trajectory_modular_{method}.csv"
        for method in ["Euler", "RK4", "ExpEuler", "ExpRK4"]
    ]
    missing_files = [f for f in data_files if not os.path.exists(f)]

    if missing_files:
        print("Missing data files. Please run 'cargo run --bin assignment1' first.")
        print(f"Missing: {missing_files}")
        return

    # Create plots
    print("Generating plots...")

    # Position comparison
    fig1 = plot_position_comparison()
    if fig1:
        fig1.savefig(
            "results/assignment1/images/assignment1_position_comparison.png",
            dpi=150,
            bbox_inches="tight",
        )
        print("Saved: results/assignment1/images/assignment1_position_comparison.png")

    # Accuracy analysis
    fig2 = plot_accuracy_analysis()
    if fig2:
        fig2.savefig(
            "results/assignment1/images/assignment1_accuracy_analysis.png",
            dpi=150,
            bbox_inches="tight",
        )
        print("Saved: results/assignment1/images/assignment1_accuracy_analysis.png")

    # Spinning hover — attitude accuracy + quaternion norm
    fig3 = plot_spin_comparison()
    if fig3:
        fig3.savefig(
            "results/assignment1/images/assignment1_spin_comparison.png",
            dpi=150,
            bbox_inches="tight",
        )
        print("Saved: results/assignment1/images/assignment1_spin_comparison.png")

    # Convergence plot (optional — only if CSVs exist)
    fig4 = plot_convergence()
    if fig4:
        fig4.savefig(
            "results/assignment1/images/assignment1_convergence.png",
            dpi=150,
            bbox_inches="tight",
        )
        print("Saved: results/assignment1/images/assignment1_convergence.png")

    # Print statistics
    print_statistics()
    print_spin_statistics()
    print_convergence_statistics()

    print("\nVisualization complete!")
    print("Open the PNG files to view the results.")


if __name__ == "__main__":
    main()
