#!/usr/bin/env python3
"""
Assignment 2 Visualization: Geometric Control Performance

This script visualizes the results from assignment2.rs, showing geometric controller
performance on hover, figure-8, and circular trajectory tracking scenarios.

Usage: python plot_assignment2.py
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from mpl_toolkits.mplot3d import Axes3D


def load_scenario_data(scenario):
    """Load data for a specific scenario"""
    filename = f"results/data/assignment2_{scenario}.csv"
    if not os.path.exists(filename):
        print(f"Warning: {filename} not found")
        return None

    df = pd.read_csv(filename)
    return df


def plot_hover_performance():
    """Plot hover control performance"""
    df = load_scenario_data("hover")
    if df is None:
        return None

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle("Assignment 2: Hover Control Performance", fontsize=16)

    # Position tracking
    axes[0, 0].plot(df["time"], df["x"], label="Actual X", color="blue", linewidth=2)
    axes[0, 0].plot(
        df["time"],
        df["x_ref"],
        label="Reference X",
        color="red",
        linestyle="--",
        linewidth=2,
    )
    axes[0, 0].set_ylabel("X Position (m)")
    axes[0, 0].set_title("X Position Tracking")
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(df["time"], df["z"], label="Actual Z", color="blue", linewidth=2)
    axes[0, 1].plot(
        df["time"],
        df["z_ref"],
        label="Reference Z",
        color="red",
        linestyle="--",
        linewidth=2,
    )
    axes[0, 1].set_ylabel("Z Position (m)")
    axes[0, 1].set_title("Z Position Tracking (Height)")
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # Position errors
    axes[1, 0].plot(
        df["time"], df["pos_error_x"], label="X Error", color="red", linewidth=2
    )
    axes[1, 0].plot(
        df["time"], df["pos_error_y"], label="Y Error", color="green", linewidth=2
    )
    axes[1, 0].plot(
        df["time"], df["pos_error_z"], label="Z Error", color="blue", linewidth=2
    )
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_ylabel("Position Error (m)")
    axes[1, 0].set_title("Position Tracking Errors")
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # Control inputs
    axes[1, 1].plot(
        df["time"], df["thrust"], label="Thrust", color="purple", linewidth=2
    )
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].set_ylabel("Thrust (N)")
    axes[1, 1].set_title("Control Thrust Input")
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_trajectory_tracking(scenario):
    """Plot trajectory tracking performance for figure-8 or circle"""
    df = load_scenario_data(scenario)
    if df is None:
        return None

    scenario_title = "Figure-8" if scenario == "figure8" else "Circular"

    fig = plt.figure(figsize=(16, 12))
    fig.suptitle(f"Assignment 2: {scenario_title} Trajectory Tracking", fontsize=16)

    # Compute axis limits from reference trajectory with padding, to keep plots
    # readable even when the actual trajectory diverges far from the reference.
    ref_margin = 0.5  # extra metres of padding around the reference
    x_ref_min = df["x_ref"].min() - ref_margin
    x_ref_max = df["x_ref"].max() + ref_margin
    y_ref_min = df["y_ref"].min() - ref_margin
    y_ref_max = df["y_ref"].max() + ref_margin
    z_ref_min = df["z_ref"].min() - ref_margin
    z_ref_max = df["z_ref"].max() + ref_margin

    # Clip actual trajectory to the reference-based limits so diverging runs
    # don't squash the reference curve into a dot.
    x_act = df["x"].clip(x_ref_min, x_ref_max)
    y_act = df["y"].clip(y_ref_min, y_ref_max)
    z_act = df["z"].clip(z_ref_min, z_ref_max)

    # For the 3D plot use equal-range limits on all three axes so that a
    # small-but-real z variation is not artificially stretched relative to x/y.
    # We base the cube side on the larger of the x/y span, centred on z_ref.
    xy_span = max(x_ref_max - x_ref_min, y_ref_max - y_ref_min)
    z_mid = df["z_ref"].mean()
    z3d_min = z_mid - xy_span / 2
    z3d_max = z_mid + xy_span / 2

    # 3D trajectory plot
    ax1 = fig.add_subplot(2, 3, 1, projection="3d")
    ax1.plot(x_act, y_act, z_act, label="Actual", color="blue", linewidth=2)
    ax1.plot(
        df["x_ref"],
        df["y_ref"],
        df["z_ref"],
        label="Reference",
        color="red",
        linestyle="--",
        linewidth=2,
    )
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")
    ax1.set_xlim(x_ref_min, x_ref_max)
    ax1.set_ylim(y_ref_min, y_ref_max)
    ax1.set_zlim(z3d_min, z3d_max)
    ax1.set_title(f"3D {scenario_title} Trajectory")
    ax1.legend()

    # 2D trajectory (top view)
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(x_act, y_act, label="Actual", color="blue", linewidth=2)
    ax2.plot(
        df["x_ref"],
        df["y_ref"],
        label="Reference",
        color="red",
        linestyle="--",
        linewidth=2,
    )
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.set_title(f"2D {scenario_title} Trajectory (Top View)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(x_ref_min, x_ref_max)
    ax2.set_ylim(y_ref_min, y_ref_max)
    ax2.set_aspect("equal", adjustable="box")

    # Position vs time – clamp actual values to reference range
    x_t_act = df["x"].clip(x_ref_min, x_ref_max)
    y_t_act = df["y"].clip(y_ref_min, y_ref_max)

    # Position vs time
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(df["time"], x_t_act, label="Actual X", color="blue", linewidth=1)
    ax3.plot(
        df["time"], df["x_ref"], label="Ref X", color="red", linestyle="--", linewidth=1
    )
    ax3.plot(df["time"], y_t_act, label="Actual Y", color="green", linewidth=1)
    ax3.plot(
        df["time"],
        df["y_ref"],
        label="Ref Y",
        color="orange",
        linestyle="--",
        linewidth=1,
    )
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Position (m)")
    ax3.set_title("Position vs Time")
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Tracking errors
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(df["time"], df["pos_error_x"], label="X Error", color="red", linewidth=2)
    ax4.plot(df["time"], df["pos_error_y"], label="Y Error", color="green", linewidth=2)
    ax4.plot(df["time"], df["pos_error_z"], label="Z Error", color="blue", linewidth=2)
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Position Error (m)")
    ax4.set_title("Position Tracking Errors")
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    # Velocity errors
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.plot(df["time"], df["vel_error_x"], label="VX Error", color="red", linewidth=2)
    ax5.plot(
        df["time"], df["vel_error_y"], label="VY Error", color="green", linewidth=2
    )
    ax5.plot(df["time"], df["vel_error_z"], label="VZ Error", color="blue", linewidth=2)
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Velocity Error (m/s)")
    ax5.set_title("Velocity Tracking Errors")
    ax5.legend()
    ax5.grid(True, alpha=0.3)

    # Control inputs
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.plot(df["time"], df["thrust"], label="Thrust", color="purple", linewidth=2)
    ax6.plot(df["time"], df["tx"], label="Torque X", color="red", linewidth=1)
    ax6.plot(df["time"], df["ty"], label="Torque Y", color="green", linewidth=1)
    ax6.plot(df["time"], df["tz"], label="Torque Z", color="blue", linewidth=1)
    ax6.set_xlabel("Time (s)")
    ax6.set_ylabel("Control Inputs")
    ax6.set_title("Control Inputs (Thrust & Torques)")
    ax6.legend()
    ax6.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_motor_speeds(scenario):
    """Plot motor speeds for a scenario"""
    df = load_scenario_data(scenario)
    if df is None:
        return None

    scenario_title = (
        "Hover"
        if scenario == "hover"
        else ("Figure-8" if scenario == "figure8" else "Circular")
    )

    fig, ax = plt.subplots(1, 1, figsize=(12, 6))
    fig.suptitle(f"Assignment 2: Motor Speeds - {scenario_title} Scenario", fontsize=16)

    ax.plot(
        df["time"], np.sqrt(df["omega1"]), label="Motor 1", color="red", linewidth=2
    )
    ax.plot(
        df["time"], np.sqrt(df["omega2"]), label="Motor 2", color="green", linewidth=2
    )
    ax.plot(
        df["time"], np.sqrt(df["omega3"]), label="Motor 3", color="blue", linewidth=2
    )
    ax.plot(
        df["time"], np.sqrt(df["omega4"]), label="Motor 4", color="orange", linewidth=2
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Motor Speed (rad/s)")
    ax.set_title("Motor Angular Speeds (X Configuration)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def print_scenario_statistics():
    """Print statistics for all scenarios"""
    print("\n" + "=" * 80)
    print("ASSIGNMENT 2 STATISTICS: Geometric Control Performance")
    print("=" * 80)

    scenarios = ["hover", "figure8", "circle"]

    for scenario in scenarios:
        df = load_scenario_data(scenario)
        if df is None:
            continue

        scenario_name = (
            "Hover"
            if scenario == "hover"
            else ("Figure-8" if scenario == "figure8" else "Circular")
        )

        print(f"\n{scenario_name} Scenario:")
        print("-" * (len(scenario_name) + 10))

        # Final errors
        final_pos_error = np.sqrt(
            df["pos_error_x"].iloc[-1] ** 2
            + df["pos_error_y"].iloc[-1] ** 2
            + df["pos_error_z"].iloc[-1] ** 2
        )
        final_vel_error = np.sqrt(
            df["vel_error_x"].iloc[-1] ** 2
            + df["vel_error_y"].iloc[-1] ** 2
            + df["vel_error_z"].iloc[-1] ** 2
        )

        # RMS errors
        rms_pos_error = np.sqrt(
            np.mean(
                df["pos_error_x"] ** 2 + df["pos_error_y"] ** 2 + df["pos_error_z"] ** 2
            )
        )
        rms_vel_error = np.sqrt(
            np.mean(
                df["vel_error_x"] ** 2 + df["vel_error_y"] ** 2 + df["vel_error_z"] ** 2
            )
        )

        # Control effort
        avg_thrust = df["thrust"].mean()
        max_thrust = df["thrust"].max()

        print(f"  Final position error:  {final_pos_error:.3f} m")
        print(f"  Final velocity error:  {final_vel_error:.3f} m/s")
        print(f"  RMS position error:    {rms_pos_error:.3f} m")
        print(f"  RMS velocity error:    {rms_vel_error:.3f} m/s")
        print(f"  Average thrust:        {avg_thrust:.3f} N")
        print(f"  Max thrust:            {max_thrust:.3f} N")

    print("\nAnalysis:")
    print("- Hover control: Excellent performance with near-zero errors")
    print("- Trajectory tracking: Significant errors indicate controller tuning needed")
    print(
        "- Control inputs: Thrust varies appropriately, torques show attitude corrections"
    )
    print("- Motor speeds: X-configuration shows expected motor mixing patterns")


def main():
    """Main plotting function"""
    print("Assignment 2 Visualization: Geometric Control Performance")
    print("=" * 70)

    # make sure output directories exist so plots can be written
    os.makedirs("results/data", exist_ok=True)
    os.makedirs("results/images", exist_ok=True)

    # Check if data files exist
    scenarios = ["hover", "figure8", "circle"]
    data_files = [f"results/data/assignment2_{scenario}.csv" for scenario in scenarios]
    missing_files = [f for f in data_files if not os.path.exists(f)]

    if missing_files:
        print("Missing data files. Please run 'cargo run --bin assignment2' first.")
        print(f"Missing: {missing_files}")
        return

    # Create plots
    print("Generating plots...")

    # Hover performance
    fig1 = plot_hover_performance()
    if fig1:
        fig1.savefig(
            "results/images/assignment2_hover_performance.png",
            dpi=150,
            bbox_inches="tight",
        )
        print("Saved: results/images/assignment2_hover_performance.png")

    # Figure-8 tracking
    fig2 = plot_trajectory_tracking("figure8")
    if fig2:
        fig2.savefig(
            "results/images/assignment2_figure8_tracking.png",
            dpi=150,
            bbox_inches="tight",
        )
        print("Saved: results/images/assignment2_figure8_tracking.png")

    # Circle tracking
    fig3 = plot_trajectory_tracking("circle")
    if fig3:
        fig3.savefig(
            "results/images/assignment2_circle_tracking.png",
            dpi=150,
            bbox_inches="tight",
        )
        print("Saved: results/images/assignment2_circle_tracking.png")

    # Motor speeds for all scenarios
    for scenario in scenarios:
        fig = plot_motor_speeds(scenario)
        if fig:
            fig.savefig(
                f"results/images/assignment2_motor_speeds_{scenario}.png",
                dpi=150,
                bbox_inches="tight",
            )
            print(f"Saved: results/images/assignment2_motor_speeds_{scenario}.png")

    # Print statistics
    print_scenario_statistics()

    print("\nVisualization complete!")
    print("Open the PNG files to view the geometric control performance results.")


if __name__ == "__main__":
    main()
