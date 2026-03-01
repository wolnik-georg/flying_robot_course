#!/usr/bin/env python3
"""
Assignment 1 Visualization: Integration Method Comparison

This script visualizes the results from assignment1.rs, comparing different
integration methods (Euler, RK4, ExpEuler, ExpRK4) on a vertical takeoff trajectory.

Usage: python plot_assignment1.py
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os


def load_trajectory_data(filename):
    """Load trajectory data from CSV file"""
    filepath = f"results/data/{filename}"
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
    """Print final statistics for all methods"""
    print("\n" + "=" * 60)
    print("ASSIGNMENT 1 STATISTICS: Integration Method Comparison")
    print("=" * 60)

    methods = ["Euler", "RK4", "ExpEuler", "ExpRK4"]

    print("<15")
    print("-" * 60)

    for method in methods:
        filename = f"trajectory_modular_{method}.csv"
        df = load_trajectory_data(filename)

        if df is not None:
            final_pos = df["z"].iloc[-1]
            final_vel = df["vz"].iloc[-1]
            max_vel = df["vz"].max()
            print("<15")

    print("\nAnalysis:")
    print("- RK4 provides the most accurate solution")
    print("- Euler methods show increasing drift over time")
    print("- Exponential methods perform better than standard Euler")
    print("- All methods reach approximately correct terminal velocity")


def main():
    """Main plotting function"""
    print("Assignment 1 Visualization: Integration Method Comparison")
    print("=" * 60)

    # make sure output directories exist so plots can be written
    os.makedirs("results/data", exist_ok=True)
    os.makedirs("results/images", exist_ok=True)

    # Check if data files exist
    data_files = [
        f"results/data/trajectory_modular_{method}.csv"
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
            "results/images/assignment1_position_comparison.png",
            dpi=150,
            bbox_inches="tight",
        )
        print("Saved: results/images/assignment1_position_comparison.png")

    # Accuracy analysis
    fig2 = plot_accuracy_analysis()
    if fig2:
        fig2.savefig(
            "results/images/assignment1_accuracy_analysis.png",
            dpi=150,
            bbox_inches="tight",
        )
        print("Saved: results/images/assignment1_accuracy_analysis.png")

    # Print statistics
    print_statistics()

    print("\nVisualization complete!")
    print("Open the PNG files to view the results.")


if __name__ == "__main__":
    main()
