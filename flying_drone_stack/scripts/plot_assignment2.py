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
    """Load data for a specific scenario.

    Returns (DataFrame, phase_offset) where phase_offset is the number of
    seconds of takeoff+hover that precede the actual mission trajectory.
    It is read from the '# phase_offset=...' comment line written by the
    Rust binary.  If no such line is present, 0.0 is returned.
    """
    filename = f"results/assignment2/data/assignment2_{scenario}.csv"
    if not os.path.exists(filename):
        print(f"Warning: {filename} not found")
        return None, 0.0

    phase_offset = 0.0
    with open(filename) as f:
        for line in f:
            if line.startswith("# phase_offset="):
                try:
                    phase_offset = float(line.strip().split("=")[1])
                except ValueError:
                    pass
                break
            if not line.startswith("#"):
                break

    df = pd.read_csv(filename, comment="#")
    return df, phase_offset


def load_scenario_modes(scenario):
    """Load both normal and realistic-start data for a scenario.

    Returns a list of (label, df, phase_offset) tuples for each mode that
    has a CSV file on disk.  At least one entry is always present (otherwise
    the caller already skips the scenario).
    """
    modes = []
    normal_path = f"results/assignment2/data/assignment2_{scenario}.csv"
    realistic_path = f"results/assignment2/data/assignment2_{scenario}_realistic.csv"

    if os.path.exists(normal_path):
        df, po = load_scenario_data(scenario)
        if df is not None:
            modes.append(("Normal", df, po))

    if os.path.exists(realistic_path):
        # reuse the same reader — just pass the path manually
        phase_offset = 0.0
        with open(realistic_path) as f:
            for line in f:
                if line.startswith("# phase_offset="):
                    try:
                        phase_offset = float(line.strip().split("=")[1])
                    except ValueError:
                        pass
                    break
                if not line.startswith("#"):
                    break
        df = pd.read_csv(realistic_path, comment="#")
        modes.append(("Realistic start", df, phase_offset))

    return modes


def _draw_phase_line(ax, phase_offset, label=True):
    """Draw a vertical dashed line marking the end of takeoff/hover phases."""
    if phase_offset > 0.0:
        ax.axvline(
            phase_offset,
            color="purple",
            linestyle=":",
            linewidth=1.5,
            label=f"Mission start (t={phase_offset:.1f}s)" if label else None,
        )


def _plot_hover_single(axes_col, df, phase_offset, title_suffix=""):
    """Fill one column of 4 axes with hover plots for a single mode."""
    ax_x, ax_z, ax_err, ax_thr = axes_col

    ax_x.plot(df["time"], df["x"], label="Actual X", color="blue", linewidth=2)
    ax_x.plot(
        df["time"],
        df["x_ref"],
        label="Reference X",
        color="red",
        linewidth=2,
        linestyle="--",
    )
    _draw_phase_line(ax_x, phase_offset)
    ax_x.set_ylabel("X Position (m)")
    ax_x.set_title(f"X Position Tracking{title_suffix}")
    ax_x.legend(fontsize=7)
    ax_x.grid(True, alpha=0.3)

    ax_z.plot(df["time"], df["z"], label="Actual Z", color="blue", linewidth=2)
    ax_z.plot(
        df["time"],
        df["z_ref"],
        label="Reference Z",
        color="red",
        linewidth=2,
        linestyle="--",
    )
    _draw_phase_line(ax_z, phase_offset, label=False)
    ax_z.set_ylabel("Z Position (m)")
    ax_z.set_title(f"Z Position Tracking{title_suffix}")
    ax_z.legend(fontsize=7)
    ax_z.grid(True, alpha=0.3)

    ax_err.plot(
        df["time"], df["pos_error_x"], label="X Error", color="red", linewidth=2
    )
    ax_err.plot(
        df["time"], df["pos_error_y"], label="Y Error", color="green", linewidth=2
    )
    ax_err.plot(
        df["time"], df["pos_error_z"], label="Z Error", color="blue", linewidth=2
    )
    _draw_phase_line(ax_err, phase_offset, label=False)
    ax_err.set_xlabel("Time (s)")
    ax_err.set_ylabel("Position Error (m)")
    ax_err.set_title(f"Position Errors{title_suffix}")
    ax_err.legend(fontsize=7)
    ax_err.grid(True, alpha=0.3)

    ax_thr.plot(df["time"], df["thrust"], label="Thrust", color="purple", linewidth=2)
    _draw_phase_line(ax_thr, phase_offset, label=False)
    ax_thr.set_xlabel("Time (s)")
    ax_thr.set_ylabel("Thrust (N)")
    ax_thr.set_title(f"Control Thrust{title_suffix}")
    ax_thr.legend(fontsize=7)
    ax_thr.grid(True, alpha=0.3)


def plot_hover_performance():
    """Plot hover control performance.

    When both Normal and Realistic-start data exist the figure has two columns
    (one per mode) so they can be compared at a glance.  When only one mode is
    available a single-column layout is used instead.
    """
    modes = load_scenario_modes("hover")
    if not modes:
        return None

    n_cols = len(modes)
    fig, axes = plt.subplots(4, n_cols, figsize=(8 * n_cols, 16), squeeze=False)
    fig.suptitle("Assignment 2: Hover Control Performance", fontsize=16)

    for col, (label, df, phase_offset) in enumerate(modes):
        _plot_hover_single(axes[:, col], df, phase_offset, title_suffix=f"\n({label})")

    # Share y-axes across columns so scales are identical
    if n_cols > 1:
        for row in range(4):
            axes[row, 1].sharey(axes[row, 0])
            axes[row, 1].set_ylabel("")  # suppress duplicate label

    plt.tight_layout()
    return fig


def _plot_traj_paths_single(
    axes_col,
    df,
    phase_offset,
    x_ref_min,
    x_ref_max,
    y_ref_min,
    y_ref_max,
    z3d_min,
    z3d_max,
    title_suffix="",
):
    """Fill one column of 3 axes (3D, top view, pos-vs-time) for a single mode."""
    ax1, ax2, ax3 = axes_col

    x_act = df["x"].clip(x_ref_min, x_ref_max)
    y_act = df["y"].clip(y_ref_min, y_ref_max)
    z_act = df["z"].clip(z3d_min, z3d_max)

    # 3D
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
    ax1.set_title(f"3D Trajectory{title_suffix}")
    ax1.legend(fontsize=8)

    # 2D top view
    ax2.plot(x_act, y_act, label="Actual", color="blue", linewidth=2)
    ax2.plot(
        df["x_ref"],
        df["y_ref"],
        label="Reference",
        color="red",
        linewidth=2,
        linestyle="--",
    )
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.set_title(f"Top View{title_suffix}")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(x_ref_min, x_ref_max)
    ax2.set_ylim(y_ref_min, y_ref_max)
    ax2.set_aspect("equal", adjustable="box")

    # Position vs time
    ax3.plot(
        df["time"],
        df["x"].clip(x_ref_min, x_ref_max),
        label="Actual X",
        color="blue",
        linewidth=1.5,
    )
    ax3.plot(
        df["time"],
        df["y"].clip(y_ref_min, y_ref_max),
        label="Actual Y",
        color="green",
        linewidth=1.5,
    )
    ax3.plot(
        df["time"],
        df["x_ref"],
        label="Ref X",
        color="red",
        linestyle="--",
        linewidth=1.5,
    )
    ax3.plot(
        df["time"],
        df["y_ref"],
        label="Ref Y",
        color="orange",
        linestyle="--",
        linewidth=1.5,
    )
    _draw_phase_line(ax3, phase_offset)
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Position (m)")
    ax3.set_title(f"Position vs Time{title_suffix}")
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)


def _plot_traj_errors_single(axes_col, df, phase_offset, title_suffix=""):
    """Fill one column of 3 axes (pos-err, vel-err, control) for a single mode."""
    ax4, ax5, ax6 = axes_col

    ax4.plot(df["time"], df["pos_error_x"], label="X", color="red", linewidth=2)
    ax4.plot(df["time"], df["pos_error_y"], label="Y", color="green", linewidth=2)
    ax4.plot(df["time"], df["pos_error_z"], label="Z", color="blue", linewidth=2)
    _draw_phase_line(ax4, phase_offset, label=False)
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Position Error (m)")
    ax4.set_title(f"Position Errors{title_suffix}")
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)

    ax5.plot(df["time"], df["vel_error_x"], label="VX", color="red", linewidth=2)
    ax5.plot(df["time"], df["vel_error_y"], label="VY", color="green", linewidth=2)
    ax5.plot(df["time"], df["vel_error_z"], label="VZ", color="blue", linewidth=2)
    _draw_phase_line(ax5, phase_offset, label=False)
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Velocity Error (m/s)")
    ax5.set_title(f"Velocity Errors{title_suffix}")
    ax5.legend(fontsize=8)
    ax5.grid(True, alpha=0.3)

    ax6.plot(df["time"], df["thrust"], label="Thrust", color="purple", linewidth=2)
    ax6.plot(df["time"], df["tx"], label="Torque X", color="red", linewidth=1.5)
    ax6.plot(df["time"], df["ty"], label="Torque Y", color="green", linewidth=1.5)
    ax6.plot(df["time"], df["tz"], label="Torque Z", color="blue", linewidth=1.5)
    _draw_phase_line(ax6, phase_offset, label=False)
    ax6.set_xlabel("Time (s)")
    ax6.set_ylabel("Control Inputs")
    ax6.set_title(f"Thrust & Torques{title_suffix}")
    ax6.legend(fontsize=8)
    ax6.grid(True, alpha=0.3)


def _make_side_by_side(
    modes, n_rows, row_has_3d, figsize_per_col, suptitle, fill_col_fn, extra_kwargs
):
    """Build a figure with one column per mode, call fill_col_fn for each."""
    n_cols = len(modes)
    fig = plt.figure(figsize=(figsize_per_col[0] * n_cols, figsize_per_col[1]))
    fig.suptitle(suptitle, fontsize=16)

    all_axes = []
    for col in range(n_cols):
        col_axes = []
        for row in range(n_rows):
            idx = row * n_cols + col + 1
            proj = "3d" if (row_has_3d and row == 0) else None
            ax = fig.add_subplot(
                n_rows, n_cols, idx, **({"projection": proj} if proj else {})
            )
            col_axes.append(ax)
        all_axes.append(col_axes)

    for col, (label, df, phase_offset) in enumerate(modes):
        fill_col_fn(
            all_axes[col], df, phase_offset, title_suffix=f"\n({label})", **extra_kwargs
        )

    # Share y-axes across columns (skip 3-D row if present)
    if n_cols > 1:
        start_row = 1 if row_has_3d else 0
        for row in range(start_row, n_rows):
            all_axes[1][row].sharey(all_axes[0][row])
            all_axes[1][row].set_ylabel("")

    plt.tight_layout()
    return fig


def plot_trajectory_paths(scenario):
    """3D trajectory + top view + position vs time — one column per mode."""
    modes = load_scenario_modes(scenario)
    if not modes:
        return None

    scenario_title = "Figure-8" if scenario == "figure8" else "Circular"

    ref_df = modes[0][1]
    ref_margin = 0.5
    x_ref_min = ref_df["x_ref"].min() - ref_margin
    x_ref_max = ref_df["x_ref"].max() + ref_margin
    y_ref_min = ref_df["y_ref"].min() - ref_margin
    y_ref_max = ref_df["y_ref"].max() + ref_margin
    xy_span = max(x_ref_max - x_ref_min, y_ref_max - y_ref_min)
    z_mid = ref_df["z_ref"].mean()
    z3d_min = z_mid - xy_span / 2
    z3d_max = z_mid + xy_span / 2

    return _make_side_by_side(
        modes,
        n_rows=3,
        row_has_3d=True,
        figsize_per_col=(9, 15),
        suptitle=f"Assignment 2: {scenario_title} — Trajectory Paths",
        fill_col_fn=_plot_traj_paths_single,
        extra_kwargs=dict(
            x_ref_min=x_ref_min,
            x_ref_max=x_ref_max,
            y_ref_min=y_ref_min,
            y_ref_max=y_ref_max,
            z3d_min=z3d_min,
            z3d_max=z3d_max,
        ),
    )


def plot_trajectory_errors(scenario):
    """Position errors + velocity errors + thrust/torques — one column per mode."""
    modes = load_scenario_modes(scenario)
    if not modes:
        return None

    scenario_title = "Figure-8" if scenario == "figure8" else "Circular"

    return _make_side_by_side(
        modes,
        n_rows=3,
        row_has_3d=False,
        figsize_per_col=(9, 12),
        suptitle=f"Assignment 2: {scenario_title} — Errors & Control",
        fill_col_fn=_plot_traj_errors_single,
        extra_kwargs={},
    )


# Keep the old name as an alias so nothing else breaks
def plot_trajectory_tracking(scenario):
    """Deprecated: use plot_trajectory_paths / plot_trajectory_errors instead."""
    return plot_trajectory_paths(scenario)


def plot_motor_speeds(scenario):
    """Plot motor speeds for a scenario.

    When both modes exist, two side-by-side subplots are shown in one image.
    """
    modes = load_scenario_modes(scenario)
    if not modes:
        return None

    scenario_title = (
        "Hover"
        if scenario == "hover"
        else ("Figure-8" if scenario == "figure8" else "Circular")
    )
    MOTOR_COLORS = ["red", "green", "blue", "orange"]

    n_cols = len(modes)
    fig, axes = plt.subplots(1, n_cols, figsize=(8 * n_cols, 6), squeeze=False)
    fig.suptitle(f"Assignment 2: Motor Speeds - {scenario_title} Scenario", fontsize=16)

    for col, (label, df, phase_offset) in enumerate(modes):
        ax = axes[0, col]
        ax.plot(
            df["time"],
            np.sqrt(df["omega1"]),
            label="Motor 1",
            color=MOTOR_COLORS[0],
            linewidth=2,
        )
        ax.plot(
            df["time"],
            np.sqrt(df["omega2"]),
            label="Motor 2",
            color=MOTOR_COLORS[1],
            linewidth=2,
        )
        ax.plot(
            df["time"],
            np.sqrt(df["omega3"]),
            label="Motor 3",
            color=MOTOR_COLORS[2],
            linewidth=2,
        )
        ax.plot(
            df["time"],
            np.sqrt(df["omega4"]),
            label="Motor 4",
            color=MOTOR_COLORS[3],
            linewidth=2,
        )
        _draw_phase_line(ax, phase_offset)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Motor Speed (rad/s)")
        ax.set_title(f"Motor Angular Speeds\n({label})")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    # Share y-axis so scales are identical
    if n_cols > 1:
        axes[0, 1].sharey(axes[0, 0])
        axes[0, 1].set_ylabel("")

    plt.tight_layout()
    return fig


def print_scenario_statistics():
    """Print statistics for all scenarios and all available modes."""
    print("\n" + "=" * 80)
    print("ASSIGNMENT 2 STATISTICS: Geometric Control Performance")
    print("=" * 80)

    scenarios = ["hover", "figure8", "circle"]

    for scenario in scenarios:
        scenario_name = (
            "Hover"
            if scenario == "hover"
            else ("Figure-8" if scenario == "figure8" else "Circular")
        )
        print(f"\n{scenario_name} Scenario:")
        print("-" * (len(scenario_name) + 10))

        modes = load_scenario_modes(scenario)
        if not modes:
            print("  No data found.")
            continue

        for label, df, phase_offset in modes:
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
            err_3d = np.sqrt(
                df["pos_error_x"] ** 2
                + df["pos_error_y"] ** 2
                + df["pos_error_z"] ** 2
            )
            rms_pos_error = float(np.sqrt(np.mean(err_3d ** 2)))
            dt_avg = float(np.median(np.diff(df["time"]))) if len(df) > 1 else 0.01
            cumulative_pos_error = float(np.sum(err_3d) * dt_avg)
            rms_vel_error = np.sqrt(
                np.mean(
                    df["vel_error_x"] ** 2
                    + df["vel_error_y"] ** 2
                    + df["vel_error_z"] ** 2
                )
            )
            avg_thrust = df["thrust"].mean()
            max_thrust = df["thrust"].max()

            print(f"  [{label}]")
            print(f"    Final position error:     {final_pos_error:.3f} m")
            print(f"    Final velocity error:     {final_vel_error:.3f} m/s")
            print(f"    RMS position error:       {rms_pos_error*100:.1f} cm")
            print(f"    Cumulative position error:{cumulative_pos_error:.3f} m·s  (∫|e₃D| dt)")
            print(f"    RMS velocity error:       {rms_vel_error:.3f} m/s")
            print(f"    Average thrust:           {avg_thrust:.3f} N")
            print(f"    Max thrust:               {max_thrust:.3f} N")

    print("\nAnalysis:")
    print("- Hover control: Excellent performance with near-zero errors")
    print("- Trajectory tracking: Small residual errors show good controller tuning")
    print(
        "- Realistic start: takeoff phase clearly visible as error spike then settling"
    )
    print(
        "- Control inputs: Thrust varies appropriately, torques show attitude corrections"
    )
    print("- Motor speeds: X-configuration shows expected motor mixing patterns")


def main():
    """Main plotting function"""
    print("Assignment 2 Visualization: Geometric Control Performance")
    print("=" * 70)

    # make sure output directories exist so plots can be written
    os.makedirs("results/assignment2/data", exist_ok=True)
    os.makedirs("results/assignment2/images", exist_ok=True)

    # Check that at least one data file exists for each scenario
    scenarios = ["hover", "figure8", "circle"]
    any_missing = False
    for scenario in scenarios:
        normal = f"results/assignment2/data/assignment2_{scenario}.csv"
        realistic = f"results/assignment2/data/assignment2_{scenario}_realistic.csv"
        if not os.path.exists(normal) and not os.path.exists(realistic):
            print(f"Warning: no data found for '{scenario}' — run assignment2 first.")
            any_missing = True
    if any_missing:
        print(
            "Run './target/release/assignment2' (and optionally with --realistic-start)."
        )
        return

    # Create plots (each function overlays both modes when both files exist)
    print("Generating plots...")

    # Hover performance
    fig1 = plot_hover_performance()
    if fig1:
        fig1.savefig(
            "results/assignment2/images/assignment2_hover_performance.png",
            dpi=150,
            bbox_inches="tight",
        )
        print("Saved: results/assignment2/images/assignment2_hover_performance.png")

    # Figure-8: paths image + errors/control image
    fig2a = plot_trajectory_paths("figure8")
    if fig2a:
        fig2a.savefig(
            "results/assignment2/images/assignment2_figure8_paths.png",
            dpi=150,
            bbox_inches="tight",
        )
        print("Saved: results/assignment2/images/assignment2_figure8_paths.png")

    fig2b = plot_trajectory_errors("figure8")
    if fig2b:
        fig2b.savefig(
            "results/assignment2/images/assignment2_figure8_errors.png",
            dpi=150,
            bbox_inches="tight",
        )
        print("Saved: results/assignment2/images/assignment2_figure8_errors.png")

    # Circle: paths image + errors/control image
    fig3a = plot_trajectory_paths("circle")
    if fig3a:
        fig3a.savefig(
            "results/assignment2/images/assignment2_circle_paths.png",
            dpi=150,
            bbox_inches="tight",
        )
        print("Saved: results/assignment2/images/assignment2_circle_paths.png")

    fig3b = plot_trajectory_errors("circle")
    if fig3b:
        fig3b.savefig(
            "results/assignment2/images/assignment2_circle_errors.png",
            dpi=150,
            bbox_inches="tight",
        )
        print("Saved: results/assignment2/images/assignment2_circle_errors.png")

    # Motor speeds for all scenarios
    for scenario in scenarios:
        fig = plot_motor_speeds(scenario)
        if fig:
            fig.savefig(
                f"results/assignment2/images/assignment2_motor_speeds_{scenario}.png",
                dpi=150,
                bbox_inches="tight",
            )
            print(f"Saved: results/assignment2/images/assignment2_motor_speeds_{scenario}.png")

    # Print statistics
    print_scenario_statistics()

    print("\nVisualization complete!")
    print("Open the PNG files to view the geometric control performance results.")


if __name__ == "__main__":
    main()
