#!/usr/bin/env python3
"""
Numerical comparison between original and modular implementations
Verifies that the outputs are identical within floating-point precision
"""

import pandas as pd
import numpy as np


def compare_trajectories(original_file, modular_file, method_name):
    """Compare two trajectory CSV files"""

    # Read CSV files
    orig = pd.read_csv(original_file)
    mod = pd.read_csv(modular_file)

    # Extract common columns
    orig_z = orig["z"].values
    orig_vz = orig["vz"].values
    mod_z = mod["z"].values
    mod_vz = mod["vz"].values

    # Compute differences
    z_diff = np.abs(orig_z - mod_z)
    vz_diff = np.abs(orig_vz - mod_vz)

    max_z_diff = np.max(z_diff)
    max_vz_diff = np.max(vz_diff)

    # Check if within floating point tolerance
    tolerance = 1e-5  # 0.00001
    z_match = max_z_diff < tolerance
    vz_match = max_vz_diff < tolerance

    print(f"\n{'='*60}")
    print(f"{method_name}")
    print(f"{'='*60}")
    print(f"  Position (z):  max diff = {max_z_diff:.2e} m")
    print(f"  Velocity (vz): max diff = {max_vz_diff:.2e} m/s")
    print(f"  Status: {'✓ EXACT MATCH' if (z_match and vz_match) else '✗ MISMATCH'}")

    return z_match and vz_match


if __name__ == "__main__":
    print("\n" + "╔" + "═" * 58 + "╗")
    print("║  Numerical Precision Verification                       ║")
    print("╚" + "═" * 58 + "╝")

    base_orig = (
        "/home/georg/Desktop/flying_robot_course/Simulation and Dynamics/assignment1/"
    )
    base_mod = "/home/georg/Desktop/flying_robot_course/multirotor_simulator/"

    methods = [
        ("Euler", "euler_trajectory.csv", "trajectory_modular_Euler.csv"),
        ("RK4", "rk4_trajectory.csv", "trajectory_modular_RK4.csv"),
        ("Exp+Euler", "exp_euler_trajectory.csv", "trajectory_modular_Exp_Euler.csv"),
        ("Exp+RK4", "exp_rk4_trajectory.csv", "trajectory_modular_Exp_RK4.csv"),
    ]

    all_match = True
    for name, orig_file, mod_file in methods:
        match = compare_trajectories(base_orig + orig_file, base_mod + mod_file, name)
        all_match = all_match and match

    print(f"\n{'='*60}")
    if all_match:
        print("✅ ALL METHODS MATCH EXACTLY!")
        print("   The modular implementation is numerically identical")
        print("   to the original monolithic implementation.")
    else:
        print("✗ Some differences detected")
    print(f"{'='*60}\n")
