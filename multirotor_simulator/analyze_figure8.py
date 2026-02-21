#!/usr/bin/env python3
import pandas as pd
import numpy as np

# Load the figure-8 data
df = pd.read_csv("results/data/assignment2_figure8.csv")

# Calculate position errors
pos_error = np.sqrt(
    (df["x"] - df["x_ref"]) ** 2
    + (df["y"] - df["y_ref"]) ** 2
    + (df["z"] - df["z_ref"]) ** 2
)

# Find max and average errors
max_error = pos_error.max()
avg_error = pos_error.mean()

print(f"Figure-8 Trajectory Analysis:")
print(f"  Max position error: {max_error*1000:.1f} mm")
print(f"  Avg position error: {avg_error*1000:.1f} mm")

# Find the time of max error
max_idx = pos_error.idxmax()
print(f"\nAt max error (t={df.loc[max_idx, 'time']:.2f}s):")
print(
    f"  Commanded: ({df.loc[max_idx, 'x_ref']:.3f}, {df.loc[max_idx, 'y_ref']:.3f}, {df.loc[max_idx, 'z_ref']:.3f})"
)
print(
    f"  Actual:    ({df.loc[max_idx, 'x']:.3f}, {df.loc[max_idx, 'y']:.3f}, {df.loc[max_idx, 'z']:.3f})"
)

# Analyze attitude tracking
# Extract quaternion and compute roll/pitch angles
qw, qx, qy, qz = df["qw"], df["qx"], df["qy"], df["qz"]
roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
pitch = np.arcsin(2 * (qw * qy - qz * qx))

print(f"\nAttitude Analysis:")
print(f"  Max roll:  {np.rad2deg(abs(roll).max()):.2f}°")
print(f"  Max pitch: {np.rad2deg(abs(pitch).max()):.2f}°")

# Look at angular velocity
omega_mag = np.sqrt(df["wx"] ** 2 + df["wy"] ** 2 + df["wz"] ** 2)
print(f"  Max angular velocity: {np.rad2deg(omega_mag.max()):.1f}°/s")
