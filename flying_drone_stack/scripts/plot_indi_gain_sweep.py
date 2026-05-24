#!/usr/bin/env python3
"""plot_indi_gain_sweep.py — Visualise INDI gain sweep (KR × ζ grid, all 3 maneuvers).

Reads the CSV from `indi_gain_sweep`.  Only the gain sub-sweep is plotted
(filter sub-sweep is noiseless-sim-insensitive — see note at bottom).

Usage:
    cargo run --release --bin indi_gain_sweep 2>/dev/null | tee results/indi_gain_sweep.csv
    python scripts/plot_indi_gain_sweep.py results/indi_gain_sweep.csv

Output: results/images/indi_gain_sweep.png  +  lab recommendation table on stdout
"""

import sys, os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
IMG_DIR = os.path.join(ROOT, "results", "images")
os.makedirs(IMG_DIR, exist_ok=True)

if len(sys.argv) < 2:
    print("Usage: python plot_indi_gain_sweep.py <csv_path>")
    sys.exit(1)

df = pd.read_csv(sys.argv[1])
for col in ["rmse_xy_m", "rmse_z_m", "peak_xy_m"]:
    df[col] = pd.to_numeric(df[col], errors="coerce")

gain_df = df[df["sweep"] == "gain"].copy()

kr_vals   = sorted(gain_df["kr_xy"].unique())
zeta_vals = sorted(gain_df["zeta"].unique())
maneuvers = ["hover", "circle", "figure8"]
titles = {
    "hover":   "Hover  (metric: Z RMSE)",
    "circle":  "Circle  r=0.3m  ω=1 rad/s  (XY RMSE)",
    "figure8": "Figure-8  a=0.92m  T=7.28s  (XY RMSE)",
}

def make_grid(maneuver, col):
    grid = np.full((len(zeta_vals), len(kr_vals)), np.nan)
    sub = gain_df[gain_df["maneuver"] == maneuver]
    for _, row in sub.iterrows():
        if row["kr_xy"] not in kr_vals or row["zeta"] not in zeta_vals:
            continue
        zi = zeta_vals.index(row["zeta"])
        ki = kr_vals.index(row["kr_xy"])
        if row["status"] == "ok":
            grid[zi, ki] = row[col]
    return grid

primary_col = {"hover": "rmse_z_m", "circle": "rmse_xy_m", "figure8": "rmse_xy_m"}
vmaxes      = {"hover": 0.05,        "circle": 0.30,        "figure8": 0.40}

extent = [min(kr_vals) - 60, max(kr_vals) + 60,
          min(zeta_vals) - 0.07, max(zeta_vals) + 0.07]

fig, axes = plt.subplots(1, 3, figsize=(17, 6))
fig.suptitle(
    "INDI gain sweep — KR_xy × ζ  ·  Outer loop: KP=28 KV=6 (firmware KP=28 KV=3; KV=6 in sim\n"
    "to avoid false divergence from missing mocap delay)  ·  Filters fixed 60/60 Hz",
    fontsize=9)

for ax, maneuver in zip(axes, maneuvers):
    grid = make_grid(maneuver, primary_col[maneuver])
    vmax = vmaxes[maneuver]

    im = ax.imshow(grid, origin="lower", aspect="auto",
                   vmin=0.0, vmax=vmax, cmap="RdYlGn_r", extent=extent)

    for zi, zeta in enumerate(zeta_vals):
        for ki, kr in enumerate(kr_vals):
            v = grid[zi, ki]
            if np.isnan(v):
                ax.text(kr, zeta, "DIV", ha="center", va="center", fontsize=6,
                        color="white",
                        bbox=dict(boxstyle="round,pad=0.1", fc="#555", ec="none", alpha=0.85))
            else:
                txt = f"{v*100:.1f}"
                ax.text(kr, zeta, txt, ha="center", va="center", fontsize=6.5, color="black")

    # Bandwidth cascade boundary: att_ωₙ = 5 × pos_ωₙ  →  KR = (5√KP)² = 25·KP = 700
    ax.axvline(700, color="navy", linestyle="--", linewidth=1.2, alpha=0.7,
               label="BW cascade KR=700\n(att_ωₙ = 5×pos_ωₙ)")

    cb = fig.colorbar(im, ax=ax, shrink=0.85)
    metric = "Z RMSE" if maneuver == "hover" else "XY RMSE"
    cb.set_label(f"{metric} [m]  (cell = cm)", fontsize=8)
    ax.set_xlabel("KR_xy  [1/s²]    ωₙ = √KR rad/s", fontsize=8)
    ax.set_ylabel("ζ = KW / (2·ωₙ)", fontsize=8)
    ax.set_title(titles[maneuver], fontsize=9)
    ax.legend(fontsize=7, loc="upper left")

fig.tight_layout()
out_path = os.path.join(IMG_DIR, "indi_gain_sweep.png")
fig.savefig(out_path, dpi=130, bbox_inches="tight")
plt.close(fig)
print(f"Saved: {out_path}")

# ── Lab recommendation table ───────────────────────────────────────────────────
print()
print("=" * 75)
print("SIMULATION FINDINGS")
print("=" * 75)
print()
print("HOVER: entire grid stable — any KR/ζ works for first hover.")
print()

for maneuver in ["circle", "figure8"]:
    sub = gain_df[(gain_df["maneuver"] == maneuver) & (gain_df["status"] == "ok")].copy()
    col = primary_col[maneuver]
    sub = sub.dropna(subset=[col]).sort_values(col)
    label = "Circle (XY RMSE)" if maneuver == "circle" else "Figure-8 (XY RMSE)"
    print(f"{label}  — top stable cells:")
    print(f"  {'KR':>6}  {'ζ':>5}  {'ωₙ [rad/s]':>10}  {'KW':>7}  {'RMSE [cm]':>10}")
    for _, row in sub.head(6).iterrows():
        omega_n = row["kr_xy"] ** 0.5
        print(f"  {row['kr_xy']:>6.0f}  {row['zeta']:>5.1f}  {omega_n:>10.1f}  "
              f"{row['kw_xy']:>7.1f}  {row[col]*100:>10.1f}")
    print()

print("=" * 75)
print("LAB SEQUENCE")
print("=" * 75)
print("""
Step 1  Hover  KR=100   ζ=1.4  KW=28    safe first INDI hover (any gain works)
Step 2  Hover  KR=800   ζ=0.8  KW=45    raise to trajectory-capable bandwidth
Step 3  Circle KR=800   ζ=0.8  KW=45    first INDI trajectory (sim: ~20 cm RMSE)
Step 4  Circle KR=1000  ζ=0.8  KW=51    better (~13 cm sim RMSE)
Step 5  Circle KR=1200  ζ=0.7  KW=48    sim best (~11 cm, matches geo baseline)
Step 6  F-8    KR=1200  ζ=0.7  KW=48    same gains, compare RMSE vs geometric
""")
print("NOTES")
print("  - Bandwidth cascade: KR < 700 diverges in circle/figure-8 sim (KP=28 outer loop)")
print("    → KR=100 hover is fine, but raise to ≥800 before any trajectory flight")
print("  - Filter cutoffs: sim is noiseless — filter sweep gave identical RMSE for all")
print("    cutoffs. Keep firmware defaults (60/60 Hz) for first flights; tune on hardware")
print("    if gyro noise causes α_meas to be too noisy (visible in /cf231/indi_state)")
print("  - Sim outer loop uses KV=6 (hardware=3). KV=3 in perfect noiseless sim causes")
print("    false divergence; hardware has 20 Hz update delay that provides damping.")
print("  - Absolute RMSE values: sim ~11 cm vs geo real-flight ~12 cm — suggests INDI")
print("    can match or beat geometric if KR≥1200 is reached stably on hardware.")
