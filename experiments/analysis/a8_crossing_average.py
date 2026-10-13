#!/usr/bin/env python3
"""A8 crossing-averaged traces (2026-10-13 flights, cf5 bottom drone, uSD 500 Hz): the downwash residual every controller sees and the z error it ends up with.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/a8_crossing_average.py [--out file.png]
Residual = measured minus model vertical acceleration: logged `a_res_z` for geometric / NS2 / ours, Omar C: a_imu_fz − a_rpm_fz (his own log channels)."""
import glob, sys
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "flying_drone_stack" / "tools")); sys.path.insert(0, str(HERE))
import decode_usd_log as dec
from a8_crossings import zero_crossings

U = HERE.parents[1] / "experiments" / "logs" / "usd_raw"
GROUPS = [("geometric", ["geometric_1", "geometric_2"], "#555555"), ("geometric + NS2 v1", ["ns2_1", "ns2_2"], "#009E73"),
          ("Omar C + Iz 1.5", ["omarC_1", "omarC_2"], "#D55E00"), ("ours INDI", ["ours_2", "ours_3"], "#0072B2")]
grid = np.arange(-0.6, 1.51, 0.02)


def traces(key):
    Z, R = [], []
    for k in key:
        d = dec.load(glob.glob(str(U / f"cf5_A8_{k}_*.bin"))[0]); t = d["t"] - d["t"][0]
        res = d["omar_a_imu_fz"] - d["omar_a_rpm_fz"] if k.startswith("omarC") else d["a_res_z"]
        for c in zero_crossings(t, d["y"]):
            base = (t > c - 3) & (t < c - 1.5)
            Z.append(np.interp(c + grid, t, d["z"]) * 100 - 50.0)
            R.append(np.interp(c + grid, t, res) - np.median(res[base]))
    return np.mean(Z, 0), np.mean(R, 0), len(Z)


fig, ax = plt.subplots(2, 1, figsize=(10.5, 7), sharex=True)
for name, keys, col in GROUPS:
    z, r, n = traces(keys)
    ax[0].plot(grid, r, color=col, lw=1.6, label=f"{name} (n={n})")
    ax[1].plot(grid, z, color=col, lw=1.6, label=f"{name}: dip {z.min():.1f} cm")
ax[0].set_ylabel("residual, measured − model [m/s²]"); ax[0].set_title("A8 crossing, bottom drone, mean over crossings: every controller sees the same downwash pulse")
ax[1].set_ylabel("z error [cm]"); ax[1].set_xlabel("time relative to the crossing (s)")
ax[1].set_title("…and ends up with a different dip")
for x in ax:
    x.grid(alpha=0.3); x.legend(fontsize=9); x.axvline(0, color="k", lw=0.6, alpha=0.4)
fig.tight_layout()
out = Path(sys.argv[sys.argv.index("--out") + 1]) if "--out" in sys.argv else HERE / "out" / "a8_crossing_average_2026-10-13.png"
out.parent.mkdir(parents=True, exist_ok=True); fig.savefig(out, dpi=150); print("wrote", out)
