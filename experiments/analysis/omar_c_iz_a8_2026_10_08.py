#!/usr/bin/env python3
"""Omar C (controller 9) + Kpos_Iz on A8, 2026-10-08, cf5 uSD. kpos_iz inferred from flight order (meta does not record it)."""
import sys, json, numpy as np
from pathlib import Path
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "flying_drone_stack/tools")); sys.path.insert(0, str(REPO / "experiments/analysis"))
import decode_usd_log as d
import ns2_2026_10_05_crossing_dip as ns2
FL = [("18-40-18","thesis00",1.0),("18-42-05","thesis01",1.0),("18-45-11","thesis02",1.5),("18-47-48","thesis03",1.5),("18-49-37","thesis04",2.0),("18-51-15","thesis05",2.0)]
rows = []
hdr = "stamp  Iz | steady z err (excl crossings) cm | z sd | crossing dip abs (4) | dip rel to steady | roll/pitch p99 | gyro_x sd | z_err 0-4s after liftoff max | landing z_err min | motor max"
print(hdr)
for st, th, iz in FL:
    r = d.load(str(REPO / f"experiments/logs/usd_raw/cf5_A8_{th}_2026-10-08_{st}.bin"))
    t = r["t"] - r["t"][0]; z = r["z"]; sp = r["ctrltarget_z"]; ez = (z - sp) * 100
    up = z > 0.3; t0 = t[up][0]; t1 = t[up][-1]
    cross = ns2.find_crossing_times(t[up], r["y"][up])
    scen = up & (t > t0 + 6) & (t < t1 - 4)
    nocross = scen.copy()
    for tc in cross: nocross &= ~((t > tc - 1.5) & (t < tc + 1.5))
    steady = float(np.mean(ez[nocross]))
    dips = [float(np.min(ez[(t >= tc - 1) & (t <= tc + 1)])) for tc in cross]
    early = float(np.max(np.abs(ez[up & (t < t0 + 4)])))
    late = up & (t > t1 - 3)
    row = dict(stamp=st, iz=iz, steady_cm=steady, z_sd_cm=float(np.std(ez[nocross])), dips=dips, dips_rel=[x - steady for x in dips],
               roll_p99=float(np.percentile(np.abs(r["roll_deg"][scen]), 99)), pitch_p99=float(np.percentile(np.abs(r["pitch_deg"][scen]), 99)),
               gyro_x_sd=float(r["gyro_x"][scen].std()), early_max_abs=early, landing_min=float(np.min(ez[late])) if late.any() else float("nan"),
               motor_max=float(max(np.nanmax(r[f"motor_m{i}"]) for i in range(1,5))))
    rows.append(row)
    print(st, iz, "| %+.1f | %.1f | %s | %s | %.0f/%.0f | %.0f | %.1f | %.1f | %.0f" % (steady, row["z_sd_cm"], np.round(dips,1).tolist(), np.round(row["dips_rel"],1).tolist(), row["roll_p99"], row["pitch_p99"], row["gyro_x_sd"], early, row["landing_min"], row["motor_max"]))
print("\nper value (2 flights each):")
for iz in (1.0, 1.5, 2.0):
    s = [x for x in rows if x["iz"] == iz]
    print(iz, "steady %+.2f cm | dip abs %.1f | dip rel %.1f | z_sd %.2f | gyro_x sd %.0f | roll p99 %.0f" % (np.mean([x["steady_cm"] for x in s]), np.mean(sum([x["dips"] for x in s], [])), np.mean(sum([x["dips_rel"] for x in s], [])), np.mean([x["z_sd_cm"] for x in s]), np.mean([x["gyro_x_sd"] for x in s]), np.mean([x["roll_p99"] for x in s])))
Path(REPO / "experiments/analysis/out/omar_c_iz_2026-10-08").mkdir(parents=True, exist_ok=True)
json.dump(rows, open(REPO / "experiments/analysis/out/omar_c_iz_2026-10-08/a8_summary.json", "w"), indent=1)
