#!/usr/bin/env bash
set -euo pipefail

# Sweep motor_time_constant (s) and thrust_rate_limit (N/s) for assignment5 figure8
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$ROOT/target/debug/assignment5"

# Ensure binary exists
if [ ! -x "$BIN" ]; then
    echo "Building assignment5 binary..."
    (cd "$ROOT" && cargo build --bin assignment5)
fi

mkdir -p "$ROOT/results/data"
SUMMARY="$ROOT/results/data/assignment5_sweep_summary.csv"
echo "tau,thrust_rate_limit,mean_acc,max_acc,mean_jerk,max_jerk,mean_pos_err,max_pos_err,planned_peak_jerk,notes" > "$SUMMARY"

# Parameter grids
taus=(0.01 0.02 0.03 0.05)
Rs=(5 10 20 40)

for tau in "${taus[@]}"; do
  for R in "${Rs[@]}"; do
    echo "Running tau=$tau, R=$R"
    # run assignment5: args -> mode speed_scale gain_scale thrust_max motor_tau thrust_rate_limit
    (cd "$ROOT" && "$BIN" figure8 1.0 1.0 1e6 "$tau" "$R")

    # Archive outputs
    mv -f "$ROOT/results/data/assignment5_planned_figure8.csv" "$ROOT/results/data/assignment5_planned_figure8_tau${tau}_R${R}.csv"
    mv -f "$ROOT/results/data/assignment5_closedloop_figure8.csv" "$ROOT/results/data/assignment5_closedloop_figure8_tau${tau}_R${R}.csv"

    # Compute metrics using Python
    python3 - <<PY
import csv, math
import numpy as np
path = "$ROOT/results/data/assignment5_closedloop_figure8_tau${tau}_R${R}.csv"
rows = []
with open(path,'r') as f:
    r = csv.DictReader(f)
    for row in r:
        rows.append(row)
if len(rows) < 2:
    print('not enough rows, skipping')
    mean_acc = max_acc = mean_jerk = max_jerk = mean_pos_err = max_pos_err = float('nan')
else:
    dt = float(rows[1]['t']) - float(rows[0]['t']) if len(rows)>1 else 0.005
    vx = np.array([float(r['sim_vx']) for r in rows])
    vy = np.array([float(r['sim_vy']) for r in rows])
    vz = np.array([float(r['sim_vz']) for r in rows])
    ax = np.concatenate(([0.0], np.diff(vx)/dt))
    ay = np.concatenate(([0.0], np.diff(vy)/dt))
    az = np.concatenate(([0.0], np.diff(vz)/dt))
    acc = np.sqrt(ax*ax+ay*ay+az*az)
    jx = np.concatenate(([0.0], np.diff(ax)/dt))
    jy = np.concatenate(([0.0], np.diff(ay)/dt))
    jz = np.concatenate(([0.0], np.diff(az)/dt))
    jerk = np.sqrt(jx*jx+jy*jy+jz*jz)
    mean_acc = float(np.nanmean(acc))
    max_acc = float(np.nanmax(acc))
    mean_jerk = float(np.nanmean(jerk))
    max_jerk = float(np.nanmax(jerk))
    # position error
    rx = np.array([float(r['ref_x']) for r in rows])
    ry = np.array([float(r['ref_y']) for r in rows])
    rz = np.array([float(r['ref_z']) for r in rows])
    sx = np.array([float(r['sim_x']) for r in rows])
    sy = np.array([float(r['sim_y']) for r in rows])
    sz = np.array([float(r['sim_z']) for r in rows])
    perr = np.sqrt((rx-sx)**2 + (ry-sy)**2 + (rz-sz)**2)
    mean_pos_err = float(np.nanmean(perr))
    max_pos_err = float(np.nanmax(perr))
    # planned peak jerk (from planned CSV)
    ppath = "$ROOT/results/data/assignment5_planned_figure8_tau${tau}_R${R}.csv"
    planned_peak = float('nan')
    try:
        prow = []
        with open(ppath,'r') as pf:
            pr = csv.DictReader(pf)
            for row in pr:
                prow.append(row)
        if len(prow) >= 2:
            pvx = np.array([float(r['vx']) for r in prow])
            pvy = np.array([float(r['vy']) for r in prow])
            pvz = np.array([float(r['vz']) for r in prow])
            pax = np.concatenate(([0.0], np.diff(pvx)/dt))
            pay = np.concatenate(([0.0], np.diff(pvy)/dt))
            paz = np.concatenate(([0.0], np.diff(pvz)/dt))
            pjx = np.concatenate(([0.0], np.diff(pax)/dt))
            pjy = np.concatenate(([0.0], np.diff(pay)/dt))
            pjz = np.concatenate(([0.0], np.diff(paz)/dt))
            pjerk = np.sqrt(pjx*pjx + pjy*pjy + pjz*pjz)
            planned_peak = float(np.nanmax(pjerk))
    except Exception:
        planned_peak = float('nan')

print(','.join(map(str,["$tau","$R",mean_acc,max_acc,mean_jerk,max_jerk,mean_pos_err,max_pos_err,planned_peak,"" ])))
PY
  done
done

echo "Sweep complete. Summary:"
cat "$SUMMARY"

