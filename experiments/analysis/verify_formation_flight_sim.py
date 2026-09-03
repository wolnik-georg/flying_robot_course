#!/usr/bin/env python3
"""Check a simulated formation_flight.py run against what it commanded.

    verify_formation_flight_sim.py <meta.json> <record_states_csv_dir> [--tol-xy 0.05] [--append FILE]

Separate from verify_formation_sim.py (the run_formation / scenario-library verifier)
because the commanded-geometry model is different. The scenario library flies
TIME-VARYING per-robot curves -- verifying it means reconstructing scenarios.py's curve
functions. formation_flight.py flies a RIGID formation: every drone gets the SAME
trajectory shape from a fixed offset, so the commanded relative geometry between any two
drones is CONSTANT for the whole flight (targets[a] - targets[b]). That's simpler to
verify and does not need the scenario library at all.

Same reason this exists separately from formation_flight.py's own report: in simulation
the radio log topics DroneLogger subscribes to do not exist, and /pose is published by
the hardware C++ server, not the simulated one. The simulator's ground truth is the
record_states visualisation output, written when the server shuts down -- after the
client has already exited. So verification has to happen afterwards, from the files.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

# record_states columns: timestamp,x,y,z,qw,qx,qy,qz
T, X, Y, Z, QW, QX, QY, QZ = range(8)

DEFAULT_TOL_XY = 0.05  # m


def load_states(csv_dir: Path, names: list[str]):
    out = []
    for n in names:
        f = Path(csv_dir) / f"{n}.csv"
        if not f.exists():
            raise SystemExit(f"missing recorded states for {n}: {f}")
        out.append(np.loadtxt(f, delimiter=",", skiprows=1, ndmin=2))
    return out


def resample(d, grid):
    return np.stack([np.interp(grid, d[:, T], d[:, c]) for c in (X, Y, Z)], axis=1)


def tilt_deg(d, grid):
    q = np.stack([np.interp(grid, d[:, T], d[:, c]) for c in (QW, QX, QY, QZ)], axis=1)
    return np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))


def verify(meta_path: Path, csv_dir: Path, tol_xy: float):
    meta = json.load(open(meta_path))
    names = meta["names"]
    targets = np.array(meta["targets"])  # (n, 3), constant commanded absolute position
    t0 = float(meta["t_start_sim"])
    dur = float(meta["duration"]) * float(meta.get("timescale", 1.0))

    data = load_states(csv_dir, names)
    t_end_file = min(d[-1, T] for d in data)
    t_end = min(t0 + dur, t_end_file)
    covered = (t_end - t0) / dur if dur > 0 else 0.0
    if t_end - t0 < 1.0:
        return dict(ok=False, reason=f"recording ends at {t_end_file:.1f}s, "
                                     f"trajectory starts at {t0:.1f}s -- no overlap",
                    covered=covered, rows=[])

    grid = np.linspace(t0, t_end, max(int((t_end - t0) * 20), 2))
    pos = [resample(d, grid) for d in data]
    tilts = [tilt_deg(d, grid) for d in data]
    max_tilt = float(max(t.max() for t in tilts))

    n = len(names)
    rows = []
    ok = True
    for a in range(n):
        for b in range(a + 1, n):
            cmd = targets[a] - targets[b]                 # constant, rigid formation
            realised = pos[a] - pos[b]                     # (T, 3) over the window
            err = realised - cmd[None, :]
            dz_mean = float(realised[:, 2].mean())
            err_mm = np.linalg.norm(err, axis=1) * 1000.0
            mean_err = float(err_mm.mean())
            max_err = float(err_mm.max())
            rmse = float(np.sqrt((err_mm ** 2).mean()))
            # Gate on MEAN error, matching verify_formation_sim.py's established convention
            # (its pair_ok = e_z.mean() < tol_z and e_xy.mean() < tol_xy) -- not max, which
            # trips on a brief boundary transient (trajectory start/end handoff) rather than
            # sustained tracking error. Max/RMSE are still reported for full transparency.
            pair_ok = mean_err < tol_xy * 1000.0
            ok = ok and pair_ok
            rows.append(dict(a=names[a], b=names[b], cmd=cmd.tolist(),
                              dz_realised=dz_mean, mean_err_mm=mean_err,
                              max_err_mm=max_err, rmse_mm=rmse, ok=pair_ok))

    return dict(ok=ok, covered=covered, max_tilt=max_tilt, rows=rows,
                formation=meta.get("formation", "?"), n=n)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("meta_json", type=Path)
    p.add_argument("csv_dir", type=Path)
    p.add_argument("--tol-xy", type=float, default=DEFAULT_TOL_XY)
    p.add_argument("--controller", default="?")
    p.add_argument("--append", type=Path, default=None)
    args = p.parse_args()

    r = verify(args.meta_json, args.csv_dir, args.tol_xy)
    verdict = "PASS" if r.get("ok") else "FAIL"
    print(f"  {r.get('formation', '?')}  controller={args.controller}  n={r.get('n', '?')}  {verdict}")
    if not r.get("rows"):
        print(f"    {r.get('reason', 'no data')}")
    else:
        print(f"    window covered {r['covered']*100:.0f}% of the trajectory, "
              f"max tilt {r['max_tilt']:.1f} deg")
        for row in r["rows"]:
            mark = "ok" if row["ok"] else "FAIL"
            print(f"      {row['a']}-{row['b']}  dz {row['dz_realised']:+.3f}  "
                  f"mean|e| {row['mean_err_mm']:6.1f} mm  max|e| {row['max_err_mm']:6.1f} mm  "
                  f"RMSE {row['rmse_mm']:6.1f} mm  {mark}")

    if args.append:
        with open(args.append, "a") as fh:
            for row in r.get("rows", [{}]):
                fh.write(
                    f"| {r.get('formation', '?')} | {args.controller} | {r.get('n', '?')} "
                    f"| {verdict} | {row.get('a', '-')}-{row.get('b', '-')} "
                    f"| {row.get('mean_err_mm', float('nan')):.1f} "
                    f"| {row.get('max_err_mm', float('nan')):.1f} "
                    f"| {row.get('rmse_mm', float('nan')):.1f} "
                    f"| {r.get('max_tilt', float('nan')):.1f} "
                    f"| {r.get('covered', 0)*100:.0f}% |\n"
                )

    raise SystemExit(0 if r.get("ok") else 1)


if __name__ == "__main__":
    main()
