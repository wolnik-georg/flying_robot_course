#!/usr/bin/env python3
"""Check a simulated formation flight against what the scenario commanded.

    verify_formation_sim.py <meta.json> <record_states_csv_dir> [--tol-z 0.05] [--append FILE]

Why this exists separately from the runner's own report: in simulation the radio log
topics `DroneLogger` subscribes to do not exist, and `/pose` is published by the hardware
C++ server, not the simulated one. The simulator's ground truth is the `record_states`
visualisation output, which is written when the server shuts down -- after the client has
already exited. So verification has to happen afterwards, from the files.

The runner writes a sidecar JSON giving the exact simulation-clock instant the trajectory
started. That matters: for a pure-hover scenario like A1 nothing moves, so the window
cannot be recovered from the recorded states by any heuristic.

Reports, per robot pair, commanded vs realised dx/dy/dz -- mean, std, max error and RMSE --
and a PASS/FAIL against a stated tolerance.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

# The scenario library is ROS-free, so it imports fine from here.
sys.path.insert(0, '/home/georg/Desktop/crazyswarm2/crazyflie_examples')
from crazyflie_examples.formations import scenarios as S  # noqa: E402

# record_states columns: timestamp,x,y,z,qw,qx,qy,qz
T, X, Y, Z, QW, QX, QY, QZ = range(8)


def load_states(csv_dir: Path, names: list[str]):
    out = []
    for n in names:
        f = Path(csv_dir) / f'{n}.csv'
        if not f.exists():
            raise SystemExit(f'missing recorded states for {n}: {f}')
        out.append(np.loadtxt(f, delimiter=',', skiprows=1, ndmin=2))
    return out


def resample(d, grid):
    """Interpolate one robot's recorded states onto a common time grid."""
    return np.stack([np.interp(grid, d[:, T], d[:, c]) for c in (X, Y, Z)], axis=1)


def tilt_deg(d, grid):
    q = np.stack([np.interp(grid, d[:, T], d[:, c]) for c in (QW, QX, QY, QZ)], axis=1)
    return np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))


def verify(meta_path: Path, csv_dir: Path, tol_z: float, tol_xy: float):
    meta = json.load(open(meta_path))
    sc = S.build(meta['scenario'], **meta['params'])
    names = meta['names']
    t0 = float(meta['t_start_sim'])
    dur = float(meta['duration']) * float(meta.get('timescale', 1.0))

    data = load_states(csv_dir, names)
    t_end_file = min(d[-1, T] for d in data)
    # The server is killed on a timeout, so a run can be cut short. Verify whatever
    # window actually exists and say so, rather than silently comparing against nothing.
    t_end = min(t0 + dur, t_end_file)
    covered = (t_end - t0) / dur if dur > 0 else 0.0
    if t_end - t0 < 1.0:
        return dict(ok=False, reason=f'recording ends at {t_end_file:.1f}s, '
                                     f'trajectory starts at {t0:.1f}s -- no overlap',
                    covered=covered, rows=[])

    grid = np.linspace(t0, t_end, max(50, int((t_end - t0) * 50)))
    pos = [resample(d, grid) for d in data]
    tilts = [tilt_deg(d, grid) for d in data]

    rows, ok = [], True
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            want = np.array([sc.relative(i, j, t - t0) for t in grid])
            got = pos[i] - pos[j]
            err = got - want
            e_z = np.abs(err[:, 2])
            e_xy = np.linalg.norm(err[:, :2], axis=1)
            rmse = float(np.sqrt(np.mean(np.sum(err ** 2, axis=1))))
            pair_ok = e_z.mean() < tol_z and e_xy.mean() < tol_xy
            ok &= pair_ok
            rows.append(dict(
                pair=f'{sc.robots[i].role}-{sc.robots[j].role}',
                dz_cmd=float(want[:, 2].mean()), dz_got=float(got[:, 2].mean()),
                dz_std=float(got[:, 2].std()),
                dx_cmd=float(want[:, 0].mean()), dx_got=float(got[:, 0].mean()),
                dy_cmd=float(want[:, 1].mean()), dy_got=float(got[:, 1].mean()),
                mean_ez=float(e_z.mean()), max_ez=float(e_z.max()),
                mean_exy=float(e_xy.mean()), rmse=rmse, ok=pair_ok))

    max_tilt = float(max(t.max() for t in tilts))
    # A vehicle past 60 degrees on these gentle scenarios is not tracking, it is falling.
    diverged = max_tilt > 60.0
    if diverged:
        ok = False
    return dict(ok=ok, rows=rows, max_tilt=max_tilt, diverged=diverged,
                covered=covered, reason='' if ok else
                ('diverged' if diverged else 'geometry out of tolerance'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('meta')
    ap.add_argument('states_dir')
    ap.add_argument('--controller', default='?')
    ap.add_argument('--tol-z', type=float, default=0.05, help='mean |dz error| [m]')
    ap.add_argument('--tol-xy', type=float, default=0.08, help='mean horizontal error [m]')
    ap.add_argument('--append', default=None, help='append a one-line summary here')
    a = ap.parse_args()

    meta = json.load(open(a.meta))
    r = verify(Path(a.meta), Path(a.states_dir), a.tol_z, a.tol_xy)
    sid = meta['scenario']
    verdict = 'PASS' if r['ok'] else 'FAIL'

    print(f'\n  {sid}  controller={a.controller}  {verdict}'
          + (f"  ({r['reason']})" if r['reason'] else ''))
    print(f"    window covered {r['covered'] * 100:.0f}% of the trajectory"
          + (f", max tilt {r['max_tilt']:.1f} deg" if 'max_tilt' in r else ''))
    for row in r['rows']:
        print(f"    {row['pair']:>15}  dz {row['dz_got']:+.3f} (cmd {row['dz_cmd']:+.3f}, "
              f"sd {row['dz_std']:.3f})  mean|ez| {row['mean_ez'] * 1000:5.1f} mm  "
              f"max|ez| {row['max_ez'] * 1000:6.1f} mm  RMSE {row['rmse'] * 1000:6.1f} mm  "
              + ('ok' if row['ok'] else 'OUT OF TOL'))

    if a.append:
        worst = max((x['mean_ez'] for x in r['rows']), default=float('nan'))
        wr = max((x['rmse'] for x in r['rows']), default=float('nan'))
        params = ','.join(f'{k}={v}' for k, v in meta['params'].items()
                          if isinstance(v, (int, float)))
        with open(a.append, 'a') as fh:
            fh.write(f"| {sid} | {a.controller} | {len(meta['names'])} | {verdict} | "
                     f"{worst * 1000:.1f} | {wr * 1000:.1f} | "
                     f"{r.get('max_tilt', float('nan')):.1f} | "
                     f"{'yes' if r.get('diverged') else 'no'} | "
                     f"{r['covered'] * 100:.0f}% | {params} | `{a.states_dir}` |\n")
    return 0 if r['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
