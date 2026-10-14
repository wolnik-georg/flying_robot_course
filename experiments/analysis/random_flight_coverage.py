#!/usr/bin/env python3
"""Coverage and safety statistics of the R1 random-waypoint scenario (NS2 training data, 2026-10-14).

For a range of seeds: safety gate as flown (auto-centred, height 0.5), fraction of time the pair is inside the NS2 gate
(|dx|,|dy| < 0.2 m, |dvx| < 1.5 m/s, neighbour above), distribution of dz and of the bottom vehicle's speed while in the gate.
Compares the stack bias 0.7 (default) with the unbiased sampling 0.0 (the IMRCLab script's behaviour).
  python3 experiments/analysis/random_flight_coverage.py [--seeds 40] [--total 40]"""
import argparse
import sys
from pathlib import Path

import numpy as np

CS2 = Path.home() / "Desktop" / "crazyswarm2" / "crazyflie_examples"
sys.path.insert(0, str(CS2))
from crazyflie_examples.formations import safety, scenarios  # noqa: E402
from crazyflie_examples.run_formation import centred_anchor  # noqa: E402


def stats(bias, seeds, total):
    lim = safety.Limits()
    bad, in_gate, dur, dzs, spd, vmax_all, amax_all, seps = [], 0.0, 0.0, [], [], 0.0, 0.0, []
    for sd in range(seeds):
        sc = scenarios.build("R1", seed=sd, total=total, bias=bias)
        base = centred_anchor(sc, lim, 0.5)
        prob = safety.check(sc, lim, base=base)
        if prob:
            bad.append((sd, prob))
        ts, pos = safety.sample_positions(sc, n=int(sc.duration * 50), base=base)
        dt = ts[1] - ts[0]
        d = pos[:, 1] - pos[:, 0]
        vel = np.gradient(pos, dt, axis=0)
        dv = vel[:, 1] - vel[:, 0]
        gate = (np.abs(d[:, 0]) < 0.2) & (np.abs(d[:, 1]) < 0.2) & (np.abs(dv[:, 0]) < 1.5) & (d[:, 2] > 0)
        in_gate += float(gate.sum()) * dt
        dur += float(sc.duration)
        dzs += list(d[gate, 2])
        spd += list(np.linalg.norm(vel[gate, 0], axis=1))
        v, a = max((r.curve.peaks() for r in sc.robots), key=lambda p: p[0])
        vmax_all, amax_all = max(vmax_all, v), max(amax_all, a)
        seps.append(safety.min_separation(sc, base=base)[0])
    dzs, spd = np.array(dzs), np.array(spd)
    q = lambda x: np.percentile(x, [5, 25, 50, 75, 95]).round(2).tolist() if len(x) else []
    return dict(bias=bias, flights=seeds, mean_duration_s=dur / seeds, in_gate_fraction=in_gate / dur, in_gate_s_per_flight=in_gate / seeds,
                dz_q=q(dzs), speed_q=q(spd), min_sep_overall=min(seps), peak_speed=vmax_all, peak_accel=amax_all, unsafe=bad)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--seeds", type=int, default=40); ap.add_argument("--total", type=float, default=40.0)
    a = ap.parse_args()
    for b in (0.7, 0.0):
        r = stats(b, a.seeds, a.total)
        print(f"bias {r['bias']:.1f}: {r['flights']} flights, mean {r['mean_duration_s']:.0f} s, in-gate {100 * r['in_gate_fraction']:.1f} % of the time "
              f"({r['in_gate_s_per_flight']:.1f} s/flight); dz in gate q5/25/50/75/95 {r['dz_q']}; bottom speed in gate {r['speed_q']}; "
              f"min separation {r['min_sep_overall']:.2f} m; peak v {r['peak_speed']:.2f} a {r['peak_accel']:.2f}; unsafe seeds {len(r['unsafe'])}")
        for sd, prob in r["unsafe"][:5]:
            print("   seed", sd, prob)


if __name__ == "__main__":
    main()
