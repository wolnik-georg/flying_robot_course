#!/usr/bin/env python3
"""Run one controller on one scenario and print the tracking error.

  python3 scripts/run_scenario.py --ctrl S0 --scen A8 --gains tuned --delay-ms 40 --out examples/run_S0_A8
Controllers: S0 (geometric), S1 (INDI), S2 (S0 + neural downwash compensation), S3 (S1 + compensation), FBL (feedback linearisation).
Scenarios: A1 (hover under the other vehicle), A4 (other vehicle passes next to it), A8 (figure-eight under it).
Output: a summary per cell (RMS error in cm, maximum tilt, crashed or not), a trace CSV (error over time) and a plot if matplotlib works.
"""
import argparse
import sys

import _common  # noqa: F401  (path setup)
from frsim import controllers, metrics, variants
from frsim.episode import EpisodeConfig, run_episode
from frsim.scenarios import study_cells, tuning_cells


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ctrl', required=True, choices=list(controllers.CONTROLLERS), help='controller')
    ap.add_argument('--scen', required=True, choices=['A1', 'A4', 'A8'], help='scenario')
    ap.add_argument('--gains', default='default', help="'default', 'tuned' (S0 - S3), or a YAML file with gain names")
    ap.add_argument('--variant', default='nominal', choices=list(variants.VARIANTS), help='robustness variant (md = state delay in ms, ffs = feedforward scale, lag = motor lag)')
    ap.add_argument('--delay-ms', type=int, default=None, help='extra state delay in ms (overrides --variant)')
    ap.add_argument('--seed', type=int, default=0, help='noise seed')
    ap.add_argument('--cells', default='study', choices=['study', 'tuning'], help='which cells of the scenario to fly (default: the study cells)')
    ap.add_argument('--full', action='store_true', help='do not stop at a crash (complete trace of a crashed flight)')
    ap.add_argument('--out', default=None, help='output directory for trace CSV and plot')
    ap.add_argument('--no-plot', action='store_true')
    a = ap.parse_args()

    gains, gains_text = _common.load_gains(a.ctrl, a.scen, a.gains)
    variant = variants.with_delay(a.delay_ms) if a.delay_ms is not None else variants.get(a.variant)
    cells = study_cells(a.scen) if a.cells == 'study' else tuning_cells(a.scen)
    print('%s on %s, %s, variant: %s (%s), noise seed %d' % (a.ctrl, a.scen, gains_text, variant.name, variant.description, a.seed))
    print('%-5s %-48s %8s %8s %8s %8s %9s  %s' % ('cell', 'settings', 'x RMS cm', 'y RMS cm', 'z RMS cm', '3-D cm', 'max tilt', 'result'))
    outs = []
    for k, cell in enumerate(cells):
        res = run_episode(EpisodeConfig(a.ctrl, a.scen, cell, gains, variant, a.seed, stop_on_crash=not a.full))
        state = 'ok' if res.stable else 'CRASHED at %.2f s' % res.crash_time_s if res.crash_time_s is not None else 'CRASHED'
        err = '%8.2f %8.2f %8.2f %8.2f' % (res.x_rms_cm, res.y_rms_cm, res.z_rms_cm, metrics.rms3d(res)) if res.stable or a.full else '       -        -        -        -'
        print('%-5s %-48s %s %8.1f deg  %s' % (k, ', '.join('%s=%s' % kv for kv in cell.items()), err, res.tilt_max_deg, state))
        outs.append((k, cell, res))
    if a.out:
        from pathlib import Path
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        for k, cell, res in outs:
            path = out / ('trace_%s_%s_%s_cell%d.csv' % (a.ctrl, a.scen, variant.name, k))
            _common.write_trace(path, res.trace)
            print('trace:', path)
            if not a.no_plot:
                png = _common.try_plot([path], path.with_suffix('.png'), title='%s on %s (%s)' % (a.ctrl, a.scen, variant.name))
                if png:
                    print('plot: ', png)


if __name__ == '__main__':
    sys.exit(main())
