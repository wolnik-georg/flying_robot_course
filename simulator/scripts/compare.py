#!/usr/bin/env python3
"""Compare controllers on one scenario: error table over several noise seeds, and an overlay plot of one flight each.

  python3 scripts/compare.py --scen A8 --ctrls S0 S1 S2 S3 FBL --gains default --variant nominal --seeds 0 1 2 --out examples/compare_A8
--gains tuned uses gains/tuned/<ctrl>.yaml where it exists (S0 - S3) and the default gains otherwise.
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np

import _common
from frsim import controllers, metrics, variants
from frsim.episode import EpisodeConfig
from frsim.parallel import run_many, wait_for_resources
from frsim.scenarios import study_cells


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--scen', required=True, choices=['A1', 'A4', 'A8'])
    ap.add_argument('--ctrls', nargs='+', default=list(controllers.CONTROLLERS))
    ap.add_argument('--gains', default='default', choices=['default', 'tuned'])
    ap.add_argument('--variant', default='nominal', choices=list(variants.VARIANTS))
    ap.add_argument('--delay-ms', type=int, default=None, help='extra state delay in ms (overrides --variant)')
    ap.add_argument('--seeds', type=int, nargs='+', default=[0, 1, 2])
    ap.add_argument('--procs', type=int, default=4)
    ap.add_argument('--out', default=None, help='output directory for the table (CSV) and plot')
    ap.add_argument('--no-wait', action='store_true')
    a = ap.parse_args()
    variant = variants.with_delay(a.delay_ms) if a.delay_ms is not None else variants.get(a.variant)
    cells = study_cells(a.scen)
    configs, keys, notes = [], [], {}
    for c in a.ctrls:
        try:
            gains, text = _common.load_gains(c, a.scen, a.gains)
        except SystemExit:
            gains, text = None, 'default gains (no tuned set for %s)' % c
        notes[c] = text
        for seed in a.seeds:
            for k, cell in enumerate(cells):
                configs.append(EpisodeConfig(c, a.scen, cell, gains, variant, seed, stop_on_crash=False))
                keys.append((c, seed, k))
    if not a.no_wait:
        wait_for_resources(a.procs)
    results = run_many(configs, a.procs)
    print('%s, variant %s (%s), study cells, noise seeds %s' % (a.scen, variant.name, variant.description, a.seeds))
    print('%-5s %-44s %12s %10s %9s' % ('ctrl', 'gains', 'error cm', 'max tilt', 'crashed'))
    table = []
    for c in a.ctrls:
        sel = [(r, k) for r, k in zip(results, keys) if k[0] == c]
        err = float(np.mean([metrics.episode_error(r) for r, _ in sel]))
        tilt = max(r.tilt_max_deg for r, _ in sel)
        crashed = sum(not r.stable for r, _ in sel)
        print('%-5s %-44s %12.2f %8.1f deg %5d / %d' % (c, notes[c][:44], err, tilt, crashed, len(sel)))
        table.append((c, notes[c], err, tilt, crashed, len(sel)))
    if a.out:
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        with open(out / ('compare_%s_%s.csv' % (a.scen, variant.name)), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['controller', 'gains', 'error_cm', 'max_tilt_deg', 'crashed_episodes', 'episodes'])
            w.writerows(table)
        traces = []
        for c in a.ctrls:
            r = [res for res, k in zip(results, keys) if k == (c, a.seeds[0], 0)][0]
            p = out / ('trace_%s_%s_%s.csv' % (c, a.scen, variant.name))
            _common.write_trace(p, r.trace)
            traces.append(str(p))
        png = _common.try_plot(traces, out / ('compare_%s_%s.png' % (a.scen, variant.name)), '%s, %s, seed %d' % (a.scen, variant.name, a.seeds[0]), labels=list(a.ctrls))
        if png:
            print('plot:', png)


if __name__ == '__main__':
    sys.exit(main())
