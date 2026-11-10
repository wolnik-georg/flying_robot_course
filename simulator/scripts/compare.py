#!/usr/bin/env python3
"""Compare controllers on one scenario: error table over several noise seeds, and an overlay plot of one flight each.

  python3 scripts/compare.py --scen A8 --ctrls S0 S1 S2 S3 FBL --gains default --variant nominal --seeds 0 1 2
--gains tuned uses gains/tuned/<ctrl>.yaml where it exists (S0 - S3) and the default gains otherwise.
Results are saved automatically in simulator/results/<date>_<time>_compare_.../ (table CSV, comparison plot, one
trace CSV per controller, summary.json, summary.md); --out DIR chooses the folder, --no-save only prints.
"""

import argparse
import sys
from dataclasses import asdict

import numpy as np

import _common
from frsim import controllers, metrics, results, variants
from frsim.episode import EpisodeConfig
from frsim.parallel import run_many, wait_for_resources
from frsim.scenarios import study_cells


def parse_args():
    """Command-line arguments."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--scen', required=True, choices=['A1', 'A4', 'A8'])
    ap.add_argument('--ctrls', nargs='+', default=list(controllers.CONTROLLERS))
    ap.add_argument('--gains', default='default', choices=['default', 'tuned'])
    ap.add_argument('--variant', default='nominal', choices=list(variants.VARIANTS))
    ap.add_argument('--delay-ms', type=int, default=None, help='extra state delay in ms (overrides --variant)')
    ap.add_argument('--seeds', type=int, nargs='+', default=[0, 1, 2])
    ap.add_argument('--procs', type=int, default=4)
    ap.add_argument('--out', default=None, help='output folder (default: simulator/results/<date>_<time>_...)')
    ap.add_argument('--no-save', action='store_true', help='print only, write no files')
    ap.add_argument('--no-wait', action='store_true')
    return ap.parse_args()


def build_configs(a, variant):
    """EpisodeConfigs, their keys (controller, seed, cell index) and the gains used per controller."""
    configs, keys, used = [], [], {}
    for c in a.ctrls:
        try:
            gains, text = _common.load_gains(c, a.scen, a.gains)
        except SystemExit:
            gains, text = None, 'default gains (no tuned set for %s)' % c
        used[c] = (gains, text)
        for seed in a.seeds:
            for k, cell in enumerate(study_cells(a.scen)):
                configs.append(EpisodeConfig(c, a.scen, cell, gains, variant, seed, stop_on_crash=False))
                keys.append((c, seed, k))
    return configs, keys, used


def main():
    """Run all controllers, print the table, save the results folder."""
    a = parse_args()
    variant = variants.with_delay(a.delay_ms) if a.delay_ms is not None else variants.get(a.variant)
    configs, keys, used = build_configs(a, variant)
    if not a.no_wait:
        wait_for_resources(a.procs)
    folder = results.output_dir('compare', a.ctrls, a.scen, variant.name, a.out, not a.no_save)
    out = run_many(configs, a.procs)
    print('%s, variant %s (%s), study cells, noise seeds %s' % (a.scen, variant.name, variant.description, a.seeds))
    header = ['controller', 'gains', 'error_cm', 'max_tilt_deg', 'crashed_episodes', 'episodes']
    table = []
    for c in a.ctrls:
        sel = [r for r, k in zip(out, keys) if k[0] == c]
        err = float(np.mean([metrics.episode_error(r) for r in sel]))
        table.append(
            (
                c,
                used[c][1],
                '%.2f' % err,
                '%.1f' % max(r.tilt_max_deg for r in sel),
                sum(not r.stable for r in sel),
                len(sel),
            )
        )
    print('%-5s %-44s %12s %10s %9s' % ('ctrl', 'gains', 'error cm', 'max tilt', 'crashed'))
    for t in table:
        print('%-5s %-44s %12s %8s deg %5d / %d' % (t[0], t[1][:44], t[2], t[3], t[4], t[5]))
    if not folder:
        return
    traces = []
    for c in a.ctrls:
        res = [r for r, k in zip(out, keys) if k == (c, a.seeds[0], 0)][0]
        path = folder / ('trace_%s_%s_%s.csv' % (c, a.scen, variant.name))
        _common.write_trace(path, res.trace)
        traces.append(path)
    table_csv = folder / ('compare_%s_%s.csv' % (a.scen, variant.name))
    _common.write_csv(table_csv, header, table)
    title = '%s, %s, seed %d' % (a.scen, variant.name, a.seeds[0])
    png = _common.try_plot(traces, folder / ('compare_%s_%s.png' % (a.scen, variant.name)), title, list(a.ctrls))
    summary = {
        'info': results.run_info(variant),
        'scenario': a.scen,
        'variant': asdict(variant),
        'seeds': a.seeds,
        'controllers': {
            c: {'gains': controllers.check_gains(controllers.get(c), used[c][0] or {}), 'gains_source': used[c][1]}
            for c in a.ctrls
        },
        'results': [dict(zip(header, t)) for t in table],
        'episodes': [
            {
                'controller': k[0],
                'seed': k[1],
                'cell_index': k[2],
                'stable': r.stable,
                'rms3d_cm': metrics.episode_error(r),
                'max_tilt_deg': r.tilt_max_deg,
            }
            for r, k in zip(out, keys)
        ],
    }
    md = [
        '# Comparison on %s' % a.scen,
        '',
        '- variant: %s (%s), study cells, noise seeds %s' % (variant.name, variant.description, a.seeds),
        '- command: `%s`' % summary['info']['command'],
        '',
    ]
    md += _common.md_table(header, table) + ['']
    if png:
        md += ['![comparison](%s)' % png.name, '']
    md += [
        'Files: ' + ', '.join('[%s](%s)' % (p.name, p.name) for p in [table_csv] + traces),
        '[summary.json](summary.json)',
        '',
    ]
    results.write_summary(folder, summary, '\n'.join(md))
    print('results:', folder)


if __name__ == '__main__':
    sys.exit(main())
