#!/usr/bin/env python3
"""Run one controller on one scenario and print the tracking error.

  python3 scripts/run_scenario.py --ctrl S0 --scen A8 --gains tuned --delay-ms 40
Controllers: S0 (geometric), S1 (INDI), S2 (S0 + neural downwash compensation), S3 (S1 + compensation),
FBL (feedback linearisation). Scenarios: A1 (hover under the other vehicle), A4 (other vehicle passes next to it),
A8 (figure-eight under it).
Results are saved automatically in simulator/results/<date>_<time>_run_scenario_.../ (trace CSV, plot, summary.json,
summary.md); --out DIR chooses the folder, --no-save only prints.
"""

import argparse
import sys
from dataclasses import asdict

import _common
from frsim import controllers, metrics, results, variants
from frsim.episode import EpisodeConfig, run_episode
from frsim.scenarios import study_cells, tuning_cells


def parse_args():
    """Command-line arguments."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ctrl', required=True, choices=list(controllers.CONTROLLERS), help='controller')
    ap.add_argument('--scen', required=True, choices=['A1', 'A4', 'A8'], help='scenario')
    ap.add_argument('--gains', default='default', help="'default', 'tuned' (S0 - S3), or a YAML file with gain names")
    ap.add_argument(
        '--variant',
        default='nominal',
        choices=list(variants.VARIANTS),
        help='robustness variant (md = state delay in ms, ffs = feedforward scale, lag = motor lag)',
    )
    ap.add_argument('--delay-ms', type=int, default=None, help='extra state delay in ms (overrides --variant)')
    ap.add_argument('--seed', type=int, default=0, help='noise seed')
    ap.add_argument(
        '--cells',
        default='study',
        choices=['study', 'tuning'],
        help='which cells of the scenario to fly (default: the study cells)',
    )
    ap.add_argument('--full', action='store_true', help='do not stop at a crash (complete trace of a crashed flight)')
    ap.add_argument('--out', default=None, help='output folder (default: simulator/results/<date>_<time>_...)')
    ap.add_argument('--no-save', action='store_true', help='print only, write no files')
    ap.add_argument('--no-plot', action='store_true')
    return ap.parse_args()


def fly(a, gains, variant, cells):
    """Run all cells; returns a list of (cell index, cell settings, EpisodeResult)."""
    outs = []
    for k, cell in enumerate(cells):
        cfg = EpisodeConfig(a.ctrl, a.scen, cell, gains, variant, a.seed, stop_on_crash=not a.full)
        outs.append((k, cell, run_episode(cfg)))
    return outs


def main():
    """Fly the cells, print the table, save the results folder."""
    a = parse_args()
    gains, gains_text = _common.load_gains(a.ctrl, a.scen, a.gains)
    variant = variants.with_delay(a.delay_ms) if a.delay_ms is not None else variants.get(a.variant)
    cells = study_cells(a.scen) if a.cells == 'study' else tuning_cells(a.scen)
    ctrl = controllers.get(a.ctrl)
    print(
        '%s on %s, %s, variant: %s (%s), noise seed %d'
        % (a.ctrl, a.scen, gains_text, variant.name, variant.description, a.seed)
    )
    folder = results.output_dir('run_scenario', [a.ctrl], a.scen, variant.name, a.out, not a.no_save)
    rows, records = [], []
    for k, cell, res in fly(a, gains, variant, cells):
        crashed = not res.stable
        shown = res.stable or a.full
        state = 'ok'
        if crashed:
            state = 'CRASHED' + (' at %.2f s' % res.crash_time_s if res.crash_time_s is not None else '')
        rec = {
            'cell_index': k,
            'cell': cell,
            'x_rms_cm': res.x_rms_cm if shown else None,
            'y_rms_cm': res.y_rms_cm if shown else None,
            'z_rms_cm': res.z_rms_cm if shown else None,
            'rms3d_cm': metrics.rms3d(res) if shown else None,
            'max_tilt_deg': res.tilt_max_deg,
            'crashed': crashed,
            'crash_time_s': res.crash_time_s,
            'trace_csv': None,
            'plot': None,
        }
        if folder:
            path = folder / ('trace_%s_%s_%s_cell%d.csv' % (a.ctrl, a.scen, variant.name, k))
            _common.write_trace(path, res.trace)
            rec['trace_csv'] = path.name
            if not a.no_plot:
                title = '%s on %s (%s)' % (a.ctrl, a.scen, variant.name)
                png = _common.try_plot([path], path.with_suffix('.png'), title=title)
                rec['plot'] = png.name if png else None
        records.append(rec)
        errs = tuple(rec[key] for key in ('x_rms_cm', 'y_rms_cm', 'z_rms_cm', 'rms3d_cm'))
        rows.append(
            (
                k,
                ', '.join('%s=%s' % kv for kv in cell.items()),
                *(('%.2f' % e) if e is not None else '-' for e in errs),
                '%.1f' % res.tilt_max_deg,
                state,
            )
        )
    header = ['cell', 'settings', 'x RMS cm', 'y RMS cm', 'z RMS cm', '3-D cm', 'max tilt deg', 'result']
    print('%-4s %-52s %9s %9s %9s %8s %9s  %s' % tuple(header))
    for r in rows:
        print('%-4s %-52s %9s %9s %9s %8s %9s  %s' % r)
    if folder:
        summary = {
            'info': results.run_info(variant),
            'controller': a.ctrl,
            'scenario': a.scen,
            'cells': a.cells,
            'variant': asdict(variant),
            'seed': a.seed,
            'stop_on_crash': not a.full,
            'gains': controllers.check_gains(ctrl, gains or {}),
            'gains_source': gains_text,
            'results': records,
        }
        md = [
            '# %s on %s' % (a.ctrl, a.scen),
            '',
            '- gains: %s' % gains_text,
            '- variant: %s (%s)' % (variant.name, variant.description),
            '- noise seed: %d, cells: %s' % (a.seed, a.cells),
            '- command: `%s`' % summary['info']['command'],
            '',
        ]
        md += _common.md_table(header, rows) + ['']
        md += [
            'Files: ' + ', '.join('[%s](%s)' % (n, n) for r in records for n in (r['trace_csv'], r['plot']) if n),
            '[summary.json](summary.json)',
            '',
        ]
        results.write_summary(folder, summary, '\n'.join(md))
        print('results:', folder)


if __name__ == '__main__':
    sys.exit(main())
