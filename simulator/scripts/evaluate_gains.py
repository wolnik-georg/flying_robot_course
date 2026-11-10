#!/usr/bin/env python3
"""Selection error and test error of a gain set, exactly as in the tuning (see frsim/evaluation.py).

  python3 scripts/evaluate_gains.py --ctrl S0 --scen A8 --gains tuned [--procs 4]
selection error = tuning cells x 9 robustness variants x noise seeds 0 - 2 (crashed episode = 100 cm);
test error = study cells x 5 variants x noise seeds 0 - 4.
With --gains tuned the recorded values of gains/tuned/<ctrl>.yaml are printed next to the new ones.
Results are saved automatically in simulator/results/<date>_<time>_evaluate_gains_.../ (episodes.csv, summary.json,
summary.md); --out DIR chooses the folder, --no-save only prints.
"""

import argparse
import sys

import _common
from frsim import controllers, evaluation, results
from frsim.parallel import wait_for_resources

EPISODE_COLUMNS = [
    'stage',
    'cell_index',
    'variant',
    'seed',
    'x_rms_cm',
    'y_rms_cm',
    'z_rms_cm',
    'rms3d_cm',
    'max_tilt_deg',
    'crashed',
    'episode_error_cm',
]


def parse_args():
    """Command-line arguments."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ctrl', required=True, choices=list(controllers.CONTROLLERS))
    ap.add_argument('--scen', required=True, choices=['A1', 'A4', 'A8'])
    ap.add_argument('--gains', default='default')
    ap.add_argument('--procs', type=int, default=4)
    ap.add_argument('--out', default=None, help='output folder (default: simulator/results/<date>_<time>_...)')
    ap.add_argument('--no-save', action='store_true', help='print only, write no files')
    ap.add_argument('--no-wait', action='store_true')
    ap.add_argument('--skip-test', action='store_true', help='selection error only')
    return ap.parse_args()


def recorded_values(a):
    """Recorded entry of gains/tuned/<ctrl>.yaml for the scenario (None unless --gains tuned)."""
    if a.gains != 'tuned':
        return None
    import yaml

    path = _common.SIMULATOR / 'gains' / 'tuned' / ('%s.yaml' % a.ctrl)
    return yaml.safe_load(open(path))[a.scen]


def main():
    """Evaluate the gains, print the errors, save the results folder."""
    a = parse_args()
    gains, text = _common.load_gains(a.ctrl, a.scen, a.gains)
    if not a.no_wait:
        wait_for_resources(a.procs)
    folder = results.output_dir('evaluate_gains', [a.ctrl], a.scen, None, a.out, not a.no_save)
    print('%s on %s, %s' % (a.ctrl, a.scen, text))
    ev = evaluation.evaluate(a.ctrl, a.scen, gains, a.procs, include_test=not a.skip_test)
    sel, crashes = ev['selection_error_cm'], ev['crashed_selection_episodes']
    lines = ['selection error: %r cm  (%d crashed episodes)' % (sel, crashes)]
    rec = recorded_values(a)
    verdict = None
    if rec:
        same = sel == rec['selection_error_cm'] and crashes == rec['crashed_selection_episodes']
        verdict = 'IDENTICAL' if same else 'DIFFERENT'
        lines.append(
            'recorded:        %r cm  (%d crashed episodes)  -> %s'
            % (rec['selection_error_cm'], rec['crashed_selection_episodes'], verdict)
        )
    if ev['test_error_cm']:
        lines.append('test error (cm): ' + ', '.join('%s %.3f' % kv for kv in ev['test_error_cm'].items()))
        if rec:
            lines.append(
                'recorded:        '
                + ', '.join('%s %.3f' % kv for kv in rec['test_error_cm'].items())
                + '  (3 decimals)'
            )
    print('\n'.join(lines))
    if not folder:
        return
    rows = [[e[c] for c in EPISODE_COLUMNS] for e in ev['episodes']]
    _common.write_csv(folder / 'episodes.csv', EPISODE_COLUMNS, rows)
    summary = {
        'info': results.run_info(),
        'controller': a.ctrl,
        'scenario': a.scen,
        'gains_source': text,
        'gains': controllers.check_gains(controllers.get(a.ctrl), gains or {}),
        'selection_error_cm': sel,
        'crashed_selection_episodes': crashes,
        'test_error_cm': ev['test_error_cm'],
        'test_error_by_seed_cm': ev['test_error_by_seed_cm'],
        'recorded': rec,
        'verdict_vs_record': verdict,
        'episodes_csv': 'episodes.csv',
        'selection_cells': 'tuning cells x 9 variants x seeds 0-2',
        'test_cells': 'study cells x 5 variants x seeds 0-4',
    }
    md = (
        [
            '# Gain evaluation: %s on %s' % (a.ctrl, a.scen),
            '',
            '- gains: %s' % text,
            '- command: `%s`' % summary['info']['command'],
            '',
            '```',
        ]
        + lines
        + ['```', '', 'Episodes: [episodes.csv](episodes.csv) (%d rows), [summary.json](summary.json)' % len(rows), '']
    )
    results.write_summary(folder, summary, '\n'.join(md))
    print('results:', folder)


if __name__ == '__main__':
    sys.exit(main())
