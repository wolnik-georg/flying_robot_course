#!/usr/bin/env python3
"""Selection error and test error of a gain set, exactly as in the tuning (see frsim/evaluation.py).

  python3 scripts/evaluate_gains.py --ctrl S0 --scen A8 --gains tuned [--procs 4]
selection error = tuning cells x 9 robustness variants x noise seeds 0 - 2 (crashed episode = 100 cm); test error = study cells x 5 variants x noise seeds 0 - 4.
With --gains tuned the recorded values of gains/tuned/<ctrl>.yaml are printed next to the new ones.
"""
import argparse
import sys

import _common
from frsim import controllers, evaluation
from frsim.parallel import wait_for_resources


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ctrl', required=True, choices=list(controllers.CONTROLLERS))
    ap.add_argument('--scen', required=True, choices=['A1', 'A4', 'A8'])
    ap.add_argument('--gains', default='default')
    ap.add_argument('--procs', type=int, default=4)
    ap.add_argument('--no-wait', action='store_true')
    ap.add_argument('--skip-test', action='store_true', help='selection error only')
    a = ap.parse_args()
    gains, text = _common.load_gains(a.ctrl, a.scen, a.gains)
    if not a.no_wait:
        wait_for_resources(a.procs)
    print('%s on %s, %s' % (a.ctrl, a.scen, text))
    sel, crashes = evaluation.selection_error(a.ctrl, a.scen, gains, a.procs)
    print('selection error: %r cm  (%d crashed episodes)' % (sel, crashes))
    recorded = None
    if a.gains == 'tuned':
        import yaml
        recorded = yaml.safe_load(open(_common.SIMULATOR / 'gains/tuned' / ('%s.yaml' % a.ctrl)))[a.scen]
        print('recorded:        %r cm  (%d crashed episodes)  -> %s' % (recorded['selection_error_cm'], recorded['crashed_selection_episodes'],
                                                                        'IDENTICAL' if (sel == recorded['selection_error_cm'] and crashes == recorded['crashed_selection_episodes']) else 'DIFFERENT'))
    if not a.skip_test:
        err, per = evaluation.test_error(a.ctrl, a.scen, gains, a.procs)
        print('test error (cm): ' + ', '.join('%s %.3f' % (k, v) for k, v in err.items()))
        if recorded:
            print('recorded:        ' + ', '.join('%s %.3f' % (k, v) for k, v in recorded['test_error_cm'].items()) + '  (3 decimals)')


if __name__ == '__main__':
    sys.exit(main())
