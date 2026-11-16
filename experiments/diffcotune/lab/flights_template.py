#!/usr/bin/env python3
"""Stage 5 A1: flights.json skeleton of one round for round.py (file names to be filled in at the lab). Tuning flights in the order of the flight card (protocol draft; Stage 7: five flights): A1, A8, A4 (cell 1), CIRCLE, FIG8
(attempt 1); rotate_deg: CIRCLE -90 and FIG8 -90 (protocol draft: the circle's frame fit alone is degenerate, so rotate_deg must be given; FIG8 turned by the same angle), A1 / A8 none.
Test flights (A4 cell 1 and 2, 3 each, rounds 0 and the last round only): rotate_deg -90 (lab `--rotate 90`), role test.   python3 flights_template.py --out folder/flights.json [--test]"""
import argparse, json
from pathlib import Path

TUNING = [('A1', None), ('A8', None), ('A4', -90.0), ('CIRCLE', -90.0), ('FIG8', -90.0)]          # Stage 7: A4 cell 1 is a tuning flight (lab order: the two-drone flights first, then --solo); A4 rotate -90 (lab `--rotate 90`)


def template(test=False, n_test=3):
    m = [dict(file='<merged uSD file of %s>' % s, scenario=s, cell=0, role='tuning', attempt=1, rotate_deg=r, lag_s=None) for s, r in TUNING]
    if test:
        m += [dict(file='<merged uSD file of A4 cell %d, flight %d>' % (c + 1, k + 1), scenario='A4', cell=c, role='test', attempt=1, rotate_deg=-90.0, lag_s=None) for c in (0, 1) for k in range(n_test)]
    return m


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); ap.add_argument('--test', action='store_true'); a = ap.parse_args()
    Path(a.out).write_text(json.dumps(template(a.test), indent=1)); print('wrote', a.out)
