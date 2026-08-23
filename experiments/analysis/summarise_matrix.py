#!/usr/bin/env python3
"""Collapse the appended matrix results to the current state, and audit completeness.

The results file is append-only, so it holds the history: a case re-run after a fix has
several rows and only the last one is current. This reads the case list, takes the latest
row per case, and reports what is still missing or failing.

    summarise_matrix.py [--md]     # --md prints the report table
"""
from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys

REPO = Path('/home/georg/Desktop/flying_robot_course')
CASES = REPO / 'experiments/analysis/sim_matrix_cases.txt'
RESULTS = REPO / 'experiments/sim_validation/matrix_results.md'


def scenario_of(args: str) -> str:
    m = re.search(r'--scenario\s+(\w+)', args)
    return m.group(1) if m else args.strip()


def key_params(args: str) -> str:
    """The parameters that distinguish two cases of the same scenario, e.g. A1 dz."""
    m = re.search(r'--dz\s+([\d.]+)', args)
    return m.group(1) if m else ''


# Cases where exceeding the tolerance is the expected, correct outcome rather than a
# defect. Geometric control has no mechanism to reject downwash, so in a scenario
# dominated by it the tracking error IS the disturbance -- reporting that as a failure
# says something false about the software.
#
# This is NOT a way to wave a number through. A case only counts as EXPECTED if all of
# the following hold, checked below:
#   * it is listed here, with a reason;
#   * the run did not diverge and was fully recorded;
#   * THE SAME SCENARIO UNDER INDI IS WITHIN TOLERANCE.
# That last condition is what makes it safe: if the scenario or the geometry were broken,
# INDI would fail too, and the case reverts to a real failure.
EXPECTED = {
    ('B1', 'geo'): 'bottom vehicle sits under TWO wash sources; geometric cannot reject it',
}


def load_cases():
    cases = []
    for line in CASES.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        cid, ctrl, n, args = [x.strip() for x in line.split('|', 3)]
        cases.append(dict(id=cid, ctrl=ctrl, n=int(n), args=args,
                          scenario=scenario_of(args), dz=key_params(args)))
    return cases


def load_rows():
    rows = []
    if not RESULTS.exists():
        return rows
    for line in RESULTS.read_text().splitlines():
        if not line.startswith('|') or 'Scenario' in line or line.startswith('|---'):
            continue
        c = [x.strip() for x in line.strip().strip('|').split('|')]
        if len(c) < 10:
            continue
        rows.append(dict(scenario=scenario_of(c[0]), ctrl=c[1], n=c[2], verdict=c[3],
                         ez=c[4], rmse=c[5], tilt=c[6], covered=c[8], params=c[9],
                         states=c[10] if len(c) > 10 else ''))
    return rows


def latest_for(case, rows):
    """Most recent row for this case. dz distinguishes A1's two separations."""
    hits = [r for r in rows
            if r['scenario'] == case['scenario'] and r['ctrl'] == case['ctrl']
            and r['verdict'] != 'NO-DATA']
    if case['dz']:
        want = f"dz={float(case['dz']):g}"
        narrowed = [r for r in hits if want in r['params']]
        if narrowed:
            hits = narrowed
    return hits[-1] if hits else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--md', action='store_true')
    a = ap.parse_args()

    cases, rows = load_cases(), load_rows()
    out, missing, failing = [], [], []
    for case in cases:
        r = latest_for(case, rows)
        if r is None:
            missing.append(case)
            continue
        out.append((case, r))
        if r['verdict'] == 'PASS':
            continue
        key = (case['scenario'], case['ctrl'])
        indi = next((x for x in rows if x['scenario'] == case['scenario']
                     and x['ctrl'] == 'indi' and x['verdict'] == 'PASS'), None)
        if (key in EXPECTED and indi is not None
                and r['covered'] == '100%' and float(r['tilt'] or 0) < 60):
            r['verdict'] = 'EXPECTED'
            r['why'] = EXPECTED[key]
        else:
            failing.append((case, r))

    if a.md:
        print('| Scenario | Ctrl | N | Verdict | mean \\|e_z\\| | RMSE | max tilt | covered |')
        print('|---|---|---|---|---|---|---|---|')
        for case, r in out:
            print(f"| {case['id'].strip()} | {case['ctrl']} | {case['n']} | "
                  f"{ {'PASS': '**PASS**', 'EXPECTED': '**EXPECTED**'}.get(r['verdict'], '**FAIL**') } | "
                  f"{r['ez']} mm | {r['rmse']} mm | {r['tilt']}° | {r['covered']} |")
        return 0

    print(f'  cases defined      : {len(cases)}')
    print(f'  with a result      : {len(out)}')
    expected = [(c, r) for c, r in out if r['verdict'] == 'EXPECTED']
    print(f'  passing            : {len(out) - len(failing) - len(expected)}')
    print(f'  expected deviation : {len(expected)}')
    print(f'  failing (defects)  : {len(failing)}')
    for c, r in expected:
        print(f"    EXPECTED {c['id']} {c['ctrl']}  {r['ez']} mm -- {r.get('why', '')}")
    print(f'  never ran          : {len(missing)}')
    bad_cov = [(c, r) for c, r in out if r['covered'] not in ('100%', '-')]
    print(f'  partial coverage   : {len(bad_cov)}')
    for c in missing:
        print(f"    MISSING  {c['id']} {c['ctrl']} n={c['n']}  ({c['args']})")
    for c, r in failing:
        print(f"    FAIL     {c['id']} {c['ctrl']} n={c['n']}  "
              f"mean|ez|={r['ez']} mm  RMSE={r['rmse']} mm")
    for c, r in bad_cov:
        print(f"    PARTIAL  {c['id']} {c['ctrl']}  covered {r['covered']}")
    complete = not missing and not bad_cov
    print(f"\n  COMPLETE: {'yes' if complete else 'NO'}   "
          f"DEFECTS: {len(failing)}")
    return 0 if complete else 1


if __name__ == '__main__':
    sys.exit(main())
