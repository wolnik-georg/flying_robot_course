#!/usr/bin/env python3
"""Compare two reference runs (directories with S0.json ... FBL.json written by make_reference.py run), episode by
episode.
  python3 compare_runs.py <dir A> <dir B> [group ...]
Stable episodes (stable in both): every RMS field, tilt and sample count compared; reports the maximum |difference| (0 =
bit-identical).
Crashed episodes: only the crash flag (stable) and the crash time are compared (the fast plants stop early, so their rms
fields are not valid).
Crash-border cells (cells where at least one episode crashed in either run, for the same cell / gains / variant) are
listed separately."""

import json, sys
from pathlib import Path

F = ['x_rms_cm', 'y_rms_cm', 'z_rms_cm', 'rms3d_cm', 'tilt_max_deg', 'n_samples']


def cmp_group(a, b):
    """Compare one group file of two runs; returns the counts and the list of differences."""
    A = json.load(open(a))['episodes']
    B = json.load(open(b))['episodes']
    assert len(A) == len(B)
    worst = 0.0
    nflip = 0
    ncrash_t = 0
    nstable = 0
    ncrashed = 0
    nbad = 0
    border = set()
    rows = []
    for x, y in zip(A, B):
        assert x['meta'] == y['meta'], 'episode lists differ'
        rx, ry = x['result'], y['result']
        m = x['meta']
        key = (m['scenario'], m.get('cell_index'), m.get('gains_label'), m.get('variant'), m['kind'], m.get('index'))
        if not rx['stable'] or not ry['stable']:
            border.add(key)
        if rx['stable'] != ry['stable']:
            nflip += 1
            rows.append(('FLIP', key, m.get('seed'), rx['stable'], ry['stable']))
            continue
        if rx['stable']:
            nstable += 1
            d = max(abs(rx[f] - ry[f]) for f in F)
            worst = max(worst, d)
            if d > 0:
                nbad += 1
                rows.append(('DIFF', key, m.get('seed'), d))
        else:
            ncrashed += 1
            if rx['crash_time_s'] != ry['crash_time_s']:
                ncrash_t += 1
                rows.append(('CRASHTIME', key, m.get('seed'), rx['crash_time_s'], ry['crash_time_s']))
    return dict(
        n=len(A),
        stable_both=nstable,
        crashed_both=ncrashed,
        flips=nflip,
        stable_with_diff=nbad,
        max_abs_diff=worst,
        crash_time_diffs=ncrash_t,
        border_cells=sorted(border, key=str),
        rows=rows,
    )


if __name__ == '__main__':
    da, db = Path(sys.argv[1]), Path(sys.argv[2])
    groups = sys.argv[3:] or ['S0', 'S1', 'S2', 'S3', 'FBL']
    for g in groups:
        r = cmp_group(da / f'{g}.json', db / f'{g}.json')
        print(
            '%-4s episodes %3d | stable in both %3d, crashed in both %2d | stable/crashed flips %d | '
            'stable episodes with any difference %d, max |diff| %.3g | crash-time diffs %d | identical: %s'
            % (
                g,
                r['n'],
                r['stable_both'],
                r['crashed_both'],
                r['flips'],
                r['stable_with_diff'],
                r['max_abs_diff'],
                r['crash_time_diffs'],
                'YES' if (r['flips'] == 0 and r['stable_with_diff'] == 0 and r['crash_time_diffs'] == 0) else 'NO',
            )
        )
        if r['border_cells']:
            print(
                '     crash-border cells (any crashed episode):',
                [(k[0], k[1], k[2], k[3]) for k in r['border_cells'] if k[4] == 'study'][:20],
            )
        for row in r['rows'][:20]:
            print('     ', row)
