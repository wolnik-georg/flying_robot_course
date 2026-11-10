#!/usr/bin/env python3
"""Equivalence gate: frsim reproduces all 199 episodes of tests/reference/reference_episodes.json (made once with the
tuning simulator and the old firmware builds).

Stable episodes: x / y / z RMS, 3-D RMS, maximum tilt and number of samples must be bit-identical. Crashed episodes:
crash flag and crash time must be equal
(the RMS of a crashed episode is not compared: episodes are stopped at the crash). Crash-border cells (any crashed
episode) are listed.

  python3 tests/test_equivalence.py [--quick] [--procs 4] [--groups S0 S1 ...] [--no-wait]
  (also runs under pytest: test_equivalence_quick)
Needs the firmware bindings (FRSIM_FIRMWARE_BUILD or the build_bindings.sh location).
"""

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from frsim import metrics, variants  # noqa: E402
from frsim.episode import EpisodeConfig  # noqa: E402
from frsim.parallel import run_many, wait_for_resources  # noqa: E402
from frsim.scenarios import tuning_cells  # noqa: E402

REFERENCE = HERE / 'reference' / 'reference_episodes.json'
GROUPS = ('S0', 'S1', 'S2', 'S3', 'FBL')


def to_config(meta):
    """EpisodeConfig of one reference episode."""
    if meta['kind'] == 'study':
        return EpisodeConfig(
            meta['group'],
            meta['scenario'],
            meta['cell'],
            meta['gains'],
            meta['variant'],
            meta['seed'],
            stop_on_crash=True,
        )
    cells = tuning_cells(
        meta['scenario']
    )  # search check: tuning cells x search variants, noise seed 0 (variant-major order)
    k = meta['index']
    return EpisodeConfig(
        meta['group'],
        meta['scenario'],
        cells[k % len(cells)],
        meta['gains'],
        variants.SEARCH_VARIANTS[k // len(cells)],
        0,
        stop_on_crash=True,
    )


def select_quick(items):
    """About 20 episodes: the first stable one of every group and scenario, plus every crashed episode of the
    reference."""
    chosen, seen = [], set()
    for k, (group, ep) in enumerate(items):
        key = (group, ep['meta']['scenario'], ep['meta']['kind'])
        if key not in seen or not ep['result']['stable']:
            seen.add(key)
            chosen.append((group, ep))
    return chosen[:24]


def compare(ref, got):
    """List of difference descriptions (empty = identical)."""
    diffs = []
    if ref['stable'] != got.stable:
        return ['stable flag: reference %s, frsim %s' % (ref['stable'], got.stable)]
    if ref['stable']:
        pairs = [
            ('x_rms_cm', got.x_rms_cm),
            ('y_rms_cm', got.y_rms_cm),
            ('z_rms_cm', got.z_rms_cm),
            ('rms3d_cm', metrics.rms3d(got)),
            ('tilt_max_deg', got.tilt_max_deg),
            ('n_samples', got.n_samples),
        ]
        diffs += [
            '%s: reference %r, frsim %r (|diff| %.3g)' % (k, ref[k], v, abs(ref[k] - v))
            for k, v in pairs
            if ref[k] != v
        ]
    elif ref['crash_time_s'] != got.crash_time_s:
        diffs.append('crash time: reference %r, frsim %r' % (ref['crash_time_s'], got.crash_time_s))
    return diffs


def run(quick=False, procs=4, groups=GROUPS, wait=True):
    """Run the reference episodes with frsim; returns (summary per group, failures)."""
    ref = json.load(open(REFERENCE))
    items = [(g, ep) for g in groups for ep in ref['groups'][g]['episodes']]
    if quick:
        items = select_quick(items)
    if wait:
        wait_for_resources(procs)
    results = run_many([to_config(ep['meta']) for _, ep in items], procs)
    summary = {g: dict(n=0, identical=0, max_abs=0.0, crashed=0, border=set()) for g in groups}
    failures = []
    for (g, ep), got in zip(items, results):
        s = summary[g]
        d = compare(ep['result'], got)
        s['n'] += 1
        s['identical'] += not d
        s['crashed'] += not ep['result']['stable']
        if not ep['result']['stable'] and ep['meta']['kind'] == 'study':
            s['border'].add(
                (ep['meta']['scenario'], ep['meta']['cell_index'], ep['meta']['gains_label'], ep['meta']['variant'])
            )
        if ep['result']['stable']:
            s['max_abs'] = max(
                s['max_abs'],
                max(
                    abs(ep['result'][k] - v)
                    for k, v in [
                        ('x_rms_cm', got.x_rms_cm),
                        ('y_rms_cm', got.y_rms_cm),
                        ('z_rms_cm', got.z_rms_cm),
                        ('tilt_max_deg', got.tilt_max_deg),
                    ]
                ),
            )
        if d:
            failures.append((g, ep['meta']['scenario'], ep['meta'].get('variant'), ep['meta'].get('seed'), d))
    return summary, failures


def report(summary, failures):
    """Print the summary; True if every episode is identical."""
    total = sum(s['n'] for s in summary.values())
    ok = sum(s['identical'] for s in summary.values())
    for g, s in summary.items():
        print(
            '%-4s %3d episodes, identical %3d, max |diff| of stable episodes %.3g, crashed in reference %d, '
            'crash-border cells %s'
            % (g, s['n'], s['identical'], s['max_abs'], s['crashed'], sorted(s['border']))
        )
    for f in failures[:20]:
        print('  DIFFERENT', f)
    print('RESULT: %d / %d episodes identical' % (ok, total))
    return ok == total


def test_equivalence_quick():
    """pytest entry: the quick subset must be identical."""
    assert report(*run(quick=True, procs=2, wait=False))


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument(
        '--quick', action='store_true', help='about 20 episodes (one per group and scenario plus all crashed ones)'
    )
    ap.add_argument('--procs', type=int, default=4)
    ap.add_argument('--groups', nargs='+', default=list(GROUPS))
    ap.add_argument('--no-wait', action='store_true', help='do not wait for free CPU / memory before starting')
    a = ap.parse_args()
    sys.exit(0 if report(*run(a.quick, a.procs, a.groups, not a.no_wait)) else 1)
