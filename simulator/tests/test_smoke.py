#!/usr/bin/env python3
"""Smoke test: every controller x scenario runs end to end with the default gains (nominal plant, study cell, noise seed
0) and returns finite numbers.
A crashed flight is allowed (some default-gain flights crash, e.g. FBL on A4); it must still return a result and a
trace.
  python3 tests/test_smoke.py [--procs 4] [--no-wait]      (also runs under pytest)
"""

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from frsim import controllers, metrics  # noqa: E402
from frsim.episode import EpisodeConfig  # noqa: E402
from frsim.parallel import run_many, wait_for_resources  # noqa: E402

SCENARIOS = ('A1', 'A4', 'A8')


def run(procs=2, wait=False):
    """Run every controller on every scenario; True if all runs return finite results."""
    keys = [(c, s) for c in controllers.CONTROLLERS for s in SCENARIOS]
    if wait:
        wait_for_resources(procs)
    results = run_many([EpisodeConfig(c, s, stop_on_crash=True) for c, s in keys], procs)
    ok = True
    for (c, s), r in zip(keys, results):
        finite = all(math.isfinite(v) for v in (r.x_rms_cm, r.y_rms_cm, r.z_rms_cm)) if r.stable else True
        good = finite and r.n_samples > 100 and r.trace is not None and r.trace.shape[1] == 11
        ok &= good
        print(
            '%-4s %-3s %s  error %6.2f cm  max tilt %5.1f deg  %s'
            % (
                c,
                s,
                'ok  ' if good else 'FAIL',
                metrics.episode_error(r),
                r.tilt_max_deg,
                'stable' if r.stable else 'crashed',
            )
        )
    return ok


def test_smoke():
    """pytest entry."""
    assert run(procs=2)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--procs', type=int, default=4)
    ap.add_argument('--no-wait', action='store_true')
    a = ap.parse_args()
    good = run(a.procs, not a.no_wait)
    print('RESULT:', 'all 15 controller x scenario runs OK' if good else 'FAILED')
    sys.exit(0 if good else 1)
