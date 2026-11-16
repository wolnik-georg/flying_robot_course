#!/usr/bin/env python3
"""Stage 6 mock helper: the 4 tuning flights of round l of one controller as fake merged uSD files (frsim flights of the dry-run plant with the gains of lab/<C>/round_<l>/gains.yaml, seeds 100 + 10 l + i, as the loop_lib dry run).
  python3 fake_round_flights.py --controller S2 --round 1 --state-dir lab/mock6/seq/S2 --out <folder>
Writes <scenario>_r<l>_merged_usd.csv (A1, A8, CIRCLE, FIG8 in that order, one second apart in modification time) into <folder>."""
import argparse, os, sys, time
from pathlib import Path
import numpy as np
if '--controller' in sys.argv: os.environ['DIFFCOTUNE_CONTROLLER'] = sys.argv[sys.argv.index('--controller') + 1]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'dryrun'))
import round as RD  # noqa
import loop_lib as L  # noqa
import fake_usd as F  # noqa
ap = argparse.ArgumentParser(); ap.add_argument('--controller', required=True); ap.add_argument('--round', type=int, required=True); ap.add_argument('--state-dir', required=True); ap.add_argument('--out', required=True); a = ap.parse_args()
phi = RD.yaml_to_phi(Path(a.state_dir) / ('round_%d' % a.round) / 'gains.yaml'); gains = L.gains_of(phi); plant = L.PLANTS['P1RD' if L.N_BETA == 5 else 'P1']; out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
LAB_ORDER = ['A1', 'A8', 'A4', 'CIRCLE', 'FIG8']          # lab flight order: modification times follow the card
for i, (s, c) in enumerate(L.TUNING):
    tr, stable = F.fly_500hz(s, c, gains, plant, 100 + 10 * a.round + i); p = out / ('%s_r%d_merged_usd.csv' % (s, a.round)); F.write_merged(p, tr, s, c); os.utime(p, (time.time() + LAB_ORDER.index(s), time.time() + LAB_ORDER.index(s))); print('wrote', p.name, 'stable', stable, flush=True)
