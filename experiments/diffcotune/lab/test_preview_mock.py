#!/usr/bin/env python3
"""Stage 8b mock test of preview.py (single process, no pool): (1) the best flown set itself as the "new" set must give PREVIEW PASSED (no event the best set does not show); (2) a gain set that crashes or tilts in frsim must give PREVIEW FAILED naming the scenario and the reason.
Only the A8 scenario is flown (to keep it short). Usage: DIFFCOTUNE_CONTROLLER=S0 python3 test_preview_mock.py -> prints the verdicts, exit 0 when both are as expected."""
import sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'dryrun'))
import loop_lib as L, preview as PV, reference as R  # noqa
L.TUNING[:] = [t for t in L.TUNING if t[0] == 'A8']
psi = np.r_[L.PSI_TRUE]; phi0 = np.zeros(10); mapper = lambda fn, jobs: map(fn, jobs)
ok1 = PV.run(L, mapper, phi0, psi, phi0); print('1) same set as new and best:', ok1['verdict'], flush=True)
cands = [np.r_[np.zeros(5), 2.5, np.zeros(4)], np.r_[np.zeros(6), 2.5, np.zeros(3)], np.r_[-2.5, np.zeros(9)], np.r_[np.zeros(7), -2.5, np.zeros(2)], np.r_[np.zeros(3), 2.5, np.zeros(6)]]
bad = None
for ph in cands:
    tr, stable, tilt = L.worker_fly(('A8', L.TUNING[0][1], L.gains_of(ph), PV.fitted_plant(L, psi), 2000)); ev = PV.abort_events(L, tr, stable, R.build('A8', L.TUNING[0][1]).scenario_s)
    print('candidate', np.round(ph, 1).tolist(), 'events', ev, flush=True)
    if 'crash' in ev or 'tilt' in ev: bad = ph; break
if bad is None: print('no crashing candidate found'); sys.exit(2)
pv = PV.run(L, mapper, bad, psi, phi0); print('2) crashing set as new, default gains as best:\n' + PV.text(pv, L), flush=True)
sys.exit(0 if (ok1['passed'] and not pv['passed']) else 1)
