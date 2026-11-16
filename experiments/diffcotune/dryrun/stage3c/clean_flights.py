#!/usr/bin/env python3
"""Stage 3c Part B, addition to check 5: the gains theta_2 ... theta_5 of (d) (P1) flown in frsim with the sensor noise OFF and ON (loop seeds),
one process, no pool. Shows whether the failure of the later gain sets needs the noise. Usage: python3 dryrun/stage3c/clean_flights.py"""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import loop_lib as L  # noqa: E402

D = json.load(open(HERE.parent / 'stage3b' / 'results' / 'full_P1_cotune.json')); out = []
for l in range(6):
    gains = L.gains_of(np.array(D['rounds'][l]['phi']))
    for i, (s, c) in enumerate(L.TUNING):
        row = dict(round=l, scenario=s)
        for noise, fn in (('noise_off', L.worker_flyclean), ('noise_on', L.worker_fly)):
            tr, stable, tilt = fn((s, c, gains, L.P1, 10 * l + i))
            cst, cz = L.flight_cost(tr, L.R.build(s, c).scenario_s); m = tr[:, 0] > L.SCORE_AFTER_S
            row[noise] = dict(rms3d_cm=100 * cst ** .5, z_rms_cm=100 * cz ** .5, tilt=tilt, z_mean=float(tr[m, 3].mean()), z_ref_mean=float(tr[m, 16].mean()), z_min=float(tr[m, 3].min()))
        out.append(row)
        print(time.strftime('%H:%M:%S'), 'theta_%d %-7s noise off: 3-D %.1f z %.1f cm tilt %.0f z mean %.2f (ref %.2f) | noise on: 3-D %.1f z %.1f cm tilt %.0f z mean %.2f' % (
            l, s, row['noise_off']['rms3d_cm'], row['noise_off']['z_rms_cm'], row['noise_off']['tilt'], row['noise_off']['z_mean'], row['noise_off']['z_ref_mean'],
            row['noise_on']['rms3d_cm'], row['noise_on']['z_rms_cm'], row['noise_on']['tilt'], row['noise_on']['z_mean']), flush=True)
L.jdump(out, HERE / 'results' / 'check5_clean_flights.json')
