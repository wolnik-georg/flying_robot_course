#!/usr/bin/env python3
"""Stage 8d T1 (b): regression of the default path (DIFFCOTUNE_DRY_STRICT=0) after the loop_lib.py change: round 0 of S0 "ours" rerun (run_8.py --tag regress8s --stop-after 1 --no-eval) against results/loop_S0_ours.json:
beta after the fit of round 0 (rounds[0].psi_after_fit) and theta_1 (rounds[1].phi of the old file = the best iterate of the tuning step of round 0 = tune.log[best_epoch].x of the new file). Writes results/regress8s_S0.json; exit 0 only if both differences are 0.0."""
import json, sys
from pathlib import Path
import numpy as np
R = Path(__file__).resolve().parent / 'results'
old = json.load(open(R / 'loop_S0_ours.json')); new = json.load(open(R / 'loop_S0_regress8s.json')); r = new['rounds'][0]
th1 = np.array(r['tune']['log'][r['tune']['best_epoch']]['x']); d_phi = float(np.abs(th1 - np.array(old['rounds'][1]['phi'])).max())
d_psi = float(np.abs(np.array(r['psi_after_fit']) - np.array(old['rounds'][0]['psi_after_fit'])).max()); d_real = abs(r['real']['rms3d_cm'] - old['rounds'][0]['real']['rms3d_cm'])
out = dict(theta1_max_abs_diff=d_phi, psi_after_fit_max_abs_diff=d_psi, real_cm_diff=d_real, passed=bool(d_phi == 0.0 and d_psi == 0.0 and d_real == 0.0), reference='results/loop_S0_ours.json', new='results/loop_S0_regress8s.json')
json.dump(out, open(R / 'regress8s_S0.json', 'w'), indent=1); print(json.dumps(out)); sys.exit(0 if out['passed'] else 1)
