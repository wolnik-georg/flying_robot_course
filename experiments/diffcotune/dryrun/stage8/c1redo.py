#!/usr/bin/env python3
"""Stage 7 step 1c1 (redo): the S3 model with the CONTROLLER-side NS2 term forced to zero equals the S1 model (forward, A1, A8, CIRCLE, theta_0 and random gains, 3000 blocks). The first version (diag7_cd.py) patched NS2.faz, which also removes the
PLANT-side downwash of the model, so it compared different plants; here only S2.ns2_hold (the controller term) is replaced by zeros. Also the NS2-on difference to S1 for reference. DIFFCOTUNE_TUNING_SET=4 not needed. -> results/c1redo.json"""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent))
import torch
import loop_lib as L, reference as R  # noqa
import s1_model as S1, s2_model as S2  # noqa
torch.set_num_threads(2); dt = torch.float64
rec = json.load(open(HERE.parent / 'stage5c_rdfix' / 'results' / 'loop_S3.json')); psi = torch.tensor(np.array(rec['rounds'][0]['psi_after_fit'])[None], dtype=dt)
phi0 = torch.zeros(1, 10, dtype=dt); rng = np.random.default_rng(1); phi_r = torch.tensor(rng.uniform(-0.2, 0.2, (1, 10)), dtype=dt); orig = S2.ns2_hold; out = {}
cells = dict(L.TUNING)
for s in ('A1', 'A8', 'CIRCLE'):
    ref = R.build(s, cells[s])
    for nm, ph in (('theta0', phi0), ('gains_random', phi_r)):
        a = S1.rollout(ref, ph, psi, n_blocks=3000, ns2=False)
        S2.ns2_hold = lambda nets, sp, k, i, x, v, nzt, active, dtype, device: torch.zeros(x.shape[0], dtype=dtype)
        try: b = S1.rollout(ref, ph, psi, n_blocks=3000, ns2=True)
        finally: S2.ns2_hold = orig
        c = S1.rollout(ref, ph, psi, n_blocks=3000, ns2=True)
        d0 = max(float((a[k] - b[k]).abs().max()) for k in ('x', 'v', 'q')); d1 = max(float((a[k] - c[k]).abs().max()) for k in ('x', 'v'))
        out['%s/%s' % (s, nm)] = dict(ns2_controller_term_zero_vs_S1=d0, ns2_on_vs_S1=d1); print(time.strftime('%H:%M:%S'), s, nm, 'controller NS2 term zero vs S1: %.2e ; NS2 on vs S1: %.2e' % (d0, d1), flush=True)
json.dump(out, open(HERE / 'results' / 'c1redo.json', 'w'), indent=1)
