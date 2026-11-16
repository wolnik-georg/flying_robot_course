#!/usr/bin/env python3
"""Why was (c) sysId-then-tune bad in Stage 3 (P1: 63.75 cm)? Compares Stage 3 (dryrun/results) and Stage 3b (dryrun/stage3b/results):
fitted beta, tuned gains, model cost drop, and where the real error of the final gains sits (3-D vs z). Prints markdown; run: python3 dryrun/stage3b/c_diag.py P1"""
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent; OLD = HERE.parent / 'results'; NEW = HERE / 'results'
G = ['Pxy', 'Pz', 'Dxy', 'Dz', 'Iz', 'KRxy', 'KRz', 'Kwxy', 'Kwz', 'KIxy']; B = ['thrust gain', 'motor lag', 'downwash', 'plume']
TRUE = np.array([0.8, 1.5, 1.3, 0.7])
pn = sys.argv[1] if len(sys.argv) > 1 else 'P1'
print('### (c) sysId then tune, %s: Stage 3 against Stage 3b\n' % pn)
print('| item | Stage 3 | Stage 3b |\n|---|---|---|')
d = {}
for tag, R in (('3', OLD), ('3b', NEW)):
    st = json.load(open(R / ('full_%s_sysid_then_tune.json' % pn))); ev = json.load(open(R / ('full_%s_eval.json' % pn)))
    es = [e for e in ev['eval']['sysid_then_tune'] if e['role'] == 'tuning']; ok = [e for e in es if not e['crashed']]
    d[tag] = dict(
        beta='/'.join('%.2f' % v for v in np.exp(st['psi'])), err='/'.join('%+.0f' % (100 * v) for v in np.exp(st['psi']) / TRUE - 1),
        fit='/'.join('%d' % f['epochs'] for f in st['fit']), tune='/'.join('%d' % t['epochs'] for t in st['tune']),
        drop='%.4f -> %.5f' % (st['tune'][0]['log'][0]['J'], st['tune'][-1]['J_best'] if 'J_best' in st['tune'][-1] else st['tune'][-1]['log'][-1]['J']),
        gains=' '.join('%s %.2f' % (g, v) for g, v in zip(G, np.exp(st['phi_final']))),
        real3=np.mean([e['rms3d_cm'] for e in ok]), realz=np.mean([e['z_rms_cm'] for e in ok]), crashes=len(es) - len(ok))
for k, n in (('beta', 'fitted beta factors (thrust gain / lag / downwash / plume; true 0.80 / 1.50 / 1.30 / 0.70)'), ('err', 'beta error vs truth (%)'), ('fit', 'epochs of the 6 beta fit loops'),
             ('tune', 'epochs of the 6 theta tune loops'), ('drop', 'model task cost, first -> last tune loop'), ('gains', 'final gains (factors to the default)'),
             ('real3', 'real 3-D RMS on the tuning cells (cm)'), ('realz', 'real z RMS on the tuning cells (cm)'), ('crashes', 'crashed tuning flights')):
    f = (lambda v: '%.2f' % v) if k in ('real3', 'realz') else str
    print('| %s | %s | %s |' % (n, f(d['3'][k]), f(d['3b'][k])))
