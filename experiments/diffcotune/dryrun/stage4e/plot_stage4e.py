"""Stage 4e figure: real cost per round (B1, T1, T2) and T0 beta errors per variant. Data: results/*.json."""
import json
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
H = Path(__file__).resolve().parent; R = H / 'results'
J = lambda p: json.load(open(p))
b1 = J(H.parent / 'stage3e' / 'results' / 'loop_P1.json')['rounds'][:5]; t1 = J(R / 'loop_T1.json')['rounds']; t2 = J(R / 'loop_T2_sg210.json')['rounds']
t0 = J(R / 't0_identifiability.json')
fig, ax = plt.subplots(1, 2, figsize=(12, 4.4), gridspec_kw={'width_ratios': [1, 1.25]})
C = {'B1': '#0072B2', 'T1': '#D55E00', 'T2': '#009E73'}
for lab, rd, mk in (('B1 (reference, 50 epochs, best iterate, p v q)', b1, 'o'), ('T1 paper stop rule', t1, 's'), ('T2 body rates, SG 210 ms', t2, '^')):
    k = lab[:2]; ax[0].plot(range(len(rd)), [r['real']['rms3d_cm'] for r in rd], marker=mk, color=C[k], label=lab, lw=2)
    for l, r in enumerate(rd):
        if any(f['crashed'] or f['floor'] for f in r['flights']): ax[0].plot(l, r['real']['rms3d_cm'], marker='x', color='k', ms=11, mew=2, zorder=5)
ax[0].set_yscale('log'); ax[0].set_xticks(range(5)); ax[0].set_xlabel('round l (gain set flown)'); ax[0].set_ylabel('real 3-D RMS, mean of 4 tuning flights [cm]'); ax[0].set_title('Real cost per round (x = a flight failed: tilt or floor)'); ax[0].legend(fontsize=8, loc='upper left'); ax[0].grid(alpha=.3)
names = ['thrust gain', 'motor lag', 'downwash', 'plume']; w = 0.16; cols = ['#0072B2', '#E69F00', '#CC79A7', '#009E73', '#56B4E9']
for i, r in enumerate(t0):
    ax[1].bar(np.arange(4) + (i - 2) * w, r['err_pct'], w, color=cols[i], label=r['variant'])
ax[1].axhline(0, color='k', lw=.8); ax[1].set_xticks(range(4)); ax[1].set_xticklabels(names); ax[1].set_ylabel('fitted beta error vs truth [%]'); ax[1].set_title('T0: beta after 50 epochs (fit only, P1, theta_0 flights)'); ax[1].legend(fontsize=8, ncol=2); ax[1].grid(alpha=.3, axis='y')
plt.tight_layout(); plt.savefig(H / 'stage4e_results.png', dpi=130)
