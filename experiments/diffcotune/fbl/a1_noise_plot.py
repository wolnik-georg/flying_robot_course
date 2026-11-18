import json, numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from pathlib import Path
H = Path(__file__).resolve().parent; d = json.load(open(H / 'a1_noise_diag_traces.json'))
for plant in ('calibration', 'P1'):
    fig, ax = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    for seed, c in (('1000', 'C0'), ('1001', 'C3'), ('nonoise', 'k')):
        r = d['%s_%s' % (plant, seed)]; t = np.array(r['t']); w = np.array(r['w']); e = np.array(r['err'])
        ax[0].plot(t, r['tilt'], c, label=('seed ' + seed) if seed != 'nonoise' else 'no noise')
        ax[1].plot(t, np.degrees(np.linalg.norm(w[:, :2], axis=1)), c); ax[2].plot(t, r['thrust_N'], c); ax[3].plot(t, 100 * np.linalg.norm(e, axis=1), c)
    for a, y in zip(ax, ('tilt (deg)', 'roll/pitch rate (deg/s)', 'sum of motor forces (N)', '3-D position error (cm)')): a.set_ylabel(y); a.grid(alpha=.3)
    ax[0].axhline(45, color='gray', ls='--'); ax[0].legend(); ax[3].set_xlabel('scenario time (s)'); ax[0].set_title('frsim FBL base, A1, preset 1, plant %s: the simulated (bottom) drone; the top drone is a prescribed path' % plant, fontsize=9)
    ax[3].set_yscale('symlog', linthresh=1); fig.tight_layout(); fig.savefig(H / 'figs' / ('a1_noise_crash_%s.png' % plant), dpi=110)
