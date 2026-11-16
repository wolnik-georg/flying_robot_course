#!/usr/bin/env python3
"""Figures of the S0 lab protocol draft: (1) the three solo flight files inside the tape-measured volume with the margins, (2) A4 frame check (logged reference of 23 Oct against the frsim reference before / after the rotation)."""
import json, sys
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'flightfiles')); sys.path.insert(0, str(HERE.parent))
import make_flightfiles as M  # noqa
from frsim.scenarios import safety  # noqa

fig, ax = plt.subplots(figsize=(4.6, 7.0))
fs = safety.FLIGHT_SPACE
ax.add_patch(plt.Rectangle((fs['x'][0], fs['y'][0]), fs['x'][1] - fs['x'][0], fs['y'][1] - fs['y'][0], fill=False, ec='k', lw=1.5))
chk = json.load(open(HERE.parent / 'flightfiles' / 'checks.json'))
# Stage 4d: the path about the hover point is file - file(0) (the firmware shifts the first point onto the hover point, crazyflie-firmware planner.c:335-348), drawn at the cf5 pad
pad = np.array(next(iter(chk.values()))['pad_cf5']); pad2 = np.array(next(iter(chk.values()))['pad_cf_second'])
style = {'circle_frsim_rot90_tuning_mode1.csv': ('tab:blue', '-', 'CIRCLE turned +90 (lab)'), 'figure8_frsim_rot90_tuning_mode1.csv': ('tab:orange', '-', 'FIG8 turned +90 (lab)'),
         'circle_frsim_tuning_mode1.csv': ('0.55', '--', 'circle unturned (not for the lab)'), 'oval_frsim_study_mode1.csv': ('0.55', ':', 'OVAL (dropped)')}
for name, c in chk.items():
    col, ls, lab = style[name]; pf = M.read_csv(HERE.parent / 'flightfiles' / name); t = np.linspace(0, c['duration_s'], 600); p = M.sample(pf, t); p = p - p[0] + np.append(pad, 0)
    ax.plot(p[:, 0], p[:, 1], color=col, lw=2, ls=ls, label='%s%s' % (lab, '' if c['fits_at_pad'] else ' - outside'))
ax.plot(*pad, 'ko', ms=5); ax.annotate('cf5 pad (start = end)', pad, xytext=(5, 6), textcoords='offset points', fontsize=7)
ax.plot(*pad2, 'kx', ms=6); ax.annotate('cf_second pad (disabled)', pad2, xytext=(5, -10), textcoords='offset points', fontsize=7)
ax.set_xlim(-2.6, 2.6); ax.set_ylim(-3.6, 3.6); ax.set_aspect('equal'); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_title('Flight files about the cf5 pad inside the flight space\n(path = file - file(0); box = safety.py FLIGHT_SPACE)', fontsize=9)
ax.legend(fontsize=7, loc='lower center'); ax.grid(alpha=0.3); fig.tight_layout(); fig.savefig(HERE / 'figs' / 'flightfiles_volume.png', dpi=140); plt.close(fig)

res = json.load(open(HERE / 'results' / 'ingest_oct23.json'))
a4 = [r for r in res if r['scenario'] == 'A4' and r['top'] == 'planned']
fig, ax = plt.subplots(1, 2, figsize=(7, 3.2), sharey=True, sharex=True)
import usd_ingest as U, reference as R  # noqa
from frsim.scenarios import study_cells  # noqa
r = a4[0]; f = HERE.parents[2] / 'experiments' / 'logs' / 'bank_unseen_S0_2026-10-23' / r['folder'] / (r['folder'] + '_merged_usd.csv'); fl = U.load_merged(f); ref = R.build('A4', r['cell'])
c = fl.cols[fl.bottom]; th = np.radians(r['frame']['theta_deg']); R2 = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]); lab = np.stack([c['ctrltarget_x'], c['ctrltarget_y']], 1); m = (fl.t > 2) & (fl.t <= ref.scenario_s)
ax[0].plot(ref.pos[:, 0], ref.pos[:, 1], 'k', label='frsim reference'); ax[0].plot(lab[m, 0], lab[m, 1], 'r--', label='logged ctrltarget (lab frame)'); ax[0].set_title('before: max diff %.2f m' % r['reference_check_raw']['max_abs_scored_m'], fontsize=9)
al = lab @ R2.T + np.array(r['frame']['t_xy']); ax[1].plot(ref.pos[:, 0], ref.pos[:, 1], 'k'); ax[1].plot(al[m, 0], al[m, 1], 'r--'); ax[1].set_title('after rotation %.0f deg + shift: %.3f m' % (r['frame']['theta_deg'], r['reference_check']['max_abs_scored_m']), fontsize=9)
for a in ax: a.set_aspect('equal'); a.set_xlabel('x (m)'); a.grid(alpha=0.3)
ax[0].set_ylabel('y (m)'); ax[0].legend(fontsize=7); fig.suptitle('A4 (S0, 23 Oct): lab flies the scenario rotated', fontsize=9); fig.tight_layout(); fig.savefig(HERE / 'figs' / 'a4_frame.png', dpi=140)
