#!/usr/bin/env python3
"""Plot the references of the six cells (A1, A4, A8, CIRCLE, FIG8, OVAL; first study cell): xy path of the bottom vehicle
(and of the top vehicle for the two-vehicle scenarios), height and speed over scenario time.
  ~/.pyenv/versions/flying_robots/bin/python checks/plot_references.py [OUT.png]"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import reference as R  # noqa: E402

SC = ['A1', 'A4', 'A8', 'CIRCLE', 'FIG8', 'OVAL']
out = sys.argv[1] if len(sys.argv) > 1 else str(HERE / 'references_6.png')
fig, ax = plt.subplots(3, 6, figsize=(24, 10))
for j, s in enumerate(SC):
    ref = R.build(s)
    t = ref.time_s; m = (t >= -0.01) & (t <= ref.scenario_s)
    ax[0, j].plot(ref.pos[m, 0], ref.pos[m, 1], 'C0', label='bottom (tuned)')
    if ref.has_top:
        ax[0, j].plot(ref.top_pos[m, 0], ref.top_pos[m, 1], 'C3--', label='top')
    ax[0, j].plot(*ref.pos[m][0, :2], 'ko', ms=4)
    ax[0, j].set_aspect('equal'); ax[0, j].set_title('%s  %s' % (s, {k: v for k, v in list(ref.cell.items())[:4]}), fontsize=8)
    ax[0, j].set_xlabel('x [m]'); ax[0, j].set_ylabel('y [m]'); ax[0, j].grid(alpha=.3); ax[0, j].legend(fontsize=7)
    ax[1, j].plot(t, ref.pos[:, 2], 'C0', label='bottom z')
    if ref.has_top:
        ax[1, j].plot(t, ref.top_pos[:, 2], 'C3--', label='top z')
    ax[1, j].axvspan(0, ref.scenario_s, color='0.9'); ax[1, j].set_xlabel('scenario time [s] (takeoff, hold, scenario (grey), landing)'); ax[1, j].set_ylabel('z [m]'); ax[1, j].grid(alpha=.3); ax[1, j].legend(fontsize=7)
    sp = np.linalg.norm(ref.vel, axis=1); ac = np.linalg.norm(ref.acc, axis=1)
    ax[2, j].plot(t, sp, 'C0', label='speed [m/s] (peak %.2f)' % sp[m].max()); ax[2, j].plot(t, ac, 'C1', lw=.8, label='acceleration [m/s^2] (peak %.2f)' % ac[m].max())
    ax[2, j].axvspan(0, ref.scenario_s, color='0.9'); ax[2, j].set_xlabel('scenario time [s]'); ax[2, j].grid(alpha=.3); ax[2, j].legend(fontsize=7)
fig.suptitle('References of the six cells (frsim, first study cell): xy path, height, speed and acceleration of the bottom vehicle')
fig.tight_layout(); fig.savefig(out, dpi=90)
print(out)
