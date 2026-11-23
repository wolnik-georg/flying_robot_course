#!/usr/bin/env python3
"""Meeting doc 2026-11-26, section 6: round 1 of protocol v1 vs v2 (same round 0 flights), real uSD flights.

Run: DIFFCOTUNE_METHOD=ours2 ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/diffcotune_v1_v2_compare.py
(ours2 makes usd_ingest return the attitude part; the position part is identical to v1.)
Flight pick per scenario = round.py's rule (first usable attempt, not crashed, no floor contact).
Attitude angle = arccos(1 - mean(1 - cos e)), the angle equivalent of the mean DiffTune attitude term tr(I - R*^T R)/2.
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
LAB = ROOT / 'experiments/diffcotune/lab'
LOGS = ROOT / 'experiments/logs/usd_merged'
OUT = ROOT / 'docs/meetings/assets/2026-11-26'
sys.path.insert(0, str(LAB.parent)); sys.path.insert(0, str(LAB))
import usd_ingest as U  # noqa: E402
import round as RD  # noqa: E402

SCEN = ['A1', 'A8', 'A4', 'CIRCLE', 'FIG8']
ORDER = ['S1', 'S3', 'S0', 'S2']
NAME = {'S0': 'S0 geometric', 'S1': 'S1 INDI', 'S2': 'S2 geometric + NS2', 'S3': 'S3 INDI + NS2'}


def round_metrics(folder):
    man = json.load(open(folder / 'flights.json')); pos, att = [], []
    for sc in SCEN:
        for m in sorted([m for m in man if m['scenario'] == sc and m['role'] == 'tuning'], key=lambda m: m['attempt']):
            g = U.ingest_flight(folder / m['file'], sc, RD.cell_of(sc, 'tuning', 0), top_from_log=sc in RD.TOP_FROM_LOG,
                                rotate_deg=m.get('rotate_deg'), lag_s=m.get('lag_s'))
            if g.get('usable') and not g['flags']['crashed'] and not g['flags']['floor']:
                pos.append(g['pos_cost_m2']); att.append(g['att_cost']); break
    p, a = float(np.mean(pos)), float(np.mean(att))
    return dict(pos_cm=100 * p ** .5, att_deg=float(np.degrees(np.arccos(1 - a))), total=p + a)


R = {s: {'round 0': round_metrics(LOGS / 'rounds' / s / 'round_0'), 'v1 round 1': round_metrics(LOGS / 'rounds' / s / 'round_1'),
         'v2 round 1': round_metrics(LOGS / 'rounds_v2' / s / 'round_1')} for s in ORDER}
for s in ORDER:
    print(s, ' | '.join('%s: %.2f cm, %.1f deg, %.2e' % (k, v['pos_cm'], v['att_deg'], v['total']) for k, v in R[s].items()))

fig, axs = plt.subplots(1, 2, figsize=(12, 4.2))
x = np.arange(len(ORDER)); w = .26
cols = {'round 0': '0.75', 'v1 round 1': '#c0392b', 'v2 round 1': '#1e8449'}
for ax, key, lab in [(axs[0], 'pos_cm', 'position RMS [cm]'), (axs[1], 'att_deg', 'attitude error [deg]')]:
    for k, (name, col) in enumerate(cols.items()):
        ax.bar(x + (k - 1) * w, [R[s][name][key] for s in ORDER], w, color=col, label=name)
    ax.set_xticks(x); ax.set_xticklabels([NAME[s] for s in ORDER], fontsize=9); ax.set_ylabel(lab); ax.grid(axis='y', alpha=.3)
    ax.spines[['top', 'right']].set_visible(False)
axs[0].legend(frameon=False, fontsize=9)
fig.suptitle('Round 1 from the same round 0 flights: v1 (position cost) vs v2 (position + attitude cost, rate in the fit)', y=1.02)
fig.tight_layout(); fig.savefig(OUT / 'v1_v2_round1.png', dpi=150, bbox_inches='tight'); plt.close(fig)
print('written', OUT / 'v1_v2_round1.png')
