"""Stage 6 step 5: facts for the scenario decision sheet (no runs): flight durations from reference.build (ticks), blocks, two-drone flag, role; estimated desk minutes per option (the desk epoch time of the pool is
the longest scenario when there are at most 4 scenarios: minutes ~ 22.5 min x longest blocks / 13250; 22.5 min = measured S1 round 0 in Stage 5c, fit 659 s + tune 690 s; an estimate, not a measurement). Writes results/scenario_facts.json and figs/*.png."""
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent))
import loop_lib as L, reference as R  # noqa
from frsim.scenarios import tuning_cells, study_cells  # noqa
cands = [('A1 tuning (dz 0.375)', 'A1', tuning_cells('A1')[0], 'tuning'), ('A8 tuning (dz 0.375, 8.0 s)', 'A8', tuning_cells('A8')[0], 'tuning'), ('CIRCLE (slow version)', 'CIRCLE', tuning_cells('CIRCLE')[0], 'tuning'), ('FIG8 (slow version)', 'FIG8', tuning_cells('FIG8')[0], 'tuning'),
         ('A4 cell 1 (test)', 'A4', study_cells('A4')[0], 'test'), ('A4 cell 2 (test)', 'A4', study_cells('A4')[1], 'test'), ('OVAL (not flyable at the pad)', 'OVAL', study_cells('OVAL')[0], 'test'),
         ('A1 study cell (dz 0.45)', 'A1', study_cells('A1')[0], 'study'), ('A8 study cell (dz 0.4, 6.25 s)', 'A8', study_cells('A8')[0], 'study')]
rows = []
for name, s, c, role in cands:
    r = R.build(s, c); rows.append(dict(name=name, scenario=s, role=role, ticks=int(r.n_ticks), blocks=int(r.n_ticks // 2), scenario_s=float(r.scenario_s), total_s=r.n_ticks / 1000.0, two_drones=bool(r.has_top)))
    print('%-34s %-7s total %.1f s, scenario part %.1f s, blocks %d, two drones %s' % (name, role, rows[-1]['total_s'], rows[-1]['scenario_s'], rows[-1]['blocks'], rows[-1]['two_drones']))
BASE_MIN, BASE_BLOCKS, FLY_S = 22.5, 13250, 86.6
def blocks(ss): return max(next(r['blocks'] for r in rows if r['name'].startswith(n)) for n in ss)
opts = {'A: current (A1, A8, CIRCLE, FIG8)': ['A1 tuning', 'A8 tuning', 'CIRCLE', 'FIG8'], 'B: solo only (CIRCLE, FIG8)': ['CIRCLE', 'FIG8'], 'C: study settings (A1 study, A8 study, CIRCLE, FIG8)': ['A1 study', 'A8 study', 'CIRCLE', 'FIG8'], 'D1: without A8 (A1, CIRCLE, FIG8)': ['A1 tuning', 'CIRCLE', 'FIG8']}
res = {}
for k, ss in opts.items():
    nb = blocks(ss); res[k] = dict(flights=len(ss), flying_min=len(ss) * FLY_S / 60, desk_min_estimate=BASE_MIN * nb / BASE_BLOCKS, longest_blocks=nb)
    print('%-58s flights %d, flying %.1f min, desk estimate %.1f min' % (k, len(ss), res[k]['flying_min'], res[k]['desk_min_estimate']))
json.dump(dict(scenarios=rows, options=res), open(HERE / 'results' / 'scenario_facts.json', 'w'), indent=1)
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
col = {'tuning': '#0072B2', 'test': '#E69F00', 'study': '#009E73'}
fig, ax = plt.subplots(figsize=(8.5, 3.6)); y = np.arange(len(rows))[::-1]
for yi, r in zip(y, rows):
    ax.barh(yi, r['total_s'], color=col[r['role']], height=0.6, hatch='' if r['two_drones'] else '///', edgecolor='white')
    ax.text(r['total_s'] + 0.3, yi, '%.1f s%s' % (r['total_s'], '' if r['two_drones'] else '  (solo)'), va='center', fontsize=8)
ax.set_yticks(y); ax.set_yticklabels([r['name'] for r in rows], fontsize=8); ax.set_xlabel('simulated flight time of the cell (s, includes take-off and hover before the scenario)'); ax.set_xlim(0, 32)
ax.legend(handles=[plt.Rectangle((0, 0), 1, 1, color=col[k]) for k in col] + [plt.Rectangle((0, 0), 1, 1, fc='white', ec='grey', hatch='///')], labels=['tuning cell', 'test cell (lab)', 'frsim study cell', 'solo (one drone)'], fontsize=7, frameon=False, loc='lower right')
for s in ('top', 'right'): ax.spines[s].set_visible(False)
plt.tight_layout(); plt.savefig(HERE / 'figs' / 'scenario_durations.png', dpi=140); plt.close()
fig, ax = plt.subplots(figsize=(8.5, 3.2)); names = list(res); x = np.arange(len(names)); w = 0.38
ax.bar(x - w / 2, [res[n]['flying_min'] for n in names], w, color='#0072B2', label='flying per round (min)'); ax.bar(x + w / 2, [res[n]['desk_min_estimate'] for n in names], w, color='#D55E00', label='desk per round (min, estimate)')
for xi, n in zip(x, names): ax.text(xi - w / 2, res[n]['flying_min'] + 0.3, '%.1f' % res[n]['flying_min'], ha='center', fontsize=8); ax.text(xi + w / 2, res[n]['desk_min_estimate'] + 0.3, '%.1f' % res[n]['desk_min_estimate'], ha='center', fontsize=8)
ax.set_xticks(x); ax.set_xticklabels([n.split(' (')[0] for n in names], fontsize=8); ax.legend(fontsize=8, frameon=False)
for s in ('top', 'right'): ax.spines[s].set_visible(False)
plt.tight_layout(); plt.savefig(HERE / 'figs' / 'scenario_options.png', dpi=140); print('figures written')
