import json
from pathlib import Path
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
H = Path(__file__).resolve().parents[2]; d = json.load(open(H / 'lab/results/cmd_check.json'))
names, vals = [], []
for o in d['run_formation'][:4]:
    names.append(o['name'] + ' (run_formation)'); vals.append(max(o['max_diff_bottom_path_mm'], o.get('max_diff_top_minus_bottom_mm', 0)))
for k, v in d['solo_files'].items():
    names.append({'circle': 'CIRCLE', 'figure8': 'FIG8', 'oval': 'OVAL'}[k] + ' (flight file)'); vals.append(v['max_pos_diff_mm'])
GAP = 86.6; DESK = [1209 / 60, 1232 / 60, (1209 + 1232) / 120, 10.0]   # min: round 0, 1 measured (dry test); round 2 = their mean (estimate); round 3 ~10 min (estimate, protocol draft)
flying = {'1 flight per scenario\n(16 flights)': 16 * GAP / 60, '4 flights per scenario\n(64 flights)': 64 * GAP / 60}
desk = sum(DESK)
ink, mut, blue, gray = '#1f2933', '#6b7280', '#2563eb', '#9ca3af'
fig, ax = plt.subplots(1, 2, figsize=(11, 3.8), gridspec_kw=dict(width_ratios=[1.15, 1]))
a = ax[0]; y = range(len(names))[::-1]; a.barh(list(y), vals, color=blue, height=0.55)
for yy, v in zip(y, vals): a.text(v + 0.012, yy, '%.4f mm' % v, va='center', fontsize=9, color=ink)
a.set_yticks(list(y)); a.set_yticklabels(names, fontsize=9, color=ink); a.set_xlim(0, 0.8); a.set_xlabel('max difference, commanded vs frsim reference [mm]', color=mut, fontsize=9)
a.set_title('Commands reproduce the frsim cells (all < 0.6 mm)', fontsize=10, color=ink, loc='left')
b = ax[1]; xs = list(flying); fl = list(flying.values())
b.bar(xs, fl, color=blue, width=0.5, label='flying (86.6 s per flight, measured 23 Oct)'); b.bar(xs, [desk] * 2, bottom=fl, color=gray, width=0.5, label='desk time between rounds (%.0f min, 4 rounds)' % desk)
for x, f in zip(xs, fl): b.text(x, f / 2, '%.0f min' % f, ha='center', va='center', color='white', fontsize=10); b.text(x, f + desk / 2, '%.0f min' % desk, ha='center', va='center', color=ink, fontsize=10); b.text(x, f + desk + 2, 'total %.0f min' % (f + desk), ha='center', fontsize=10, color=ink)
b.set_ylim(0, 230); b.set_ylabel('minutes (estimate, no repeats, no battery swaps)', color=mut, fontsize=9); b.legend(fontsize=8, frameon=False, loc='upper left'); b.tick_params(labelsize=9)
b.set_title('One S0 loop, 4 rounds (tuning flights only)', fontsize=10, color=ink, loc='left')
for x in ax: [x.spines[s].set_visible(False) for s in ('top', 'right')]
fig.tight_layout(); out = H / 'lab/figs/stage4c_checks_and_time.png'; fig.savefig(out, dpi=140); print(out, desk)
