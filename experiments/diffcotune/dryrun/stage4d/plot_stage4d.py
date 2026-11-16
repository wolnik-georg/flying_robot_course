import json
from pathlib import Path
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
H = Path(__file__).resolve().parents[2]; d = json.load(open(H / 'lab/results/cmd_check.json')); ck = json.load(open(H / 'flightfiles/checks.json'))
names, vals = [], []
for o in d['run_formation'][:4]:
    names.append(o['name'] + ' (run_formation)'); vals.append(min(o['margin_to_flight_space_m']['x'], o['margin_to_flight_space_m']['y']))
for k, lab in (('circle_frsim_rot90_tuning_mode1.csv', 'CIRCLE turned +90 (file)'), ('figure8_frsim_rot90_tuning_mode1.csv', 'FIG8 turned +90 (file)')):
    names.append(lab); vals.append(ck[k]['min_margin_at_pad_m'])
GAP = 86.6 / 60; DESK = [1209 / 60, 1232 / 60, (1209 + 1232) / 120, 10.0]
ink, mut, blue, gray = '#1f2933', '#6b7280', '#2563eb', '#9ca3af'
fig, ax = plt.subplots(1, 2, figsize=(11, 3.8), gridspec_kw=dict(width_ratios=[1.15, 1]))
a = ax[0]; y = list(range(len(names)))[::-1]; a.barh(y, vals, color=blue, height=0.55)
for yy, v in zip(y, vals): a.text(v + 0.02, yy, '%.2f m' % v, va='center', fontsize=9, color=ink)
a.set_yticks(y); a.set_yticklabels(names, fontsize=9, color=ink); a.set_xlim(0, 1.2); a.set_xlabel('smallest horizontal margin to the flight space [m] (files: at the cf5 pad (0.0, 0.3))', color=mut, fontsize=9)
a.set_title('All lab cells fit; CIRCLE has the smallest margin', fontsize=10, color=ink, loc='left')
b = ax[1]; xs = ['round %d' % i for i in range(4)]; fl = [4 * GAP] * 4
b.bar(xs, fl, color=blue, width=0.55, label='4 tuning flights (86.6 s each, measured 23 Oct)'); b.bar(xs, DESK, bottom=fl, color=gray, width=0.55, label='desk time before the next round (rounds 2, 3 estimated)')
for x, f, dk in zip(xs, fl, DESK): b.text(x, f / 2, '%.1f' % f, ha='center', va='center', color='white', fontsize=9); b.text(x, f + dk / 2, '%.1f' % dk, ha='center', va='center', color=ink, fontsize=9)
b.set_ylim(0, 40); b.set_ylabel('minutes', color=mut, fontsize=9); b.legend(fontsize=8, frameon=False, loc='upper right'); b.tick_params(labelsize=9)
b.set_title('One S0 loop: 4 rounds = %.0f min (23 flying + 71 desk)' % (4 * 4 * GAP + sum(DESK)), fontsize=10, color=ink, loc='left')
for x in ax: [x.spines[s].set_visible(False) for s in ('top', 'right')]
fig.tight_layout(); out = H / 'lab/figs/stage4d_checks_and_time.png'; fig.savefig(out, dpi=140); print(out)
