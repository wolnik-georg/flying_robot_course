"""Stage 8 (copy of the Stage 7 / Stage 5 section writer for loop_<C>_<tag>.json, 6 rounds, paper variant: crashes / floor / tilt per round and the largest gain change per round added). Usage: python3 sections8.py <C> (env STAGE8_TAG, default paper)\nStage 5: markdown section of a controller dry run (criteria a-c, beta per round) from results/loop_<C>.json and eval_<C>.json.
Usage: python3 sections5.py S2|S1|S3   (writes results/sections_<C>.md)"""
import json, os, sys
from pathlib import Path
C = sys.argv[1]; os.environ['DIFFCOTUNE_CONTROLLER'] = C
import numpy as np
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE.parent))
import loop_lib as L  # noqa
R = HERE / 'results'; J = lambda p: json.load(open(p))
TAG = os.environ.get('STAGE8_TAG', 'paper'); rec = J(R / f'loop_{C}_{TAG}.json'); ev = J(R / f'eval_{C}_{TAG}.json'); rd = rec['rounds']; n = len(rd)
failed = lambda e: bool(e['crashed'] or e.get('floor', False))
berr = lambda psi: [100 * (np.exp(a - b) - 1) for a, b in zip(psi, L.PSI_TRUE)]
rm = lambda e_, role: float(np.mean([e['rms3d_cm'] for e in e_ if e['role'] == role]))
yn = lambda t: 'yes' if t else 'no'
SC = [s for s, _ in L.TUNING]; o = []; P = lambda *a: o.append(' '.join(str(x) for x in a))
P('| Gain set | real 3-D RMS (cm, mean of %d tuning flights) | z RMS (cm) | failed flights (tilt or floor) | minutes (round) |\n|---|---|---|---|---|' % len(L.TUNING))
for l, r in enumerate(rd):
    bad = [f['scenario'] for f in r['flights'] if failed(f)]
    P('| theta_%d | %.2f | %.2f | %s | %.1f |' % (l, r['real']['rms3d_cm'], r['real'].get('z_rms_cm', float('nan')), ', '.join(bad) or '0', r['seconds_round'] / 60))
P('\nKeep-best over the flown sets: theta_%d (%.2f cm).\n' % (rec['best']['round'], 100 * rec['best']['cost'] ** 0.5))
P('| Round | loop | epochs | best epoch | J start | J best | gain-limit hits |\n|---|---|---|---|---|---|---|')
for r in rd:
    for key, nm in (('fit', 'fit beta'), ('tune', 'tune theta')):
        if key in r:
            x = r[key]; P('| %d | %s | %d | %s | %.5g | %.5g | %s |' % (r['round'], nm, x['epochs'], x['best_epoch'] if x['best_epoch'] is not None else 'last (%s)' % x.get('reason'), x['J0'], x['J_best'] if x['J_best'] is not None else x['J_end'], x['hits'] if key == 'tune' else '-'))
P('\nbeta error after each fit (%% against the %s truth; %s):\n' % (L.PLANT_NAME if hasattr(L, 'PLANT_NAME') else 'plant', ' / '.join(['thrust gain', 'motor lag', 'downwash', 'plume'] + (['rpm delay'] if L.N_BETA == 5 else []))))
P('| Fit of round | error % |\n|---|---|')
for l, r in enumerate(rd):
    if 'fit' in r: P('| %d | %s |' % (l, ' / '.join('%+.1f' % x for x in berr(r['psi_after_fit']))))
P('\nFinal evaluation (seeds 2000-2004; 3-D RMS cm / z RMS cm / failed of 5):\n')
cells = list(L.TUNING) + list(L.TEST)
if L.TUNING_SET == '5': H = ['A1', 'A8', 'CIRCLE', 'FIG8', 'A4 cell 1 (tuning, in-sample)', 'A4 cell 2 (held out)', 'OVAL (frsim only)']
else: H = ['A1', 'A8', 'CIRCLE', 'FIG8', 'A4 cell 1', 'A4 cell 2', 'OVAL']
def tm(e_):          # test mean: Stage 7 = A4 cell 2 only (held out; OVAL is frsim-only information), earlier = all test cells
    es = [e['rms3d_cm'] for e in e_ if e['role'] == 'test' and (L.TUNING_SET == '4' or e['scenario'] == 'A4')]; return float(np.mean(es))
P('| Gain set | ' + ' | '.join(H) + ' | tuning mean | test mean |\n|---|' + '---|' * (len(H) + 2))
e0 = ev['eval']['theta_0']; eb = ev['eval']['theta_best']
for nm, e_ in (('theta_0 (default)', e0), ('theta_best (round %d)' % ev['best_round'], eb)):
    cs = []
    for s, c in cells:
        es = [e for e in e_ if e['scenario'] == s and e['cell'] == c]
        cs.append('%.2f / %.2f / %d' % (np.mean([e['rms3d_cm'] for e in es]), np.mean([e['z_rms_cm'] for e in es]), sum(failed(e) for e in es)))
    P('| %s | ' % nm + ' | '.join(cs) + ' | %.2f | %.2f |' % (rm(e_, 'tuning'), tm(e_)))
r_ = [x['real']['rms3d_cm'] for x in rd]; fl = [{f['scenario']: failed(f) for f in r['flights']} for r in rd]
a = all(r_[l] <= r_[1] + 1.0 for l in range(2, n)) if n >= 5 else None
bad = ['theta_%d %s' % (l, s) for l in range(1, n) for s in SC if fl[l][s] and not (fl[0][s] and fl[1][s])]
c = [rm(eb, 'tuning') < rm(e0, 'tuning'), tm(eb) < tm(e0)]
P('\n| Criterion | result |\n|---|---|')
P('| a. rounds 2-4 <= round 1 + 1 cm (round 1 = %.2f, limit %.2f) | **%s** (%s) |' % (r_[1], r_[1] + 1, 'not assessable' if a is None else yn(a), ', '.join('%.2f' % x for x in r_[2:])))
P('| b. no flown set after theta_0 fails a scenario that theta_0 or theta_1 passed | **%s** (%s) |' % (yn(not bad), ', '.join(bad) or 'none'))
P('| c. theta_best beats theta_0 on tuning and test means | **%s** (theta_best %.2f / %.2f, theta_0 %.2f / %.2f) |' % (yn(all(c)), rm(eb, 'tuning'), tm(eb), rm(e0, 'tuning'), tm(e0)))

P('\nPer round: real flights of the round (3-D RMS cm per scenario), crashed (tilt abort) / floor flights, repeats, largest relative gain change to the next flown set, epochs used by the stop rule (fit / tune):\n')
P('| Round | ' + ' | '.join(SC) + ' | crashed | floor | attempts (5 = first try each) | largest gain change to next set | epochs fit / tune | stop reasons |\n|---|' + '---|' * (len(SC) + 6))
for l, r in enumerate(rd):
    fls = r['flights']; per = {f['scenario']: f for f in fls}
    cm = ' | '.join('%.2f' % (100 * per[s]['cost'] ** 0.5) for s in SC)
    gc = rec['max_gain_change_per_round'][l] if 'max_gain_change_per_round' in rec and l < len(rec['max_gain_change_per_round']) else float('nan')
    ef = r['fit']['epochs'] if 'fit' in r else '-'; et = r['tune']['epochs'] if 'tune' in r else '-'
    rs = '%s / %s' % (r['fit'].get('reason', '-') if 'fit' in r else '-', r['tune'].get('reason', '-') if 'tune' in r else '-')
    P('| %d | %s | %d | %d | %d | %s | %s / %s | %s |' % (l, cm, sum(bool(f['crashed']) for f in fls), sum(bool(f['floor']) for f in fls), sum(f['tries'] for f in fls), ('%.1f %%' % (100 * gc)) if gc == gc else '-', ef, et, rs))
e0t = [e for e in e0 if e['role'] == 'test' and e['scenario'] == 'A4']; ebt = [e for e in eb if e['role'] == 'test' and e['scenario'] == 'A4']
a4c1 = lambda e_: float(np.mean([x['rms3d_cm'] for x in e_ if x['scenario'] == 'A4' and x['role'] == 'tuning']))
P('\nSummary: tuning mean theta_0 %.2f -> theta_best %.2f cm; held-out A4 cell 2 %.2f -> %.2f cm; in-sample A4 cell 1 %.2f -> %.2f cm; keep-best round %d; halt: %s.' % (rm(e0, 'tuning'), rm(eb, 'tuning'), tm(e0), tm(eb), a4c1(e0), a4c1(eb), ev['best_round'], ev.get('halt')))
t = '\n'.join(o) + '\n'; (R / f'sections8_{C}_{TAG}.md').write_text(t); print(t)
