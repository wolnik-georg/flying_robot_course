#!/usr/bin/env python3
"""Stage 3c Part B report (markdown) from dryrun/stage3c/results/*.json -> report_stage3c.md (part B). python3 dryrun/stage3c/report_partb.py"""
import json, collections
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent; RES = HERE / 'results'
out = []; P = lambda *a: out.append(' '.join(str(x) for x in a))
BN = ['thrust gain', 'motor lag', 'downwash', 'plume']; SC = ['A1', 'A8', 'CIRCLE', 'FIG8']
c34 = json.load(open(RES / 'check3_check4.json')); pr = json.load(open(RES / 'check_predict.json'))
cl = json.load(open(RES / 'check5_clean_flights.json')); c2 = json.load(open(RES / 'check2_fit_truth.json')); c1 = json.load(open(RES / 'check1_oracle.json'))

P('# Stage 3c, Part B: cause of the drift of (d) on P1 (desk checks)\n')
P('Setup of every check: P1, J_sysId = position + velocity + attitude, 50 epochs, best iterate, learning rate 0.01 (Stage 3b). The six gain sets and flights of (d) are re-flown with the loop seeds (reproduced exactly, real cost equal to the loop record).\n')

P('## Check 1: oracle tune (true beta, no fit), one inner loop, real drone 5 seeds, mean 3-D RMS cm (crashes by the tilt rule in brackets)\n')
P('| start of the tune | model cost start -> best | best epoch | real tuning cells: start gains -> tuned | real test cells: start gains -> tuned | tuned gains (factor to default) |\n|---|---|---|---|---|---|')
G = ['Pxy', 'Pz', 'Dxy', 'Dz', 'Iz', 'KRxy', 'KRz', 'Kwxy', 'Kwz', 'KIxy']
for r in c1:
    f = lambda k, role: np.mean([e['rms3d_cm'] for e in r[k] if e['role'] == role])
    P('| %s | %.5f -> %.5f | %d | %.2f -> %.2f | %.2f -> %.2f | %s |' % (r['start'], r['J0'], r['J_best'], r['best_epoch'], f('eval_start', 'tuning'), f('eval_tuned', 'tuning'), f('eval_start', 'test'), f('eval_tuned', 'test'), ' '.join('%s %.2f' % (g, v) for g, v in zip(G, np.exp(r['phi'])))))
P('\nPer scenario (mean 3-D RMS cm over 5 seeds), start -> tuned:\n')
P('| start | A1 | A8 | CIRCLE | FIG8 | A4 off 0.125 | A4 off 0.075 | OVAL |\n|---|---|---|---|---|---|---|---|')
for r in c1:
    def per(k):
        t = collections.OrderedDict()
        for e in r[k]:
            t.setdefault(e['scenario'] + (str(e['cell'].get('offset')) if e['scenario'] == 'A4' else ''), []).append(e['rms3d_cm'])
        return [np.mean(v) for v in t.values()]
    P('| %s | %s |' % (r['start'], ' | '.join('%.1f -> %.1f' % (a, b) for a, b in zip(per('eval_start'), per('eval_tuned')))))

P('\n## Check 2: beta fit started at the true beta, flights of (d) round 3 and round 4 (those flights are the failed ones)\n')
P('| flights of | best epoch | J start -> best | beta error % (thrust / lag / downwash / plume) at epoch 10 | 30 | 50 |\n|---|---|---|---|---|---|')
TRUE = np.log([0.8, 1.5, 1.3, 0.7])
for r in c2:
    e = lambda k: [round(float(100 * (np.exp(x - t) - 1)), 1) for x, t in zip([l for l in r['log'] if l['k'] == k][0]['x'], TRUE)]
    P('| round %d | %d | %.4f -> %.4f | %s | %s | %s |' % (r['round'], r['best_epoch'], r['J0'], r['J_best'], e(10), e(30), e(50)))
P('')

P('## Check 3: what drives the fit (J_sysId = p + v + attitude, at theta_l and the beta of round l)\n')
P('Share of each part in the full four-part cost (mean over the 4 scenarios; w = body rates, not in the cost of Stage 3b) and the gradient of the Stage 3b cost with respect to beta, split by part (sum over the scenarios, positive = descent lowers that beta):\n')
P('| round | J p | J v | J q | J w | share w % | dJ/dbeta total (thrust / lag / downwash / plume) | from p | from v | from q |\n|---|---|---|---|---|---|---|---|---|---|')
for r in c34['parts']:
    v = {k: np.mean(r['values'][k]) for k in 'pvqw'}; g = {k: np.mean(r['grads'][k], axis=0) for k in 'pvq'}; tot = sum(g.values())
    P('| %d | %.4f | %.4f | %.4f | %.2f | %.0f | %s | %s | %s | %s |' % (r['round'], v['p'], v['v'], v['q'], v['w'], 100 * v['w'] / sum(v.values()), np.round(tot, 4).tolist(), np.round(g['p'], 4).tolist(), np.round(g['v'], 4).tolist(), np.round(g['q'], 4).tolist()))
P('\nPer scenario, J parts (p / v / q) in the rounds:\n')
P('| round | ' + ' | '.join(SC) + ' |\n|---|---|---|---|---|')
for r in c34['parts']:
    P('| %d | %s |' % (r['round'], ' | '.join('%.4f / %.4f / %.4f' % (r['values']['p'][i], r['values']['v'][i], r['values']['q'][i]) for i in range(4))))
P('\nWhere the state of the trace comes from: `frsim/episode.py` records `body.x, body.v, body.q, body.w`, the true state of the plant at the start of the tick, not the noisy sensor estimate (`sen.state` feeds only the controller). The model output is the true state of its own noise-free plant, so position and velocity are compared as true-state against true-state. The noise enters through the controller (noisy feedback and gyro), which makes the real true-state differ from the noise-free model.\n')

P('## Check 4: per scenario real cost of (d), 3-D RMS / z RMS cm and max tilt deg of the flight (one flight per scenario and round)\n')
P('| round | ' + ' | '.join(SC) + ' |\n|---|---|---|---|---|')
for l, r in enumerate(c34['per_scenario']):
    P('| %d | %s |' % (l, ' | '.join('%.1f / %.1f, %.0f deg' % (x['rms3d_cm'], x['z_rms_cm'], x['tilt']) for x in r)))

P('\n## Check 5: the model against the real drone, per scenario (3-D RMS cm, task window)\n')
P('Columns: real = the flight of the loop (noise on); model, true beta = the differentiable model with the true P1 beta; model, beta0 = with the calibration; model, beta of the tune = with the beta fitted in the round that produced the gains (that is the model the tune used). `noise off` = frsim with the sensor noise off, same gains (extra flights, `clean_flights.py`).\n')
P('| gains | scenario | real (noise on) | real max tilt | frsim noise off | model true beta | model beta0 | model beta of the tune |\n|---|---|---|---|---|---|---|---|')
cl_by = {(c['round'], c['scenario']): c for c in cl}
for l, r in enumerate(pr):
    for i, s in enumerate(SC):
        c = cl_by[(l, s)]
        P('| theta_%d | %s | %.1f | %.0f | %.1f | %.1f | %.1f | %.1f |' % (l, s, r['real'][i], c['noise_on']['tilt'], c['noise_off']['rms3d_cm'], r['true_beta'][i], r['beta0'][i], r['beta_of_the_round_that_tuned_it'][i]))
P('')
P(open(HERE / 'reading_partB.md').read())
open(HERE / 'report_stage3c_partB.md', 'w').write('\n'.join(out))
print('\n'.join(out))
