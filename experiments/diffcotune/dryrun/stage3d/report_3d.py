#!/usr/bin/env python3
"""Builds report_stage3d.md from the result files (all numbers come from the JSON files)."""
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import loop_lib as L  # noqa
R = HERE / 'results'; B3 = HERE.parent / 'stage3b' / 'results'
J = lambda p: json.load(open(p))
GAINS = L.GAIN_NAMES; SC = [s for s, _ in L.TUNING]
out = []; P = lambda *a: out.append(' '.join(str(x) for x in a))
VAR = [v for v in ('A', 'B', 'AB') if (R / ('eval_%s.json' % v)).exists()]
LABEL = {'A': 'A (noise in the tuning step)', 'B': 'B (gain limit 0.25)', 'AB': 'A + B'}


def role_mean(ev, role, key='rms3d_cm'):
    return float(np.mean([e[key] for e in ev if e['role'] == role]))


def failed(e):
    return bool(e['crashed'] or e.get('floor', False))


def flightflags(rounds):
    """per round: dict scenario -> failed (tilt-crashed after the repeats, or floor)."""
    return [{f['scenario']: bool(f['crashed'] or f['floor']) for f in r['flights']} for r in rounds]


P('# Stage 3d report (S0, P1): noise in the tuning model (A), gain limit (B), A + B\n')
P(open(HERE / 'report_head.md').read())
# ---- speed
sc = J(HERE / 'speed_cpu.json'); sg = J(HERE / 'speed_gpu.json')
P('\n## 3. Speed of one tuning epoch of variant A (4 scenarios x 2 noise seeds = 8 rollouts)\n')
P('| Path | seconds per epoch | note |\n|---|---|---|')
P('| CPU pool, 4 workers, 2 rows (seeds) per worker as one compiled batch | %s | measured epochs 0-3; steady state %.1f s; the first epochs include compile and a leftover load from stopping my earlier job |' % (' / '.join('%.1f' % t for t in sc['epoch_seconds']), sc['epoch_seconds'][-1]))
P('| GPU float64, 8 rows in one batch (compiled) | %s | iteration 1 includes the compile; cost J identical to the CPU value (%.6e) |' % (' / '.join('%.1f' % t for t in sg['epoch_seconds']), sg['J']))
P('| CPU, 8 workers with 1 row each | not run | 8 x 1.5 GB does not fit with the other memory in use (MemAvailable 5.6 GB with one pool) |')
P('\nUsed for A and A + B: **the CPU pool** (about 3 x faster than the GPU). B has no noise rows and uses the same pool.')
for v in VAR:
    rec = J(R / ('loop_%s.json' % v)); m = [r['seconds_round'] / 60 for r in rec['rounds']]
    P('- minutes per round, variant %s: %s (tune epochs %s s on average)' % (v, ' / '.join('%.1f' % x for x in m), ' / '.join('%.1f' % np.mean([e['seconds'] for e in r['tune']['log']]) for r in rec['rounds'] if 'tune' in r)))
# ---- tables
ref = J(B3 / 'full_P1_cotune.json'); fr = J(R / 'floor_ref_rounds.json') if (R / 'floor_ref_rounds.json').exists() else None
P('\n## 4. Per variant: real cost per round, failed flights, gain-limit hits, beta error\n')
P('Real cost = mean over the 4 tuning scenarios (one flight each) of the 3-D RMS, cm. Failed = tilt rule (after up to 2 repeats) or floor rule (z mean < 50 % of the reference z mean). In brackets the scenarios that failed.\n')
P('| Gain set (round) | Stage 3b (d), reference | ' + ' | '.join('variant ' + v for v in VAR) + ' |\n|---|---|' + '---|' * len(VAR))
recs = {v: J(R / ('loop_%s.json' % v)) for v in VAR}
for l in range(4):
    def cell(rd):
        fl = rd.get('flights'); real = rd['real']
        s = '%.2f' % real['rms3d_cm']
        if fl:
            bad = [f['scenario'] for f in fl if f['crashed'] or f['floor']]
            s += ' cm, failed %d%s' % (len(bad), (' (' + ', '.join(bad) + ')') if bad else '')
        else:
            s += ' cm, tilt-crashed %d' % real['crashed']
        return s
    refcell = '%.2f cm' % ref['rounds'][l]['real']['rms3d_cm']
    if fr:
        bad = [f['scenario'] for f in fr[l]['flights'] if f['crashed'] or f['floor']]
        refcell += ', failed %d%s' % (len(bad), (' (' + ', '.join(bad) + ')') if bad else '')
    P('| theta_%d | %s | ' % (l, refcell) + ' | '.join(cell(recs[v]['rounds'][l]) if len(recs[v]['rounds']) > l else 'not run' for v in VAR) + ' |')
P('\nGain-limit hits (number of projected Adam steps out of 50 per gain; order %s):\n' % ' '.join(GAINS))
for v in VAR:
    for r in recs[v]['rounds']:
        if 'tune' in r and r['tune'].get('hits') is not None:
            P('- %s, tuning after round %d: limit %s, hits %s' % (v, r['round'], 'off' if not r['tune'].get('limit') else '%.4f (= log 1.25)' % r['tune']['limit'], r['tune']['hits']))
P('\nBeta error after round 3 (the fit of round 3 is skipped, so this is the beta of the round-2 fit; relative error of each value against the P1 truth, %):\n')
P('| | thrust gain | motor lag | downwash | plume |\n|---|---|---|---|---|')
P('| Stage 3b (d), after round 2 fit | ' + ' | '.join('%+.1f' % (100 * (np.exp(a - b) - 1)) for a, b in zip(ref['rounds'][2]['psi_after_fit'], L.PSI_TRUE)) + ' |')
for v in VAR:
    P('| %s | ' % v + ' | '.join('%+.1f' % (100 * (np.exp(a - b) - 1)) for a, b in zip(recs[v]['rounds'][2]['psi_after_fit'], L.PSI_TRUE)) + ' |')
# ---- final evaluation
P('\n## 5. Final evaluation (5 seeds, 4 tuning + 3 test cells; mean over seeds)\n')
evr = J(R / 'eval_ref.json') if (R / 'eval_ref.json').exists() else None
rows = []
if evr:
    for k in ('default', 'theta_best', 'theta_3'):
        rows.append(('Stage 3b: ' + {'default': '(a) default', 'theta_best': '(d) theta_best (round %d)' % evr['best_round'], 'theta_3': '(d) theta_3'}[k], evr['eval'][k]))
evs = {}
for v in VAR:
    e = J(R / ('eval_%s.json' % v)); evs[v] = e
    rows.append(('%s theta_best (round %d)' % (v, e['best_round']), e['eval']['theta_best'])); rows.append(('%s theta_3' % v, e['eval']['theta_3']))
cells = [(s, c) for s, c in L.TUNING] + L.TEST
P('3-D RMS cm / z RMS cm / failed flights of 5 (tilt or floor); per scenario:\n')
hdr = ['A1', 'A8', 'CIRCLE', 'FIG8', 'A4 cell 1', 'A4 cell 2', 'OVAL']
P('| Gain set | ' + ' | '.join(hdr) + ' | tuning mean | test mean |\n|---|' + '---|' * (len(hdr) + 2))
for name, ev in rows:
    cs = []
    for s, c in cells:
        es = [e for e in ev if e['scenario'] == s and e['cell'] == c]
        cs.append('%.2f / %.2f / %d' % (np.mean([e['rms3d_cm'] for e in es]), np.mean([e['z_rms_cm'] for e in es]), sum(failed(e) for e in es)))
    P('| %s | ' % name + ' | '.join(cs) + ' | %.2f | %.2f |' % (role_mean(ev, 'tuning'), role_mean(ev, 'test')))
# ---- criteria
P('\n## 6. Pass criteria (P1, per variant)\n')
dflt = evr['eval']['default'] if evr else None
P('| Criterion | ' + ' | '.join('variant ' + v for v in VAR) + ' | Stage 3b (d) for comparison |\n|---|' + '---|' * (len(VAR) + 1))
def c1(rounds):
    r = [x['real']['rms3d_cm'] for x in rounds]; ok = all(r[l] <= r[1] + 1.0 for l in (2, 3)); return ok, 'theta_2 %.2f, theta_3 %.2f vs theta_1 + 1 = %.2f' % (r[2], r[3], r[1] + 1)
def c2(flags):
    bad = []
    for l in range(1, len(flags)):
        for s in SC:
            if flags[l][s] and not (flags[0][s] and flags[1][s]):
                bad.append('theta_%d %s' % (l, s))
    return not bad, ('violations: ' + ', '.join(bad)) if bad else 'none'
def c3(ev_best, ev_3):
    t = [role_mean(e, r) < role_mean(dflt, r) for e in (ev_best, ev_3) for r in ('tuning', 'test')]
    return all(t), 'theta_best tuning %.2f test %.2f; theta_3 tuning %.2f test %.2f; default %.2f / %.2f' % (role_mean(ev_best, 'tuning'), role_mean(ev_best, 'test'), role_mean(ev_3, 'tuning'), role_mean(ev_3, 'test'), role_mean(dflt, 'tuning'), role_mean(dflt, 'test'))
res1 = {v: c1(recs[v]['rounds']) for v in VAR}; res2 = {v: c2(flightflags(recs[v]['rounds'])) for v in VAR}
res3 = {v: c3(evs[v]['eval']['theta_best'], evs[v]['eval']['theta_3']) for v in VAR} if dflt else {}
rr = [x['real']['rms3d_cm'] for x in ref['rounds'][:4]]
refc1 = (all(rr[l] <= rr[1] + 1 for l in (2, 3)), 'theta_2 %.2f, theta_3 %.2f vs %.2f' % (rr[2], rr[3], rr[1] + 1))
refc2 = c2([{f['scenario']: bool(f['crashed'] or f['floor']) for f in x['flights']} for x in fr]) if fr else (None, 'not available')
refc3 = c3(evr['eval']['theta_best'], evr['eval']['theta_3']) if evr else (None, 'not available')
yn = lambda t: 'yes' if t[0] else ('no' if t[0] is not None else '?')
P('| 1. real cost does not rise after round 1 (round 2, 3 <= round 1 + 1 cm) | ' + ' | '.join('**%s** (%s)' % (yn(res1[v]), res1[v][1]) for v in VAR) + ' | **%s** (%s) |' % (yn(refc1), refc1[1]))
P('| 2. no flown gain set after theta_0 fails a scenario that theta_0 or theta_1 passed (tilt or floor) | ' + ' | '.join('**%s** (%s)' % (yn(res2[v]), res2[v][1]) for v in VAR) + ' | **%s** (%s) |' % (yn(refc2), refc2[1]))
P('| 3. theta_best and theta_3 beat (a) default on tuning and test | ' + ' | '.join('**%s** (%s)' % (yn(res3[v]), res3[v][1]) for v in VAR) + ' | **%s** (%s) |' % (yn(refc3), refc3[1]))
P('| 4. beta after round 3 | reported in section 4, no gate | |')
P('\nFloor rule per flown gain set and scenario (failed = tilt or floor; rows theta_0 ... theta_3):\n')
P('| | ' + ' | '.join(SC) + ' |\n|---|' + '---|' * len(SC))
if fr:
    for l in range(4):
        P('| Stage 3b (d) theta_%d | ' % l + ' | '.join('FAILED' if f['crashed'] or f['floor'] else 'ok' for f in fr[l]['flights']) + ' |')
for v in VAR:
    for l, fl in enumerate(flightflags(recs[v]['rounds'])):
        P('| %s theta_%d | ' % (v, l) + ' | '.join('FAILED' if fl[s] else 'ok' for s in SC) + ' |')
Path(HERE / 'report_stage3d.md').write_text('\n'.join(out) + '\n' + (HERE / 'report_tail.md').read_text() if (HERE / 'report_tail.md').exists() else '\n'.join(out) + '\n')
print('written', len(out))
