"""Stage 4e: markdown sections of T0, the real-data check, T1 and T2 from results/*.json (every number comes from those files).
Usage: python3 sections_4e.py t0 | real | run <tag>      (tag = T1 | T2_raw | T2_sg110 ...)   -> prints markdown and writes results/sections_<name>.md"""
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import loop_lib as L  # noqa
R = HERE / 'results'; R3E = HERE.parent / 'stage3e' / 'results'
J = lambda p: json.load(open(p))
SC = [s for s, _ in L.TUNING]; HDR = ['A1', 'A8', 'CIRCLE', 'FIG8', 'A4 cell 1', 'A4 cell 2', 'OVAL']
failed = lambda e: bool(e['crashed'] or e.get('floor', False))
berr = lambda psi: [100 * (np.exp(a - b) - 1) for a, b in zip(psi, L.PSI_TRUE)]
role_mean = lambda ev, role: float(np.mean([e['rms3d_cm'] for e in ev if e['role'] == role]))   # equal weight per record = equal weight per cell (5 seeds each)
yn = lambda t: 'yes' if t else 'no'


def t0():
    d = J(R / 't0_identifiability.json'); o = []; P = lambda *a: o.append(' '.join(str(x) for x in a))
    P('| Variant | thrust gain factor (err %) | motor lag (err %) | downwash (err %) | plume (err %) | best epoch | J_sysId start -> best |\n|---|---|---|---|---|---|---|')
    tr = d[0]['truth']
    P('| truth | %.3f | %.3f | %.3f | %.3f | | |' % tuple(tr))
    for r in d:
        P('| %s | ' % r['variant'] + ' | '.join('%.3f (%+.1f)' % (f, e) for f, e in zip(r['factors'], r['err_pct'])) + ' | %d | %.4g -> %.4g |' % (r['best_epoch'], r['J0'], r['J_best']))
    P('\nParts of the full J_sysId (p, v, q, w; m^2, m^2/s^2, -, rad^2/s^2; mean over 4 scenarios x 2 flights) at the start and at the best iterate, share of each part of the four:\n')
    P('| Variant | | p | v | q | w | w share |\n|---|---|---|---|---|---|---|')
    for r in d:
        for lab, pt in (('start', r['parts_start']), ('end', r['parts_end'])):
            tot = sum(pt.values())
            P('| %s | %s | %.5f (%.1f %%) | %.5f (%.1f %%) | %.5f (%.1f %%) | %.5f (%.1f %%) | %.1f %% |' % (r['variant'], lab, *[x for k in 'pvqw' for x in (pt[k], 100 * pt[k] / tot)], 100 * pt['w'] / tot))
    return '\n'.join(o) + '\n'


def real():
    d = J(R / 't0_realdata.json'); o = []; P = lambda *a: o.append(' '.join(str(x) for x in a))
    P('| Gyro | p | v | q | w | sum | w share |\n|---|---|---|---|---|---|---|')
    for k, m in d['mean'].items():
        P('| %s | %.4f | %.4f | %.4f | %.3f | %.3f | %.1f %% |' % (k, m['p'], m['v'], m['q'], m['w'], m['total'], 100 * m['share_w']))
    P('\nPer flight, w share (raw -> sg110) and J_w (raw -> sg110, rad^2/s^2):\n')
    P('| Flight | raw w | sg110 w | w share raw | w share sg110 |\n|---|---|---|---|---|')
    for f in d['flights']:
        a, b = f['variants']['raw'], f['variants']['sg110']
        P('| %s | %.3f | %.3f | %.1f %% | %.1f %% |' % (f['folder'], a['parts']['w'], b['parts']['w'], 100 * a['share_w'], 100 * b['share_w']))
    return '\n'.join(o) + '\n'


def run(tag):
    rec = J(R / ('loop_%s.json' % tag)); ev = J(R / ('eval_%s.json' % tag)); ref = J(R3E / 'loop_P1.json'); rev = J(R / 'ref_eval_B1_5rounds.json')['eval']
    ev0 = J(R3E / 'eval_P1.json')['eval']['theta_0']; rd = rec['rounds']; rr = ref['rounds'][:5]; n = len(rd); o = []; P = lambda *a: o.append(' '.join(str(x) for x in a))
    P('#### Real cost per round (3-D RMS cm, mean of the 4 tuning flights; reference = B1 of Stage 3e, same flight seeds)\n')
    P('| Gain set | this run | B1 | failed flights this run (tilt or floor) | B1 failed | minutes (round) |\n|---|---|---|---|---|---|')
    for l, r in enumerate(rd):
        bad = [f['scenario'] for f in r['flights'] if failed(f)]; badr = [f['scenario'] for f in rr[l]['flights'] if failed(f)]
        P('| theta_%d | %.2f | %.2f | %s | %s | %.1f |' % (l, r['real']['rms3d_cm'], rr[l]['real']['rms3d_cm'], ', '.join(bad) or '0', ', '.join(badr) or '0', r['seconds_round'] / 60))
    P('\nKeep-best over the flown sets: theta_%d (%.2f cm); B1 over rounds 0-4: theta_4 (8.13 cm).\n' % (rec['best']['round'], 100 * rec['best']['cost'] ** 0.5))
    P('#### Inner loops (epochs = cost evaluations)\n')
    P('| Round | loop | epochs | stop reason / best epoch | J at start | J at end (returned iterate) | gain-limit hits |\n|---|---|---|---|---|---|---|')
    for r in rd:
        for key, nm in (('fit', 'fit beta'), ('tune', 'tune theta')):
            if key in r:
                x = r[key]; jend = x['J_end'] if x.get('J_end') is not None else x['J_best']
                why = x['reason'] if x.get('paper_stop') else 'best epoch %d of %d (no stop)' % (x['best_epoch'], x['epochs'] - 1)
                P('| %d | %s | %d | %s | %.5g | %.5g | %s |' % (r['round'], nm, x['epochs'], why, x['J0'], jend, x['hits'] if key == 'tune' else '-'))
    P('\n#### beta error after each fit (% against the P1 truth: thrust gain / motor lag / downwash / plume)\n')
    P('| Fit of round | this run | B1 |\n|---|---|---|')
    for l, r in enumerate(rd):
        if 'fit' in r:
            P('| %d | %s | %s |' % (l, ' / '.join('%+.1f' % x for x in berr(r['psi_after_fit'])), ' / '.join('%+.1f' % x for x in berr(rr[l]['psi_after_fit']))))
    P('\n#### Final evaluation (evaluation seeds 2000-2004; 3-D RMS cm / z RMS cm / failed flights of 5)\n')
    cells = [(s, c) for s, c in L.TUNING] + L.TEST
    P('| Gain set | ' + ' | '.join(HDR) + ' | tuning mean | test mean |\n|---|' + '---|' * (len(HDR) + 2))
    sets = [('theta_0 (default)', ev0), ('theta_best of this run (round %d)' % ev['best_round'], ev['eval']['theta_best']), ('B1 theta_best over rounds 0-4 (theta_4)', rev)]
    for nm, e_ in sets:
        cs = []
        for s, c in cells:
            es = [e for e in e_ if e['scenario'] == s and e['cell'] == c]
            cs.append('%.2f / %.2f / %d' % (np.mean([e['rms3d_cm'] for e in es]), np.mean([e['z_rms_cm'] for e in es]), sum(failed(e) for e in es)))
        P('| %s | ' % nm + ' | '.join(cs) + ' | %.2f | %.2f |' % (role_mean(e_, 'tuning'), role_mean(e_, 'test')))
    r_ = [x['real']['rms3d_cm'] for x in rd]; fl = [{f['scenario']: failed(f) for f in r['flights']} for r in rd]
    c1 = (all(r_[l] <= r_[1] + 1.0 for l in range(2, n)) if n >= 5 else None)
    bad = ['theta_%d %s' % (l, s) for l in range(1, n) for s in SC if fl[l][s] and not (fl[0][s] and fl[1][s])]
    bst = ev['eval']['theta_best']; c3 = [role_mean(bst, ro) < role_mean(ev0, ro) for ro in ('tuning', 'test')]
    c4 = [role_mean(bst, ro) - role_mean(rev, ro) <= 1.0 for ro in ('tuning', 'test')]
    P('\n#### Pass criteria 1-4 (against B1, rounds 0-4)\n')
    P('| Criterion | result |\n|---|---|')
    P('| 1. rounds 2-4 <= round 1 + 1 cm (round 1 = %.2f, limit %.2f) | **%s** (%s) |' % (r_[1], r_[1] + 1, 'not assessable: %d rounds flown' % n if c1 is None else yn(c1), ', '.join('%.2f' % x for x in r_[2:]) or 'none flown'))
    P('| 2. no flown set after theta_0 fails (tilt or floor) a scenario that theta_0 or theta_1 passed | **%s** (%s) |' % (yn(not bad), ('violations: ' + ', '.join(bad)) if bad else 'none'))
    P('| 3. theta_best beats theta_0 on the tuning and the test scenarios (5 seeds) | **%s** (theta_best %.2f / %.2f, theta_0 %.2f / %.2f, tuning / test) |' % (yn(all(c3)), role_mean(bst, 'tuning'), role_mean(bst, 'test'), role_mean(ev0, 'tuning'), role_mean(ev0, 'test')))
    P('| 4. not worse than the B1 reference by more than 1 cm on the tuning mean and the test mean | **%s** (difference %+.2f / %+.2f cm, tuning / test) |' % (yn(all(c4)), role_mean(bst, 'tuning') - role_mean(rev, 'tuning'), role_mean(bst, 'test') - role_mean(rev, 'test')))
    allp = (c1 is True) and not bad and all(c3) and all(c4)
    P('\nRun passes 1-4: **%s**.%s' % (yn(allp), '' if c1 is not None else ' (criterion 1 could not be assessed because the run stopped after round 1: %s)' % rec.get('halt')))
    return '\n'.join(o) + '\n'


if __name__ == '__main__':
    k = sys.argv[1]
    txt = t0() if k == 't0' else real() if k == 'real' else run(sys.argv[2])
    (R / ('sections_%s.md' % (k if k != 'run' else sys.argv[2]))).write_text(txt); print(txt)
