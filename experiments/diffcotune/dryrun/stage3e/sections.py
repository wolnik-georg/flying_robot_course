"""Stage 3e: markdown sections (per plant) from results/loop_<P>.json and results/eval_<P>.json. Every number comes from those files."""
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import loop_lib as L  # noqa
R = HERE / 'results'; R3D = HERE.parent / 'stage3d' / 'results'
J = lambda p: json.load(open(p))
SC = [s for s, _ in L.TUNING]; GAINS = L.GAIN_NAMES
failed = lambda e: bool(e['crashed'] or e.get('floor', False))


def role_mean(ev, role, key='rms3d_cm'):
    return float(np.mean([e[key] for e in ev if e['role'] == role]))


def flags(rounds):
    return [{f['scenario']: bool(f['crashed'] or f['floor']) for f in r['flights']} for r in rounds]


def berr(psi):
    return [100 * (np.exp(a - b) - 1) for a, b in zip(psi, L.PSI_TRUE)]


def plant_section(pl, run_to_run=False):
    rec = J(R / ('loop_%s.json' % pl)); ev = J(R / ('eval_%s.json' % pl)); rd = rec['rounds']; out = []; P = lambda *a: out.append(' '.join(str(x) for x in a))
    n = len(rd)
    P('### Real cost per round (%s; 3-D RMS in cm, mean of the 4 tuning flights)\n' % pl)
    P('| Gain set | real 3-D RMS cm | z RMS cm | failed flights (tilt or floor) | flight seeds | minutes (round) |\n|---|---|---|---|---|---|')
    for r in rd:
        bad = [f['scenario'] for f in r['flights'] if f['crashed'] or f['floor']]
        P('| theta_%d | %.2f | %.2f | %d%s | %s | %.1f |' % (r['round'], r['real']['rms3d_cm'], r['real']['z_rms_cm'], len(bad), (' (' + ', '.join(bad) + ')') if bad else '', [f['seed'] for f in r['flights']], r['seconds_round'] / 60))
    P('\nKeep-best (lowest real cost over the flown sets): theta_%d (%.2f cm).\n' % (rec['best']['round'], 100 * rec['best']['cost'] ** 0.5))
    P('Gain-limit hits (projected Adam steps of 51 evaluations per gain; order %s):\n' % ' '.join(GAINS))
    for r in rd:
        if 'tune' in r:
            P('- tuning after round %d: noise seeds %s, hits %s, best epoch %s' % (r['round'], r['noise_seeds'], r['tune']['hits'], r['tune']['best_epoch']))
    P('\n### beta error after each fit (relative error of each value against the P1 truth, %; no gate)\n')
    P('| Round | thrust gain | motor lag | downwash | plume |\n|---|---|---|---|---|')
    for r in rd:
        if 'fit' in r:
            P('| fit of round %d | ' % r['round'] + ' | '.join('%+.1f' % x for x in berr(r['psi_after_fit'])) + ' |')
    # evaluation
    P('\n### Final evaluation (real drone = frsim with noise, evaluation seeds 2000-2004; 3-D RMS cm / z RMS cm / failed flights of 5; mean over seeds)\n')
    cells = [(s, c) for s, c in L.TUNING] + L.TEST; hdr = ['A1', 'A8', 'CIRCLE', 'FIG8', 'A4 cell 1', 'A4 cell 2', 'OVAL']
    P('| Gain set | ' + ' | '.join(hdr) + ' | tuning mean | test mean |\n|---|' + '---|' * (len(hdr) + 2))
    names = {'theta_0': 'theta_0 (default)', 'theta_best': 'theta_best (round %d)' % ev['best_round'], 'theta_%d' % (n - 1): 'theta_%d (last flown)' % (n - 1)}
    for k in ('theta_0', 'theta_best', 'theta_%d' % (n - 1)):
        e_ = ev['eval'][k]; cs = []
        for s, c in cells:
            es = [e for e in e_ if e['scenario'] == s and e['cell'] == c]
            cs.append('%.2f / %.2f / %d' % (np.mean([e['rms3d_cm'] for e in es]), np.mean([e['z_rms_cm'] for e in es]), sum(failed(e) for e in es)))
        P('| %s | ' % names[k] + ' | '.join(cs) + ' | %.2f | %.2f |' % (role_mean(e_, 'tuning'), role_mean(e_, 'test')))
    # criteria
    r = [x['real']['rms3d_cm'] for x in rd]; fl = flags(rd)
    a = all(r[l] <= r[1] + 1.0 for l in range(2, n)); bad = []
    for l in range(1, n):
        for s in SC:
            if fl[l][s] and not (fl[0][s] and fl[1][s]):
                bad.append('theta_%d %s' % (l, s))
    d0 = ev['eval']['theta_0']; last = ev['eval']['theta_%d' % (n - 1)]; bst = ev['eval']['theta_best']
    cc = [role_mean(e, ro) < role_mean(d0, ro) for e in (bst, last) for ro in ('tuning', 'test')]
    yn = lambda t: 'yes' if t else 'no'
    P('\n### Pass criteria (%s)\n' % pl)
    P('| Criterion | result |\n|---|---|')
    P('| a. rounds 2-%d <= round 1 + 1 cm (round 1 = %.2f, limit %.2f) | **%s** (rounds 2-%d: %s) |' % (n - 1, r[1], r[1] + 1, yn(a), n - 1, ', '.join('%.2f' % x for x in r[2:])))
    P('| b. no flown set after theta_0 fails (tilt or floor) a scenario that theta_0 or theta_1 passed | **%s** (%s) |' % (yn(not bad), ('violations: ' + ', '.join(bad)) if bad else 'none'))
    P('| c. theta_best and theta_%d beat theta_0 on the tuning and the test scenarios (mean over 5 seeds) | **%s** (theta_best %.2f / %.2f, theta_%d %.2f / %.2f, theta_0 %.2f / %.2f, tuning / test) |' % (
        n - 1, yn(all(cc)), role_mean(bst, 'tuning'), role_mean(bst, 'test'), n - 1, role_mean(last, 'tuning'), role_mean(last, 'test'), role_mean(d0, 'tuning'), role_mean(d0, 'test')))
    P('| d. beta per round | reported above, no gate |')
    P('\nFloor / tilt rule per flown gain set (failed = tilt or floor):\n')
    P('| | ' + ' | '.join(SC) + ' |\n|---|' + '---|' * len(SC))
    for l, f in enumerate(fl):
        P('| theta_%d | ' % l + ' | '.join('FAILED' if f[s] else 'ok' for s in SC) + ' |')
    if run_to_run:
        old = J(R3D / 'loop_AB.json')['rounds']
        P('\n### Run-to-run spread, rounds 0-3 (this run against the Stage 3d A + B run; different flight and noise seeds; no gate)\n')
        P('| Round | real cost cm, Stage 3d | real cost cm, this run |\n|---|---|---|')
        for l in range(4):
            P('| theta_%d | %.2f | %.2f |' % (l, old[l]['real']['rms3d_cm'], rd[l]['real']['rms3d_cm']))
        P('\nbeta error after the fit (%, thrust gain / motor lag / downwash / plume):\n')
        P('| Fit of round | Stage 3d | this run |\n|---|---|---|')
        for l in range(3):
            P('| %d | %s | %s |' % (l, ' / '.join('%+.1f' % x for x in berr(old[l]['psi_after_fit'])), ' / '.join('%+.1f' % x for x in berr(rd[l]['psi_after_fit']))))
        P('\ntheta factors against the default gains (order %s):\n' % ' '.join(GAINS))
        P('| Gain set | Stage 3d | this run |\n|---|---|---|')
        for l in range(1, 4):
            P('| theta_%d | %s | %s |' % (l, ' '.join('%.2f' % np.exp(x) for x in old[l]['phi']), ' '.join('%.2f' % np.exp(x) for x in rd[l]['phi'])))
    return '\n'.join(out) + '\n'


if __name__ == '__main__':
    pl = sys.argv[1]; txt = plant_section(pl, run_to_run=(pl == 'P1')); (R / ('pass_%s.md' % pl)).write_text(txt); print(txt)
