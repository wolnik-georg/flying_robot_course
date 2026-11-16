#!/usr/bin/env python3
"""Tables and plots of the Stage 3b dry run from dryrun/stage3b/results/*.json (python3 dryrun/stage3b/report_s3b.py); Stage 3 numbers are read from dryrun/results/."""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent          # dryrun/stage3b
RES = HERE / 'results'
OLD = HERE.parent / 'results'              # Stage 3 results (read only)
BETA = ['thrust-law gain', 'motor lag', 'downwash scale', 'plume scale']
TRUE = np.log([0.8, 1.5, 1.3, 0.7])
GAINS = ['Pxy', 'Pz', 'Dxy', 'Dz', 'Iz', 'KRxy', 'KRz', 'Kwxy', 'Kwz', 'KIxy']
out = []
P = lambda *a: out.append(' '.join(str(x) for x in a))


def load(name):
    f = RES / name
    return json.load(open(f)) if f.exists() else None


# ---------------------------------------------------------------------------------------------------------- B1 identifiability
if (RES / 'b1_identifiability.json').exists():
    P('## B1 identifiability (P1, noisy flights of theta0, 2 noise seeds, 50 epochs, best iterate; error = fitted / true - 1)\n')
    P('| states | learning rate | thrust gain | motor lag | downwash | plume | best epoch | parts share start (p / v / q / w, % of the full 4-part cost) | end |\n|---|---|---|---|---|---|---|---|---|')
    for x in json.load(open(RES / 'b1_identifiability.json')):
        a, b = x['parts_start'], x['parts_end']; ta, tb = sum(a.values()), sum(b.values())
        P('| %s | %g | %s | %d | %s | %s |' % (x['states'], x['lr'], ' | '.join('%+.1f %%' % v for v in x['err_pct']), x['best_epoch'], ' / '.join('%.0f' % (100 * a[k] / ta) for k in 'pvqw'), ' / '.join('%.0f' % (100 * b[k] / tb) for k in 'pvqw')))
    P('\nChosen: pva (position + velocity + attitude).\n')

# ---------------------------------------------------------------------------------------------------------- B1 pilot
rows = []
for f in sorted(RES.glob('pilot_P1_lr*.json')):
    d = json.load(open(f)); r = d['rounds']
    c = [x['real']['rms3d_cm'] for x in r]
    ep = [(x.get('fit', {}).get('epochs'), x.get('fit', {}).get('reason'), x.get('tune', {}).get('epochs'), x.get('tune', {}).get('reason')) for x in r]
    rows.append((d['lr'], c, ep, r[-1]['psi_after_fit'] if r else None))
if rows:
    P('## B2 pilot (P1, 3 learning rates, rounds 0 and 1 update, the real cost after round 2 = flights of theta_2)\n')
    P('| learning rate | real 3-D RMS cm round 0 / 1 / 2 | fit epochs (stop) round 0, 1 | tune epochs (stop) round 0, 1 |\n|---|---|---|---|')
    for lr, c, ep, _ in rows:
        P('| %g | %s | %s | %s |' % (lr, ' / '.join('%.2f' % x for x in c), '; '.join('%s (%s)' % (e[0], e[1]) for e in ep[:2]), '; '.join('%s (%s)' % (e[2], e[3]) for e in ep[:2])))
    if all(len(c) == 3 for _, c, _, _ in rows):
        best = min(rows, key=lambda r: r[1][2]); P('\nLowest real cost after round 2: learning rate %g (%.2f cm).\n' % (best[0], best[1][2]))

# ------------------------------------------------------------------------------------------------------- B4 per plant
for pn in ('P1', 'P2'):
    co, to, st, ev = load('full_%s_cotune.json' % pn), load('full_%s_tune_only.json' % pn), load('full_%s_sysid_then_tune.json' % pn), load('full_%s_eval.json' % pn)
    if co is None:
        continue
    P('\n## %s (%s)\n' % (pn, 'thrust gain x0.8, lag 45 ms, downwash x1.3, plume x0.7' + (' + state delay 10 ms' if pn == 'P2' else '')))
    R = co['rounds']
    # 1 beta over the rounds
    psi = np.array([[0, 0, 0, 0]] + [r['psi_after_fit'] for r in R])
    fig, ax = plt.subplots(1, 4, figsize=(16, 3.4))
    for j in range(4):
        ax[j].plot(range(len(psi)), np.exp(psi[:, j]), 'o-', label='fitted'); ax[j].axhline(np.exp(TRUE[j]), color='r', ls='--', label='true')
        ax[j].set_title(BETA[j] + ' (factor to the calibration)'); ax[j].set_xlabel('after round (0 = start)'); ax[j].grid(alpha=.3)
    ax[0].legend(); fig.suptitle('%s: beta over the rounds (DiffCoTune)' % pn); fig.tight_layout(); fig.savefig(HERE / ('beta_%s.png' % pn), dpi=90); plt.close(fig)
    err = np.exp(psi[-1]) / np.exp(TRUE) - 1
    P('**1. beta** after round 5 (fit of round 5): ' + ', '.join('%s %.4f (true %.2f, error %+.1f %%)' % (BETA[j], np.exp(psi[-1, j]), np.exp(TRUE[j]), 100 * err[j]) for j in range(4)) + '. Plot: `beta_%s.png`.\n' % pn)
    # 2 learning curve
    fig, ax = plt.subplots(figsize=(6.5, 4))
    ax.plot([r['real']['rms3d_cm'] for r in R], 'o-', label='(d) DiffCoTune')
    if to: ax.plot([r['real']['rms3d_cm'] for r in to['rounds']], 's-', label='(b) tune only')
    if ev:
        m = lambda k: np.mean([e['rms3d_cm'] for e in ev['eval'][k] if e['role'] == 'tuning'])
        ax.axhline(m('default'), color='k', ls=':', label='(a) default (5 seeds)')
        if 'sysid_then_tune' in ev['eval']: ax.axhline(m('sysid_then_tune'), color='g', ls='--', label='(c) sysId then tune (final, 5 seeds)')
    ax.set_xlabel('round (flight of theta_l)'); ax.set_ylabel('real 3-D RMS, mean of 4 tuning scenarios [cm]'); ax.legend(); ax.grid(alpha=.3); ax.set_title('%s: real cost per round' % pn)
    fig.tight_layout(); fig.savefig(HERE / ('cost_%s.png' % pn), dpi=90); plt.close(fig)
    P('**2. real cost per round** (3-D RMS cm, 1 flight per scenario): (d) ' + ' / '.join('%.2f' % r['real']['rms3d_cm'] for r in R) + (('; (b) ' + ' / '.join('%.2f' % r['real']['rms3d_cm'] for r in to['rounds'])) if to else '') + '. Plot: `cost_%s.png`.\n' % pn)
    # 3 final table
    if ev:
        P('**3. final evaluation** (theta_best of each method on the real drone, 5 noise seeds per cell; mean over all 5 seeds (crashed flights included, their RMS up to the crash) of 3-D RMS / z RMS in cm, crashed seeds in brackets):\n')
        cells = []
        for e in ev['eval']['default']:
            k = (e['scenario'], json.dumps(e['cell'], sort_keys=True), e['role'])
            if k not in cells: cells.append(k)
        names = [('default', '(a) default'), ('tune_only', '(b) tune only'), ('sysid_then_tune', '(c) sysId then tune'), ('cotune', '(d) DiffCoTune')]
        P('| cell | role | ' + ' | '.join(n for _, n in names) + ' |\n|---|---|' + '---|' * len(names))
        for sc, cj, role in cells:
            cell = json.loads(cj); lab = sc + (' dz%s off%s' % (cell.get('dz'), cell.get('offset')) if sc == 'A4' else '')
            vals = []
            for k, _ in names:
                es = [e for e in ev['eval'][k] if e['scenario'] == sc and json.dumps(e['cell'], sort_keys=True) == cj]
                nc = sum(e['crashed'] for e in es)
                vals.append('%.2f / %.2f' % (np.mean([e['rms3d_cm'] for e in es]), np.mean([e['z_rms_cm'] for e in es])) + (' (%d crashed)' % nc if nc else ''))
            P('| %s | %s | %s |' % (lab, role, ' | '.join(vals)))
        for role in ('tuning', 'test'):
            vals = []
            for k, _ in names:
                es = [e for e in ev['eval'][k] if e['role'] == role]; nc = sum(e['crashed'] for e in es)
                vals.append('%.2f%s' % (np.mean([e['rms3d_cm'] for e in es]), ' (%d crashed)' % nc if nc else ''))
            P('| **mean %s cells** | | %s |' % (role, ' | '.join(vals)))
        P('')
    # 4 theta
    phi = np.array([r['phi'] for r in R]); fig, ax = plt.subplots(figsize=(7, 4))
    for j in range(10): ax.plot(phi[:, j], 'o-', label=GAINS[j])
    ax.set_xlabel('round (gains flown)'); ax.set_ylabel('log-factor relative to the default'); ax.legend(ncol=2, fontsize=7); ax.grid(alpha=.3); ax.set_title('%s: theta over the rounds (DiffCoTune)' % pn)
    fig.tight_layout(); fig.savefig(HERE / ('theta_%s.png' % pn), dpi=90); plt.close(fig)
    if ev:
        b = np.array(ev['phis']['cotune']); P('**4. theta**: theta_best = round %d of the loop; factors to the defaults: ' % co['best']['round'] + ', '.join('%s %.2f' % (g, np.exp(v)) for g, v in zip(GAINS, b)) + '. Plot: `theta_%s.png`.\n' % pn)
    # 5 inner loops
    P('**5. inner loops** (epochs used, stop reason; Jn = cost / cost at the start of the loop, last epoch):\n')
    P('| round | fit epochs | fit stop | fit Jn | tune epochs | tune stop | tune Jn | best epoch fit / tune | seconds fit + tune |\n|---|---|---|---|---|---|---|---|---|')
    for r in R:
        f, t = r.get('fit'), r.get('tune')
        P('| %d | %s | %s | %s | %s | %s | %s | %s | %.0f |' % (r['round'], f['epochs'] if f else '-', f['reason'] if f else '-', '%.3f' % f['log'][-1]['Jn'] if f else '-', t['epochs'] if t else '-', t['reason'] if t else '-', '%.3f' % t['log'][-1]['Jn'] if t else '-', '%s / %s' % (f['best_epoch'] if f else '-', t['best_epoch'] if t else '-'), r.get('seconds_fit', 0) + r.get('seconds_tune', 0)))
    P('')
    # 6 Adam on weak parameters
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True); step = 0; fd_rows = []
    for r in R:
        f = r.get('fit')
        if not f: continue
        for e in f['log']:
            ax[0].plot(step, e['x'][1], 'b.'); ax[0].plot(step, e['x'][3], 'r.'); ax[1].plot(step, e['g'][1], 'b.'); ax[1].plot(step, e['g'][3], 'r.')
            if 'fd_check' in e:
                for c in e['fd_check']:
                    fd_rows.append((r['round'], e['k'], 'motor lag' if c['param'] == 11 else 'plume scale', c['autograd'], c['fd']))
            step += 1
        ax[0].axvline(step - .5, color='0.8')
    ax[0].set_ylabel('log-factor (blue motor lag, red plume)'); ax[1].set_ylabel('autograd gradient of J_sysId'); ax[1].set_xlabel('fit epoch (all rounds, grey lines = round ends)')
    ax[0].grid(alpha=.3); ax[1].grid(alpha=.3); ax[1].set_yscale('symlog', linthresh=1e-3); fig.suptitle('%s: Adam on the weak parameters' % pn); fig.tight_layout(); fig.savefig(HERE / ('adam_weak_%s.png' % pn), dpi=90); plt.close(fig)
    P('**6. Adam on motor lag and plume scale**: epoch-by-epoch trace in `adam_weak_%s.png`; autograd against central finite difference (h = 1e-6) at the checked epochs:\n' % pn)
    P('| round | epoch | parameter | autograd | finite difference | same sign | relative difference |\n|---|---|---|---|---|---|---|')
    for rd, k, nm, a, f in fd_rows:
        P('| %d | %d | %s | %+.4e | %+.4e | %s | %.1f %% |' % (rd, k, nm, a, f, 'yes' if a * f > 0 else 'NO', 100 * abs(a - f) / abs(f) if f else float('nan')))
    P('')
    # side by side with Stage 3
    o_co, o_ev = (json.load(open(OLD / ('full_%s_cotune.json' % pn))), json.load(open(OLD / ('full_%s_eval.json' % pn))))
    po = np.array(o_co['rounds'][-1]['psi_after_fit']); pn_ = psi[-1]
    P('**Stage 3 against Stage 3b**\n')
    P('| item | Stage 3 | Stage 3b |\n|---|---|---|')
    P('| beta error after round 5 (thrust / lag / downwash / plume, %%) | %s | %s |' % (' / '.join('%+.1f' % (100 * v) for v in np.exp(po - TRUE) - 1), ' / '.join('%+.1f' % (100 * v) for v in np.exp(pn_ - TRUE) - 1)))
    P('| (d) real 3-D RMS per round (cm) | %s | %s |' % (' / '.join('%.1f' % r['real']['rms3d_cm'] for r in o_co['rounds']), ' / '.join('%.1f' % r['real']['rms3d_cm'] for r in R)))
    if ev:
        for role in ('tuning', 'test'):
            for k, nm in (('default', '(a)'), ('tune_only', '(b)'), ('sysid_then_tune', '(c)'), ('cotune', '(d)')):
                mm = lambda E: np.mean([e['rms3d_cm'] for e in E['eval'][k] if e['role'] == role])
                P('| final %s cells %s (cm) | %.2f | %.2f |' % (role, nm, mm(o_ev), mm(ev)))
    rr = [r['real']['rms3d_cm'] for r in R]
    P('\nReal cost rises after round 1 (Stage 3b, (d)): %s (round 1 = %.2f cm; later rounds %s).\n' % ('yes' if max(rr[2:]) > rr[1] else 'no', rr[1], ' / '.join('%.2f' % v for v in rr[2:])))
    # 7 wall time
    P('**7. wall time per round (fit + tune, 4 scenarios in parallel processes)**: ' + ', '.join('%.1f min' % ((r.get('seconds_fit', 0) + r.get('seconds_tune', 0)) / 60) for r in R) + '; mean %.1f min.\n' % (np.mean([(r.get('seconds_fit', 0) + r.get('seconds_tune', 0)) / 60 for r in R])))
if (HERE / 'pass_criteria_P1.md').exists():
    out.append((HERE / 'pass_criteria_P1.md').read_text())
(HERE / 'report_stage3b.md').write_text('\n'.join(out))
print('\n'.join(out))
