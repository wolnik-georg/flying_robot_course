#!/usr/bin/env python3
"""Stage 3c Part B: desk checks of the drift of (d) on P1 (full50best, J_sysId = p + v + attitude, learning rate 0.01, as Stage 3b).
Stages (one per call, each < 1 h):
  flights  : re-fly the six gain sets theta_0 ... theta_5 of (d) (seeds as in the loop) -> per-scenario real cost (check 4) and the share of each
             part of J_sysId in the value and in the gradient with respect to the four beta at (theta_l, beta_l) (check 3)
  fit_truth: beta fit started at the TRUE P1 beta on the flights of round 3 and round 4 (check 2)
  oracle   : theta tuned for one inner loop in the model with the TRUE beta (no fit) from theta_0 / theta_2 / theta_3, evaluated on the real drone (check 1)
Usage (nice -n 19): python3 dryrun/stage3c/partb.py <stage>"""
import json, pickle, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import loop_lib as L  # noqa: E402
import partb_workers as W  # noqa: E402

RES = HERE / 'results'; D = json.load(open(HERE.parent / 'stage3b' / 'results' / 'full_P1_cotune.json'))
STATES = ('p', 'v', 'q')


def say(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)


def get_flights(pool):
    f = RES / 'flights_P1_d.pkl'
    if f.exists():
        return pickle.load(open(f, 'rb'))
    fl = [L.fly_set(pool, np.array(r['phi']), L.P1, L.TUNING, [10 * l + i for i in range(4)]) for l, r in enumerate(D['rounds'])]
    for l, r in enumerate(D['rounds']):
        assert abs(L.round_cost(fl[l])['rms3d_cm'] - r['real']['rms3d_cm']) < 1e-6, 'flights of round %d are not reproduced' % l
    pickle.dump(fl, open(f, 'wb')); return fl


if __name__ == '__main__':
    stage = sys.argv[1]; pool = L.Pool(4, states=STATES)
    try:
        fl = get_flights(pool)
        if stage == 'flights':
            say('flights reproduced (real cost per round equal to the loop record)')
            out = dict(per_scenario=[], parts=[])
            for l in range(6):
                out['per_scenario'].append([dict(scenario=s, rms3d_cm=100 * f['cost'] ** .5, z_rms_cm=100 * f['z_cost'] ** .5, tilt=f['tilt']) for (s, c), f in zip(L.TUNING, fl[l])])
                x = np.concatenate([D['rounds'][l]['phi'], D['rounds'][l]['psi']])
                res = pool.pool.map(W.worker_partgrad, [(s, c, x, fl[l][i]['trace']) for i, (s, c) in enumerate(L.TUNING)], chunksize=1)
                vals = {k: [r[0][k] for r in res] for k in 'pvqw'}; grads = {k: [r[1][k] for r in res] for k in 'pvqw'}
                out['parts'].append(dict(round=l, values=vals, grads=grads))
                say('round %d per scenario 3-D RMS cm %s  z %s' % (l, [round(100 * f['cost'] ** .5, 1) for f in fl[l]], [round(100 * f['z_cost'] ** .5, 1) for f in fl[l]]))
                say('   J parts mean: %s' % {k: round(float(np.mean(v)), 4) for k, v in vals.items()})
                for i, (s, _) in enumerate(L.TUNING):
                    say('   %-7s parts p %.4f v %.4f q %.4f w %.4f' % (s, vals['p'][i], vals['v'][i], vals['q'][i], vals['w'][i]))
                g = {k: np.mean(grads[k], axis=0) for k in 'pvq'}; tot = sum(g.values())
                say('   dJ(pva)/dbeta (mean over scenarios) = %s ; parts p %s v %s q %s' % (np.round(tot, 4), np.round(g['p'], 4), np.round(g['v'], 4), np.round(g['q'], 4)))
            L.jdump(out, RES / 'check3_check4.json')
        elif stage == 'predict':      # model task cost per scenario (m^2 -> cm RMS) of theta_l with the true beta and with the fitted beta, against the real flight
            out = []
            for l in range(6):
                phi = np.array(D['rounds'][l]['phi']); row = dict(round=l, real=[100 * f['cost'] ** .5 for f in fl[l]])
                for name, psi in (('true_beta', L.PSI_TRUE), ('beta0', np.zeros(4)), ('beta_of_the_round_that_tuned_it', np.array(D['rounds'][l - 1]['psi_after_fit']) if l > 0 else np.zeros(4))):
                    x = np.concatenate([phi, psi])
                    res = pool.pool.map(L.worker_rollout, [('task_fwd', s, c, x, None) for s, c in L.TUNING], chunksize=1)
                    row[name] = [100 * r[0] ** .5 for r in res]
                out.append(row)
                say('theta_%d  real %s | model true beta %s | model beta0 %s | model beta that tuned it %s' % (l, np.round(row['real'], 1).tolist(), np.round(row['true_beta'], 1).tolist(), np.round(row['beta0'], 1).tolist(), np.round(row['beta_of_the_round_that_tuned_it'], 1).tolist()))
            L.jdump(out, RES / 'check_predict.json')
        elif stage == 'fit_truth':
            out = []
            for l in (3, 4):
                traces = {i: [fl[l][i]['trace']] for i in range(4)}
                x0 = np.concatenate([D['rounds'][l]['phi'], L.PSI_TRUE])
                x, rec = L.adam_inner(lambda xx: pool.rollouts('sysid', xx, traces), x0, np.arange(10, 14), 0.01, K2=50, stop=False, best=True, label='fit from truth, flights of round %d' % l)
                err = lambda v: (np.exp(v - L.PSI_TRUE) - 1) * 100
                say('flights of round %d: start at the truth; best epoch %d; J %.4f -> %.4f; beta error %% at epoch 0/10/20/30/40/50: %s' % (l, rec['best_epoch'], rec['J0'], rec['J_best'], [np.round(err(np.array(r['x'])), 1).tolist() for r in rec['log'] if r['k'] in (0, 10, 20, 30, 40, 50)]))
                out.append(dict(round=l, best_epoch=rec['best_epoch'], J0=rec['J0'], J_best=rec['J_best'], log=rec['log']))
                L.jdump(out, RES / 'check2_fit_truth.json')
        elif stage == 'oracle':
            out = []
            psi_true = L.PSI_TRUE
            for name, phi0 in (('theta_0', np.zeros(10)), ('theta_2 of (d)', np.array(D['rounds'][2]['phi'])), ('theta_3 of (d)', np.array(D['rounds'][3]['phi']))):
                t0 = time.time(); x0 = np.concatenate([phi0, psi_true])
                x, rec = L.adam_inner(lambda xx: pool.rollouts('task', xx, None), x0, np.arange(10), 0.01, K2=50, stop=False, best=True, label='oracle tune from ' + name)
                ev0 = L.evaluate(pool, phi0, L.P1); ev1 = L.evaluate(pool, x[:10], L.P1)
                say('%s: model J %.5f -> %.5f (best epoch %d), %.0f s' % (name, rec['J0'], rec['J_best'], rec['best_epoch'], time.time() - t0))
                out.append(dict(start=name, phi0=phi0.tolist(), phi=x[:10].tolist(), J0=rec['J0'], J_best=rec['J_best'], best_epoch=rec['best_epoch'], eval_start=ev0, eval_tuned=ev1))
                L.jdump(out, RES / 'check1_oracle.json')
    finally:
        pool.close()
