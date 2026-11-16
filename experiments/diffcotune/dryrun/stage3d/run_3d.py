#!/usr/bin/env python3
"""Stage 3d runs (S0, P1). Usage (system python3, nice -n 19):
  run_3d.py confirm            # one short run: round 0 (fit + tune) and round 1 flight of the Stage 3b loop (d); must reproduce 19.80 cm and theta_1
  run_3d.py variant A|B|AB     # rounds l = 0 ... 3, 3 updates, --skip-last-fit, Stage 3b settings otherwise; then evaluation of theta_best and theta_3
  run_3d.py evalref            # evaluation of theta_3 of the Stage 3b loop (d), same 5 seeds (the Stage 3b run itself is not rerun)
Variant A: the tuning cost is the mean over 2 fixed noise seeds per round (5000 + 2 l, 5001 + 2 l; not flight, not evaluation seeds). B: gain limit 0.25.
Results in dryrun/stage3d/results/."""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import loop_lib as L  # noqa
RES = HERE / 'results'; RES.mkdir(exist_ok=True)
B3 = HERE.parent / 'stage3b' / 'results'


def say(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)


if __name__ == '__main__':
    mode = sys.argv[1]
    pool = L.Pool(4, states=('p', 'v', 'q'))
    try:
        if mode == 'confirm':
            rec = L.run_loop(pool, L.P1, 'cotune', 0.01, n_rounds=1, extra_final_flight=True, log=say, stop=False, best=True)
            ref = json.load(open(B3 / 'full_P1_cotune.json'))
            same_phi1 = np.array(rec['rounds'][1]['phi']); ref_phi1 = np.array(ref['rounds'][1]['phi'])
            out = dict(rec=rec, round0_rms_new=rec['rounds'][0]['real']['rms3d_cm'], round0_rms_stage3b=ref['rounds'][0]['real']['rms3d_cm'],
                       round1_rms_new=rec['rounds'][1]['real']['rms3d_cm'], round1_rms_stage3b=ref['rounds'][1]['real']['rms3d_cm'],
                       phi1_max_abs_diff=float(np.abs(same_phi1 - ref_phi1).max()), psi_after_fit_max_abs_diff=float(np.abs(np.array(rec['rounds'][0]['psi_after_fit']) - np.array(ref['rounds'][0]['psi_after_fit'])).max()))
            L.jdump(out, RES / 'confirm_round0.json'); say({k: v for k, v in out.items() if k != 'rec'})
        elif mode == 'variant':
            v = sys.argv[2]; kw = {}
            if 'A' in v:
                kw['noise_seeds_of'] = lambda l: (5000 + 2 * l, 5001 + 2 * l)
            if 'B' in v:
                kw['gain_limit'] = 0.25
            save = lambda r: L.jdump(r, RES / ('loop_%s.json' % v))
            rec = L.run_loop(pool, L.P1, 'cotune', 0.01, n_rounds=4, log=say, stop=False, best=True, skip_last_fit=True, on_round=save, **kw)
            save(rec)
            phis = {'theta_best': np.array(rec['best']['phi']), 'theta_3': np.array(rec['rounds'][3]['phi'])}
            ev = {k: L.evaluate(pool, p, L.P1) for k, p in phis.items()}
            L.jdump(dict(phis={k: p.tolist() for k, p in phis.items()}, best_round=rec['best']['round'], eval=ev), RES / ('eval_%s.json' % v)); say(v, 'evaluated')
        elif mode == 'evalref':          # default gains, theta_best (= round 1) and theta_3 of the Stage 3b loop (d), 5 seeds, with the floor flag
            ref = json.load(open(B3 / 'full_P1_cotune.json'))
            phis = {'default': np.zeros(10), 'theta_best': np.array(ref['best']['phi']), 'theta_3': np.array(ref['rounds'][3]['phi'])}
            L.jdump(dict(phis={k: p.tolist() for k, p in phis.items()}, best_round=ref['best']['round'], eval={k: L.evaluate(pool, p, L.P1) for k, p in phis.items()}), RES / 'eval_ref.json'); say('evalref done')
        elif mode == 'floorref':         # re-fly the four gain sets of the Stage 3b loop (d) with its flight seeds: per-flight floor / tilt flags (deterministic, costs must repeat)
            ref = json.load(open(B3 / 'full_P1_cotune.json')); out = []
            for l in range(4):
                fl = L.fly_set(pool, np.array(ref['rounds'][l]['phi']), L.P1, L.TUNING, [10 * l + i for i in range(len(L.TUNING))])
                rc = L.round_cost(fl)
                out.append(dict(round=l, rms3d_cm=rc['rms3d_cm'], stage3b_rms3d_cm=ref['rounds'][l]['real']['rms3d_cm'], flights=[dict(scenario=L.TUNING[i][0], cost=f['cost'], z_cost=f['z_cost'], floor=f['floor'], crashed=f['crashed'], tries=f['tries']) for i, f in enumerate(fl)]))
                say('floorref round', l, rc['rms3d_cm'], ref['rounds'][l]['real']['rms3d_cm'])
            L.jdump(out, RES / 'floor_ref_rounds.json')
    finally:
        pool.close()
