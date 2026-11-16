#!/usr/bin/env python3
"""Stage 8b: SAFETY PREVIEW of a new gain set (a check, not a method change: it never changes the gains).

After the tuning step of a round the new gains theta are flown in frsim on the 5 tuning scenarios with the round's FITTED beta as the plant, sensor noise on, 2 noise seeds (the evaluation seeds 2000, 2001), and the abort rules of the
protocol are applied to every simulated flight: crash (frsim tilt crash), tilt (|roll| or |pitch| > 25 deg for 0.5 s continuously), floor (z < 0.25 m after the start of the scenario part). The same is done for the best flown gain set so far, for comparison.
  PREVIEW PASSED          no abort event in any scenario / seed of the new set, or only events the best flown set shows in the same scenario too (listed)
  PREVIEW FAILED: <scenario, seed, reason>   an abort event of the new set that the best flown set does not show: do NOT fly this set without asking; default action in the lab: fly the best flown set again.
Information (never changes PASSED / FAILED; the threshold is a parameter, not a decided rule): "NOTE worse" lines where the new set's mean 3-D RMS in a scenario exceeds 1.5 x that of the best flown set and by more than 2 cm.
Usage inside round.py (pool workers that already exist: `mapper = pool.pool.map`), or stand-alone replay of a stored dry run:
  python3 preview.py --controller S0 --loop dryrun/stage8/results/loop_S0_ours.json [--serial]  -> prints per round the preview of theta_l (made with the beta after the fit of round l-1 and the best set flown before round l) next to the real result of round l"""
import argparse, json, os, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'dryrun'))
PREVIEW_SEEDS = (2000, 2001); TILT_DEG = 25.0; TILT_S = 0.5; FLOOR_Z = 0.25; WORSE_FACTOR = 1.5; WORSE_ABS_CM = 2.0


def fitted_plant(L, psi):
    """frsim plant overrides of the fitted beta (log factors relative to the calibration): the same convention as loop_lib.PLANTS (thrust, downwash, plume as factors 'x..', motor lag and rpm delay absolute)."""
    from frsim import config
    cal = config.CALIBRATED_PLANT; psi = np.asarray(psi, float)
    over = {'thrust_gain': 'x%.10g' % np.exp(psi[0]), 'motor_lag_ms': float(cal.motor_lag_ms * np.exp(psi[1])), 'downwash_scale': 'x%.10g' % np.exp(psi[2]), 'plume_scale': 'x%.10g' % np.exp(psi[3])}
    if len(psi) > 4: over['rpm_delay_ms'] = float(cal.rpm_delay_ms * np.exp(psi[4])); over['rpm_delay_fractional'] = 1          # S1 / S3: the model's rpm delay is fractional
    return dict(overrides=over, variant='nominal')


def abort_events(L, trace, stable, scenario_s):
    """list of abort events of one simulated flight (protocol rules)."""
    ev = []
    if not stable: ev.append('crash')
    t = trace[:, 0]; m = (t > 0) & (t <= scenario_s)
    q = trace[:, 7:11]; x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    zb = np.stack([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)], -1); tilt = np.degrees(np.arccos(np.clip(zb[:, 2], -1, 1)))
    over = (tilt > TILT_DEG) & m; dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.01; run = best = 0
    for o in over:
        run = run + 1 if o else 0; best = max(best, run)
    if best * dt >= TILT_S: ev.append('tilt')
    if (trace[m, 3] < FLOOR_Z).any(): ev.append('floor')
    return ev


def run(L, mapper, phi_new, psi, phi_best, seeds=PREVIEW_SEEDS):
    """Preview of phi_new (and phi_best) on the tuning scenarios of loop_lib at the plant of the fitted beta psi. mapper(fn, jobs) as Pool.map (or the builtin map)."""
    import reference as R
    t0 = time.time(); plant = fitted_plant(L, psi); sets = {'new': np.asarray(phi_new, float), 'best': np.asarray(phi_best, float)}
    jobs = [(s, c, L.gains_of(ph), plant, seed) for nm, ph in sets.items() for s, c in L.TUNING for seed in seeds]
    res = list(mapper(L.worker_fly, jobs)); out = {nm: {} for nm in sets}; k = 0
    for nm in sets:
        for s, c in L.TUNING:
            ss = R.build(s, c).scenario_s; per = []
            for seed in seeds:
                tr, stable, tilt = res[k]; k += 1; cst, cz = L.flight_cost(tr, ss)
                per.append(dict(seed=seed, rms3d_cm=100 * cst ** 0.5, z_rms_cm=100 * cz ** 0.5, events=abort_events(L, tr, stable, ss), tilt_max_deg=tilt))
            out[nm][s] = per
    fails = []; notes = []; same = []
    for s, _ in L.TUNING:
        ev_best = {e for p in out['best'][s] for e in p['events']}
        for p in out['new'][s]:
            for e in p['events']:
                (same if e in ev_best else fails).append((s, p['seed'], e))
        rn = np.mean([p['rms3d_cm'] for p in out['new'][s]]); rb = np.mean([p['rms3d_cm'] for p in out['best'][s]])
        if rn > WORSE_FACTOR * rb and rn - rb > WORSE_ABS_CM: notes.append((s, rn, rb))
    verdict = 'PREVIEW PASSED' if not fails else 'PREVIEW FAILED: ' + '; '.join('%s seed %d: %s' % f for f in fails)
    mean = lambda nm: float(np.mean([np.mean([p['rms3d_cm'] for p in out[nm][s]]) for s, _ in L.TUNING]))
    return dict(verdict=verdict, passed=not fails, fails=fails, events_shared_with_best=same, notes_worse=notes, mean_rms_cm=dict(new=mean('new'), best=mean('best')), per=out, seeds=list(seeds), seconds=time.time() - t0)


def text(pv, L):
    lines = [pv['verdict'], 'predicted 3-D RMS (mean over the %d tuning scenarios and %d seeds): new set %.2f cm, best flown set %.2f cm (%.0f s)' % (len(L.TUNING), len(pv['seeds']), pv['mean_rms_cm']['new'], pv['mean_rms_cm']['best'], pv['seconds'])]
    for s, _ in L.TUNING:
        f = lambda nm: ' | '.join('%.1f cm%s' % (p['rms3d_cm'], (' ' + '+'.join(p['events'])) if p['events'] else '') for p in pv['per'][nm][s])
        lines.append('  %-7s new: %-36s best: %s' % (s, f('new'), f('best')))
    if pv['events_shared_with_best']: lines.append('  events the best flown set shows too (not counted): ' + ', '.join('%s seed %d %s' % e for e in pv['events_shared_with_best']))
    for s, rn, rb in pv['notes_worse']: lines.append('  NOTE worse: %s predicted %.1f cm against %.1f cm of the best flown set (information only)' % (s, rn, rb))
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--controller', required=True); ap.add_argument('--loop', required=True); ap.add_argument('--serial', action='store_true', help='no pool: builtin map (slow, one process)')
    ap.add_argument('--rounds', type=int, nargs='*'); ap.add_argument('--out'); a = ap.parse_args()
    os.environ['DIFFCOTUNE_CONTROLLER'] = a.controller
    import loop_lib as L
    rec = json.load(open(a.loop)); R_ = rec['rounds']; results = []
    pool = None if a.serial else L.Pool(len(L.TUNING)); mapper = (lambda fn, jobs: map(fn, jobs)) if a.serial else (lambda fn, jobs: pool.pool.map(fn, jobs, chunksize=1))
    try:
        for l in range(1, len(R_)):
            if a.rounds and l not in a.rounds: continue
            prev = R_[:l]; bi = int(np.argmin([r['real']['cost'] for r in prev])); phi_best = prev[bi]['phi']; psi = R_[l - 1]['psi_after_fit']
            pv = run(L, mapper, R_[l]['phi'], psi, phi_best); real = R_[l]['real']['rms3d_cm']; real_best = prev[bi]['real']['rms3d_cm']
            print('=== round %d: theta_%d (flown in round %d) against best flown set round %d | real cost %.2f cm (best flown %.2f cm) | actual events in the real flights: %s' % (l, l, l, bi, real, real_best,
                  [(f['scenario'], 'crash' if f['crashed'] else 'floor') for f in R_[l]['flights'] if f['crashed'] or f['floor']]), flush=True); print(text(pv, L), flush=True)
            results.append(dict(round=l, best_round=bi, real_cm=real, real_best_cm=real_best, preview=pv))
    finally:
        if pool: pool.close()
    if a.out: json.dump(results, open(a.out, 'w'), indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))


if __name__ == '__main__':
    main()
