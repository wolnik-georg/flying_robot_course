#!/usr/bin/env python3
"""Stage F2.3: forward check of fbl_model.py against frsim's FBL WITH sensor noise (frsim noise=True, model fed by fbl_model.frsim_noise with the same seed), after the Stage 3d procedure
(dryrun/stage3d/noise_forward_check.py, noise_gate.py; docs/diffcotune_spec.md section 13): per cell the 3-D RMS over the scoring window of frsim and model; gate = our choice of Stage 3d:
  * a cell is "stable" when frsim itself moves < 0.5 % in 3-D RMS under a 1e-6 relative change of the thrust-law gain (same gains, same seed); there the model must agree within 0.5 %;
  * in the other (chaotic) cells the model must show the failure too: crash flag (tilt >= 45 deg in the scoring window, the FBL limit), or 3-D RMS > 3 x the frsim RMS of the same scenario and seed at preset-1 gains
    (our reading of 'round-1 value' of Stage 3d, which had round-1 gains; there is no FBL round 1 yet). For a preset-1 cell that is unstable only the crash flag counts.
Cells: 5 tuning scenarios (A1, A8, CIRCLE, FIG8, A4 cell 1) x 3 gain rows (preset 1; two random sets within +-30 % of preset 1, rng seeds 1 and 2) x 2 noise seeds (1000, 1001) at plant P1 (as loop_lib P1), per variant.
  python3 fbl/noise_forward_check_fbl.py [--variants base rom both] [--procs 1] [--out FILE.json]"""
import argparse, json, os, sys, time
from dataclasses import replace
from multiprocessing import Pool
from pathlib import Path
os.environ.setdefault('OMP_NUM_THREADS', '2')
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent))
import numpy as np, torch
import reference as R
import fbl_model as F
import forward_check_fbl as FC
from frsim import config
from frsim.episode import EpisodeConfig, run_episode

SCEN = ['A1', 'A8', 'CIRCLE', 'FIG8', 'A4']; SEEDS = (1000, 1001); TOL = 0.5
P1 = FC.ROWS['P1']
GAIN_ROWS = {'preset1': np.ones(6), 'gains1': np.random.default_rng(1).uniform(0.7, 1.3, 6), 'gains2': np.random.default_rng(2).uniform(0.7, 1.3, 6)}


def _fr(a):
    var, scen, cell, gains, seed, eps = a
    os.environ['FRSIM_FBL_VARIANT'] = var
    p = config.plant_with(P1); p = replace(p, thrust_law_gain_n=p.thrust_law_gain_n * (1 + eps))
    return run_episode(EpisodeConfig('FBL', scen, cell, gains, 'nominal', seed, plant=p, noise=True, record_state=True)).state_trace


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--variants', nargs='+', default=['base', 'rom', 'both']); ap.add_argument('--procs', type=int, default=1); ap.add_argument('--out', default=str(HERE / 'noise_forward_check_fbl.json'))
    a = ap.parse_args(); rows = []; psi = FC.psi_of(config.plant_with(P1))
    for var in a.variants:
        keys = [(s, g, sd) for s in SCEN for g in GAIN_ROWS for sd in SEEDS]
        jobs = []
        for s, g, sd in keys:
            gains = {k: float(v * f) for k, v, f in zip(F.GAIN_NAMES, F.GAIN_DEFAULT, GAIN_ROWS[g])}; cell = FC.CELLS[s][1]
            jobs += [(var, s, cell, gains, sd, 0.0), (var, s, cell, gains, sd, 1e-6)]
        t0 = time.time()
        with Pool(a.procs) as pool: res = pool.map(_fr, jobs, chunksize=1)
        print('variant %s: frsim %d episodes %.0f s' % (var, len(jobs), time.time() - t0), flush=True)
        fr = {k: (res[2 * i], res[2 * i + 1]) for i, k in enumerate(keys)}
        for s in SCEN:
            ref = R.build(s, FC.CELLS[s][1]); ks = [k for k in keys if k[0] == s]; B = len(ks); t1 = time.time()
            phi = torch.as_tensor(np.stack([np.log(GAIN_ROWS[g]) for _, g, _ in ks]), dtype=torch.float64); ps = torch.as_tensor(np.tile(psi, (B, 1)), dtype=torch.float64)
            noise = np.stack([F.frsim_noise(sd, ref.n_ticks) for _, _, sd in ks], 1)
            with torch.no_grad(): out = F.rollout(ref, phi, ps, variant=var, noise=noise)
            print('  %s %s model %d rows %.0f s' % (var, s, B, time.time() - t1), flush=True)
            for j, k in enumerate(ks):
                tr, tr6 = fr[k]; n = min(len(tr), len(out['t'])); tt = tr[:n, 0]
                rf = FC.win_rms(tt, tr[:n, 1:4], tr[:n, 14:17], ref.scenario_s); rf6 = FC.win_rms(tr6[:n, 0], tr6[:n, 1:4], tr6[:n, 14:17], ref.scenario_s)
                rm = FC.win_rms(tt, out['x'][:n, j].numpy(), tr[:n, 14:17], ref.scenario_s)
                win = (tt > 0) & (tt <= ref.scenario_s); tilt_m = float(F.tilt_deg(out['q'][:n, j])[win].max()); tilt_f = float(F.tilt_deg(torch.as_tensor(tr[:n, 7:11]))[win].max())
                rows.append(dict(variant=var, scenario=s, gains=k[1], seed=k[2], rms_frsim_cm=rf, rms_frsim_perturbed_cm=rf6, sens_change_pct=100 * abs(rf6 - rf) / rf, rms_model_cm=rm, rel_pct=100 * abs(rm - rf) / rf,
                                 crash_frsim=bool(tilt_f >= F.TILT_CRASH_DEG), crash_model=bool(tilt_m >= F.TILT_CRASH_DEG), max_pos_diff_mm=1000 * float(np.max(np.linalg.norm(tr[:n, 1:4] - out['x'][:n, j].numpy(), axis=-1)))))
    base_rms = {(r['variant'], r['scenario'], r['seed']): r['rms_frsim_cm'] for r in rows if r['gains'] == 'preset1'}
    ok = True
    for r in rows:
        stable = r['sens_change_pct'] < TOL
        if stable: r['passed'] = bool(r['rel_pct'] <= TOL and r['crash_frsim'] == r['crash_model']); r['rule'] = 'stable: model within 0.5 %, same crash flag'
        else:
            lim = 3 * base_rms[(r['variant'], r['scenario'], r['seed'])]; r['passed'] = bool(r['crash_model'] or (r['gains'] != 'preset1' and r['rms_model_cm'] > lim)); r['rule'] = 'chaotic: model shows failure (crash %s, RMS %.1f cm vs 3 x preset-1 %.1f cm)' % (r['crash_model'], r['rms_model_cm'], lim)
        r['stable'] = bool(stable); ok &= r['passed']
    print('%-5s %-7s %-8s %5s %9s %9s %8s %9s %-6s %5s %5s %s' % ('var', 'scen', 'gains', 'seed', 'frsim cm', 'model cm', 'rel %', 'sens %', 'stable', 'crF', 'crM', 'pass'))
    for r in rows: print('%-5s %-7s %-8s %5d %9.3f %9.3f %8.4f %9.4f %-6s %5s %5s %s' % (r['variant'], r['scenario'], r['gains'], r['seed'], r['rms_frsim_cm'], r['rms_model_cm'], r['rel_pct'], r['sens_change_pct'], r['stable'], r['crash_frsim'], r['crash_model'], 'yes' if r['passed'] else 'NO'))
    for v in a.variants: rv = [r for r in rows if r['variant'] == v]; print('%s: %d of %d cells pass; stable cells %d; max rel (stable) %.4f %%' % (v, sum(r['passed'] for r in rv), len(rv), sum(r['stable'] for r in rv), max([r['rel_pct'] for r in rv if r['stable']] or [0])))
    print('NOISE FORWARD CHECK FBL:', 'PASS' if ok else 'FAIL'); json.dump(dict(passed=bool(ok), table=rows), open(a.out, 'w'), indent=1)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
