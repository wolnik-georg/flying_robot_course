#!/usr/bin/env python3
"""Stage 6 benchmark: the round-0 dry test of round.py (4 frsim flights of the dry-run plant written as fake merged uSD files, round 0 of the controller) with the speed levers of Stage 6 as flags.
Reports desk seconds split into ingest / pool start / fit / tune, the first-epoch excess (the compile share), peak memory and load (sampled every 5 s), and the difference of theta_1 and beta after the fit to a REFERENCE run
(a previous dry_test6 result of the same controller with the default settings, --ref results/dry6_<C>_ref.json; for S0 also to lab/results/dry_test.json). Identity gate: relative difference <= 1e-10 (summation order may differ).
  python3 dry_test6.py --controller S0 --tag ref                                        (default settings = reference)
  python3 dry_test6.py --controller S0 --tag w8 --tune-workers 8 --ref results/dry6_S0_ref.json [--warm] [--fit-threads 4 --tune-threads 4] [--fit-workers 4]
Env: DIFFCOTUNE_CONTROLLER set from --controller; TORCHINDUCTOR_FX_GRAPH_CACHE=1 in the environment gives lever f (compiled-graph cache). Run with nice -n 19 through job_runner."""
import argparse, json, os, shutil, sys, threading, time
from pathlib import Path
import numpy as np
if '--controller' in sys.argv: os.environ['DIFFCOTUNE_CONTROLLER'] = sys.argv[sys.argv.index('--controller') + 1]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'dryrun'))
import round_fast as RD  # noqa  (sets DIFFCOTUNE_GRAD_MODE from its per-controller table before loop_lib is imported)
import loop_lib as L  # noqa
import fake_usd as F  # noqa
from frsim.scenarios import tuning_cells  # noqa

if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--controller', default='S0'); ap.add_argument('--tag', required=True); ap.add_argument('--ref', default=None)
    ap.add_argument('--fit-workers', type=int, default=4); ap.add_argument('--tune-workers', type=int, default=4); ap.add_argument('--fit-threads', type=int, default=2); ap.add_argument('--tune-threads', type=int, default=2)
    ap.add_argument('--warm', action='store_true', help='lever c: compile every graph in every worker at pool start (the pool start time is reported separately, as the compile happens while the flights are flown)'); ap.add_argument('--epochs', type=int, default=None); a = ap.parse_args()
    C = L.CTRL; PLANT = L.PLANTS['P1RD' if L.N_BETA == 5 else 'P1']
    mem = dict(min_avail=1e9, max_load=0.0, stop=False)
    def sampler():
        while not mem['stop']:
            for line in open('/proc/meminfo'):
                if line.startswith('MemAvailable:'): mem['min_avail'] = min(mem['min_avail'], int(line.split()[1]) / 1048576.0)
            mem['max_load'] = max(mem['max_load'], float(open('/proc/loadavg').read().split()[0])); time.sleep(5)
    mem['start_avail'] = [int(l.split()[1]) / 1048576.0 for l in open('/proc/meminfo') if l.startswith('MemAvailable:')][0]; mem['start_load'] = float(open('/proc/loadavg').read().split()[0])
    threading.Thread(target=sampler, daemon=True).start()
    DRY = HERE / 'dry6' / ('%s_%s' % (C, a.tag)); shutil.rmtree(DRY, ignore_errors=True); DRY.mkdir(parents=True); RD.init_state(DRY / C)
    if a.epochs:
        f = DRY / C / 'round_0' / 'state.json'; st = json.load(open(f)); st['settings']['epochs'] = a.epochs; json.dump(st, open(f, 'w'), indent=1)
    cache = HERE / 'dry6' / ('%s_flights' % C); gains = L.gains_of(np.zeros(10))
    if not (cache / 'flights.json').exists():
        cache.mkdir(parents=True, exist_ok=True); man = []
        for i, (s, c) in enumerate(L.TUNING):
            tr, stable = F.fly_500hz(s, c, gains, PLANT, 100 + i); name = '%s_dry_merged_usd.csv' % s; F.write_merged(cache / name, tr, s, c)
            idx = [k for k, cc in enumerate(tuning_cells(s)) if cc == c][0]; man.append(dict(file=name, scenario=s, cell=idx, role='tuning', attempt=1, rotate_deg=0, lag_s=0))
        json.dump(man, open(cache / 'flights.json', 'w'), indent=1)
    t0 = time.time()
    perf = dict(fit_workers=a.fit_workers, fit_threads=a.fit_threads, tune_workers=a.tune_workers, tune_threads=a.tune_threads, warm=a.warm)
    RD.SETTINGS['n_rounds'] = 5
    rep = RD.run_round(0, cache, DRY / C, top='planned', strict=False, perf=perf)
    mem['stop'] = True; es = rep['epoch_seconds']
    sec = lambda l: dict(first=l[0], median_rest=float(np.median(l[1:])) if len(l) > 1 else None, total=float(np.sum(l)), n=len(l))
    out = dict(controller=C, tag=a.tag, perf=perf, grad_mode=L.GRAD_MODE, seconds_total=rep['seconds_total'], seconds_ingest=rep['seconds_ingest'], seconds_pool_start=rep.get('seconds_pool_start'), seconds_pool_start_tune=rep.get('seconds_pool_start_tune'),
               seconds_fit=rep['seconds_fit'], seconds_tune=rep['seconds_tune'], fit_epochs=sec(es['fit']), tune_epochs=sec(es['tune']),
               compile_excess_fit=es['fit'][0] - float(np.median(es['fit'][1:])), compile_excess_tune=es['tune'][0] - float(np.median(es['tune'][1:])),
               mem_available_start_gb=mem['start_avail'], mem_available_min_gb=mem['min_avail'], load_start=mem['start_load'], load_max=mem['max_load'], real_cm=rep['real']['rms3d_cm'],
               phi_next=rep['phi_next'], psi_after_fit=rep['psi_after_fit'], tune_J0=rep['tune']['J0'], tune_J_best=rep['tune']['J_best'], fit_J_best=rep['fit']['J_best'], hits=rep['tune']['hits'], env_fx_graph_cache=os.environ.get('TORCHINDUCTOR_FX_GRAPH_CACHE'))
    ref = json.load(open(a.ref)) if a.ref else None
    if ref:
        d = lambda x, y: float(np.max(np.abs(np.array(x) - np.array(y)) / np.maximum(np.abs(np.array(y)), 1e-300)))
        out['rel_diff_phi_next'] = d(out['phi_next'], ref['phi_next']); out['rel_diff_psi_after_fit'] = d(out['psi_after_fit'], ref['psi_after_fit']); out['abs_diff_phi_next'] = float(np.max(np.abs(np.array(out['phi_next']) - np.array(ref['phi_next'])))); out['abs_diff_psi'] = float(np.max(np.abs(np.array(out['psi_after_fit']) - np.array(ref['psi_after_fit']))))
        out['identity_gate_1e-10'] = bool(out['rel_diff_phi_next'] <= 1e-10 and out['rel_diff_psi_after_fit'] <= 1e-10)
    if C == 'S0' and (HERE / 'results' / 'dry_test.json').exists():
        r0 = json.load(open(HERE / 'results' / 'dry_test.json'))['round_0']; out['abs_diff_phi_next_vs_dry_test_json'] = float(np.max(np.abs(np.array(out['phi_next']) - np.array(r0['phi_next_loop_lib']))))
    (HERE / 'results').mkdir(exist_ok=True); json.dump(out, open(HERE / 'results' / ('dry6_%s_%s.json' % (C, a.tag)), 'w'), indent=1); print(json.dumps({k: v for k, v in out.items() if k not in ('phi_next', 'psi_after_fit', 'hits')}, indent=1))
