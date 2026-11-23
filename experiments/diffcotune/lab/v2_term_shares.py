#!/usr/bin/env python3
"""Protocol v2: share of each term on the existing round 0 flights of S1 / S3 (default gains, beta = calibration, no reweighting). Run with DIFFCOTUNE_METHOD=ours2 DIFFCOTUNE_CONTROLLER=S1|S3.
 cost:   real flights (merged round_0 files): mean ||p - p*||^2 [m^2] and mean tr(I - R*^T R)/2 [-] over the scored window, per scenario and the round mean (the number the round cost uses);
 sysId:  J_sysId parts p, v, q, w [m^2, m^2/s^2, -, rad^2/s^2] of the model rollout (default gains, psi = 0) against each real flight, per scenario (peer input = logged top path for A1 / A8, as in the desk step) and the mean over the 5 scenarios (what the fit minimises).
 tune:   the model task cost parts (p, att) at the same point (what the tuning step minimises), noise free.
Output: lab/results/v2_term_shares_<C>.json and a printed table."""
import dataclasses, json, os, sys
from pathlib import Path
import numpy as np
os.environ.setdefault('DIFFCOTUNE_METHOD', 'ours2'); os.environ.setdefault('DIFFCOTUNE_FIX_RD', '0'); os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'dryrun')); sys.path.insert(0, str(HERE))
import torch; torch.set_num_threads(2)
import loop_lib as L, usd_ingest as U, round as RD, costs as C, ingest, reference as R  # noqa
assert L.METHOD == 'ours2'
ctrl = L.CTRL; folder = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE.parent.parent / 'logs' / 'usd_merged' / 'rounds' / ctrl / 'round_0'
rot = dict(A1=None, A8=None, A4=-90.0, CIRCLE=-90.0, FIG8=-90.0); S = L.model(); rows = []
for s, _ in L.TUNING:
    cell = RD.cell_of(s, 'tuning', 0)
    g = U.ingest_flight(str(folder / (s + '.csv')), s, cell, top_from_log=s in ('A1', 'A8'), rotate_deg=rot[s])
    ref = g['ref']; x = torch.zeros(1, 10 + L.N_BETA, dtype=torch.float64)
    with torch.no_grad():
        out = S.rollout(ref, x[:, :10], x[:, 10:], compile=False)
        J, parts = C.j_sysid(out, ingest.on_grid(ingest.from_trace(g['trace']), out['t']), ref.scenario_s, parts=True)
        Jt, pt = C.j_task_att(out, ref.scenario_s, parts=True)
    rows.append(dict(scenario=s, real_pos_m2=g['pos_cost_m2'], real_att=g['att_cost'], real_cost=g['cost_m2'], real_rms3d_cm=g['rms3d_cm'],
                     sysid={k: float(v[0]) for k, v in parts.items()}, sysid_total=float(J[0]), task_model={k: float(v[0]) for k, v in pt.items()}))
mean = lambda f: float(np.mean([f(r) for r in rows]))
summ = dict(real_pos_m2=mean(lambda r: r['real_pos_m2']), real_att=mean(lambda r: r['real_att']), sysid={k: mean(lambda r, k=k: r['sysid'][k]) for k in 'pvqw'}, task_model={k: mean(lambda r, k=k: r['task_model'][k]) for k in ('p', 'att')})
summ['real_att_share'] = summ['real_att'] / (summ['real_att'] + summ['real_pos_m2']); tot = sum(summ['sysid'].values()); summ['sysid_share'] = {k: v / tot for k, v in summ['sysid'].items()}
tt = sum(summ['task_model'].values()); summ['task_model_share'] = {k: v / tt for k, v in summ['task_model'].items()}
json.dump(dict(controller=ctrl, folder=str(folder), per_scenario=rows, mean=summ), open(HERE / 'results' / ('v2_term_shares_%s.json' % ctrl), 'w'), indent=1)
print(ctrl, 'REAL flights, cost terms (mean over the scored window):')
for r in rows: print('  %-7s pos %.3e m2 (%.2f cm)  att %.3e  att share %.1f %%' % (r['scenario'], r['real_pos_m2'], r['real_rms3d_cm'], r['real_att'], 100 * r['real_att'] / (r['real_att'] + r['real_pos_m2'])))
print('  mean    pos %.3e  att %.3e  att share %.1f %%' % (summ['real_pos_m2'], summ['real_att'], 100 * summ['real_att_share']))
print(ctrl, 'J_sysId parts (model vs real flight, default gains / calibration beta):')
for r in rows: print('  %-7s p %.3e  v %.3e  q %.3e  w %.3e' % (r['scenario'], *[r['sysid'][k] for k in 'pvqw']))
print('  mean    p %.3e  v %.3e  q %.3e  w %.3e   shares %s' % (*[summ['sysid'][k] for k in 'pvqw'], {k: '%.1f %%' % (100 * v) for k, v in summ['sysid_share'].items()}))
print(ctrl, 'model task cost parts (tuning step, noise free):', {k: '%.3e' % v for k, v in summ['task_model'].items()}, {k: '%.1f %%' % (100 * v) for k, v in summ['task_model_share'].items()})
