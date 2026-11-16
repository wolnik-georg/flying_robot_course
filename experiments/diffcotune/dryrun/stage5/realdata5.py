#!/usr/bin/env python3
"""Stage 5 B3 real-data check (no fit): J_sysId parts (p, v, q; w for information) of the model of one controller with beta_0 (calibration; S1 / S3: rpm delay 4.05 ms) and the default gains on the 23 Oct
flights of that controller (uSD merged files, logged top path for A1 / A8, as Stage 4a / 4b / 4e). The controller of a flight = the `strategy` column of
experiments/results/unseen_block_2026-10-23/per_flight.csv, which `experiments/analysis/unseen_block_analysis.py::strategy` derives FROM THE LOG (INDI estimate active in the log, NN prediction applied);
the meta.json files do not distinguish (all say controller 9, ctrl_mode 0, study readback None). Folder names are not used. Banks: S0 bank_unseen_S0, S1 bank_unseen_S1, S2 bank_unseen_S3S2, S3 bank_unseen_S3 and bank_unseen_S3S2.
Output results/realdata_<C>.json. Usage (nice -n 19): DIFFCOTUNE_CONTROLLER=S2 python3 realdata5.py"""
import csv, json, os, sys
from pathlib import Path
import numpy as np
os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
HERE = Path(__file__).resolve().parent; TOP = HERE.parents[0]; LAB = HERE.parents[1] / 'lab'
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(LAB)); sys.path.insert(0, str(TOP))
import torch
torch.set_num_threads(2)
import loop_lib as L  # noqa
import usd_ingest as U, ingest, costs as C  # noqa
import test_ingest_oct23 as T  # noqa

if __name__ == '__main__':
    ctrl = L.CTRL; M = L.model(); rows = [r for r in csv.DictReader(open(T.PF)) if r['strategy'] == ctrl]; out = []
    for r in rows:
        sc = r['scenario']; cell = T.cell_of(sc, r); f = T.BANK.parent / r['bank'] / r['folder'] / (r['folder'] + '_merged_usd.csv')
        g = U.ingest_flight(f, sc, cell, top_from_log=sc in ('A1', 'A8'))
        if not g.get('usable', False):
            out.append(dict(folder=r['folder'], bank=r['bank'], scenario=sc, usable=False, gate=g['gate']['reasons'])); print(r['folder'], 'not usable', g['gate']['reasons'], flush=True); continue
        tr = g['trace']; ref = g['ref']
        with torch.no_grad():
            o = M.rollout(ref, np.zeros((1, 10)), np.zeros((1, M.N_BETA)), compile=False)
        tgt = ingest.on_grid(ingest.from_trace(tr), o['t']); _, parts = C.j_sysid(o, tgt, ref.scenario_s, parts=True); parts = {k: float(v[0]) for k, v in parts.items()}
        pva = parts['p'] + parts['v'] + parts['q']
        out.append(dict(folder=r['folder'], bank=r['bank'], scenario=sc, cell=r['cell'], vbat_start=float(r['vbat_start']), usable=True, parts=parts, J_pva=pva, share_w_of_all=parts['w'] / (pva + parts['w']),
                        real_rms3d_cm=g['rms3d_cm'], flags=g['flags'])); print('%-24s %s %-4s real %.2f cm  p %.4f v %.4f q %.4f | w %.3f' % (r['folder'], r['bank'][12:20], sc, g['rms3d_cm'], parts['p'], parts['v'], parts['q'], parts['w']), flush=True)
    ok = [x for x in out if x['usable']]
    mean = {k: float(np.mean([x['parts'][k] for x in ok])) for k in 'pvqw'}; mean['J_pva'] = float(np.mean([x['J_pva'] for x in ok]))
    per_sc = {s: {k: float(np.mean([x['parts'][k] for x in ok if x['scenario'] == s])) for k in 'pvqw'} for s in sorted({x['scenario'] for x in ok})}
    print('MEAN of %d flights:' % len(ok), {k: round(v, 4) for k, v in mean.items()}); print('per scenario:', {s: {k: round(v, 4) for k, v in d.items()} for s, d in per_sc.items()})
    json.dump(dict(controller=ctrl, n=len(ok), n_listed=len(out), flights=out, mean=mean, per_scenario=per_sc), open(HERE / 'results' / ('realdata_%s.json' % ctrl), 'w'), indent=1)
