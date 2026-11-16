#!/usr/bin/env python3
"""Stage 4e T0, real-data check (no fit): J_sysId parts (p, v, q, w) on the 10 S0 flights of 23 Oct (uSD logs, as Stage 4a C1 / Stage 4b gyro check: default gains, beta_0, logged top path
for A1 / A8) with the logged gyro raw and smoothed by the zero-phase 5th-order Savitzky-Golay filter (windows 70 / 110 / 210 ms, converted to odd sample counts at the trace's row rate);
the model's body rates are not filtered. Share of each part in the sum p + v + q + w. Output results/t0_realdata.json."""
import csv, json, os, sys
from pathlib import Path
import numpy as np
os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
HERE = Path(__file__).resolve().parent; LAB = HERE.parents[1] / 'lab'
sys.path.insert(0, str(HERE.parent.parent)); sys.path.insert(0, str(LAB)); sys.path.insert(0, str(HERE))
import torch
torch.set_num_threads(2)
import usd_ingest as U, ingest, costs as C, s0_model as S  # noqa
import test_ingest_oct23 as T  # noqa
import gyro_variants as G  # noqa

WINS = (None, 70, 110, 210)          # None = raw logged gyro

if __name__ == '__main__':
    rows = [r for r in csv.DictReader(open(T.PF)) if r['bank'] == 'bank_unseen_S0_2026-10-23']; out = []
    for r in rows:
        sc = r['scenario']; cell = T.cell_of(sc, r); f = T.BANK / r['folder'] / (r['folder'] + '_merged_usd.csv')
        g = U.ingest_flight(f, sc, cell, top_from_log=sc in ('A1', 'A8')); tr = np.array(g['trace'], float); ref = g['ref']; dt = float(np.median(np.diff(tr[:, 0])))
        with torch.no_grad():
            o = S.rollout(ref, np.zeros((1, 10)), np.zeros((1, 4)), compile=False)
        row = dict(folder=r['folder'], scenario=sc, dt=dt, variants={})
        for ms in WINS:
            t2 = tr.copy()
            if ms: t2[:, 11:14] = G.sg(tr[:, 11:14], ms, dt)
            tgt = ingest.on_grid(ingest.from_trace(t2), o['t']); _, parts = C.j_sysid(o, tgt, ref.scenario_s, parts=True); parts = {k: float(v[0]) for k, v in parts.items()}
            tot = sum(parts.values()); row['variants']['raw' if ms is None else 'sg%d' % ms] = dict(parts=parts, total=tot, share_w=parts['w'] / tot, samples=None if ms is None else G.win_samples(ms, dt))
        out.append(row)
        print('%-24s dt %.4f  ' % (r['folder'], dt) + ' | '.join('%s p %.4f v %.4f q %.4f w %.3f (w share %.0f %%)' % (k, v['parts']['p'], v['parts']['v'], v['parts']['q'], v['parts']['w'], 100 * v['share_w']) for k, v in row['variants'].items()), flush=True)
    mean = {k: {pk: float(np.mean([r['variants'][k]['parts'][pk] for r in out])) for pk in 'pvqw'} for k in out[0]['variants']}
    for k, m in mean.items():
        m['total'] = sum(m.values()); m['share_w'] = m['w'] / m['total']
    print('MEAN over %d flights:' % len(out)); [print(' ', k, {a: round(b, 4) for a, b in m.items()}) for k, m in mean.items()]
    json.dump(dict(flights=out, mean=mean), open(HERE / 'results' / 't0_realdata.json', 'w'), indent=1)
