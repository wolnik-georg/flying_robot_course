#!/usr/bin/env python3
"""Stage 4a C1 test: the 10 S0 flights of 23 Oct (experiments/logs/bank_unseen_S0_2026-10-23: A1 x3, A4 x4, A8 x3; default gains, study cells):
ingest, quality gate, reference check, flags, real cost, and J_sysId of the S0 model with beta_0 and the default gains per state part
(no fit of beta). The noise floor per part is estimated from the log itself: mean squared residual of the logged channel from its 0.1 s moving average
(an upper bound: fast true dynamics are in it too). Output lab/results/ingest_oct23.json."""
import csv, json, os, sys
from pathlib import Path
import numpy as np
os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
HERE = Path(__file__).resolve().parent; ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE.parent))
import torch
torch.set_num_threads(2)
import usd_ingest as U, ingest, costs as C, s0_model as S  # noqa
from frsim.scenarios import study_cells  # noqa

BANK = ROOT / 'experiments' / 'logs' / 'bank_unseen_S0_2026-10-23'
PF = ROOT / 'experiments' / 'results' / 'unseen_block_2026-10-23' / 'per_flight.csv'


def cell_of(scenario, row):
    for c in study_cells(scenario):
        if scenario == 'A1' and abs(c['dz'] - float(row['dz'])) < 1e-9: return c
        if scenario == 'A4' and abs(c['dz'] - float(row['dz'])) < 1e-9 and abs(c['offset'] - float(row['setting'])) < 1e-9: return c
        if scenario == 'A8' and abs(c['dz'] - float(row['dz'])) < 1e-9 and abs(c['duration'] - float(row['setting'])) < 1e-9: return c
    raise KeyError((scenario, row))


def ma_floor(tr, ref):
    """Noise-floor estimate per part (position, velocity, attitude, rates) in the units of J_sysId, over the scenario part of the log."""
    t = tr[:, 0]; m = (t > 0) & (t <= ref.scenario_s); k = 51          # 0.1 s at 500 Hz
    ker = np.ones(k) / k
    def f(a):
        sm = np.stack([np.convolve(a[:, j], ker, mode='same') for j in range(a.shape[1])], 1)
        return float(((a - sm)[m][k:-k] ** 2).sum(-1).mean())
    q = tr[:, 7:11]
    return dict(p=f(tr[:, 1:4]), v=f(tr[:, 4:7]), q=f(q), w=f(tr[:, 11:14]))


if __name__ == '__main__':
    rows = [r for r in csv.DictReader(open(PF)) if r['bank'] == 'bank_unseen_S0_2026-10-23']
    out = []
    for r in rows:
        sc = r['scenario']; cell = cell_of(sc, r); f = BANK / r['folder'] / (r['folder'] + '_merged_usd.csv')
        for top_log in (False, True):
            g = U.ingest_flight(f, sc, cell, top_from_log=top_log)
            if not g.get('usable', False) and 'trace' not in g:
                print(r['folder'], 'NOT USABLE', g['gate']['reasons']); break
            ref = g['ref']; tr = g['trace']
            with torch.no_grad():
                o = S.rollout(ref, np.zeros((1, 10)), np.zeros((1, 4)), compile=False)
            tgt = ingest.on_grid(ingest.from_trace(tr), o['t'])
            tot, parts = C.j_sysid(o, tgt, ref.scenario_s, parts=True)
            parts = {k: float(v[0]) for k, v in parts.items()}
            jt = float(C.j_task(o, ref.scenario_s)[0])
            row = dict(folder=r['folder'], scenario=sc, cell=cell, top='logged' if top_log else 'planned', gate=g['gate'], reference_check=g['reference_check'], reference_check_raw=g['reference_check_raw'], frame=g['frame'], flags=g['flags'],
                       real_rms3d_cm=g['rms3d_cm'], model_rms3d_cm=100 * jt ** 0.5, j_sysid_parts=parts, floor=ma_floor(tr, ref), trace_source=g['trace_source'])
            out.append(row)
            rc, rr, fr = g['reference_check'], g['reference_check_raw'], g['frame']
            print('%-24s top %-7s gate %s | frame theta %.1f deg t (%.3f, %.3f) lag %.0f ms | ref diff raw %.3f m, aligned %.4f m (scored; +lag %.4f @ %d ms) | real %.2f cm model %.2f cm | J p %.2e v %.2e q %.2e w %.2e | floor p %.1e v %.1e q %.1e w %.1e | crashed %s floor %s' % (
                r['folder'], row['top'], 'ok' if g['gate']['ok'] else g['gate']['reasons'], fr['theta_deg'], fr['t_xy'][0], fr['t_xy'][1], 1000 * fr['lag_s'], rr['max_abs_scored_m'], rc['max_abs_scored_m'],
                rc['max_abs_scored_after_offset_and_lag_m'], rc['best_lag_ms'], row['real_rms3d_cm'], row['model_rms3d_cm'], parts['p'], parts['v'], parts['q'], parts['w'],
                row['floor']['p'], row['floor']['v'], row['floor']['q'], row['floor']['w'], g['flags']['crashed'], g['flags']['floor']), flush=True)
    json.dump(out, open(HERE / 'results' / 'ingest_oct23.json', 'w'), indent=1)
