"""Stage 3e add-on 2: time alignment of the logged ctrltarget against the frsim reference, lag search +-200 ms (2 ms steps), A8 and A4 flights of 23 Oct."""
import csv, json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent; ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE.parent))
import usd_ingest as U, reference as R  # noqa
sys.path.insert(0, str(HERE))
from test_ingest_oct23 import cell_of, BANK, PF  # noqa
out = []
for r in csv.DictReader(open(PF)):
    if r['bank'] != 'bank_unseen_S0_2026-10-23' or r['scenario'] == 'A1': continue
    sc = r['scenario']; cell = cell_of(sc, r); fl = U.load_merged(BANK / r['folder'] / (r['folder'] + '_merged_usd.csv')); ref = R.build(sc, cell)
    g = U.ingest_flight(BANK / r['folder'] / (r['folder'] + '_merged_usd.csv'), sc, cell); fr = g['frame']; c = fl.cols[fl.bottom]; t = fl.t
    th = np.radians(fr['theta_deg']); R3 = U._rz(th)
    lg = np.stack([c['ctrltarget_x'], c['ctrltarget_y'], c['ctrltarget_z']], 1) @ R3.T + np.array([fr['t_xy'][0], fr['t_xy'][1], 0])
    m = (t > 2) & (t <= ref.scenario_s); res = []
    for lag in range(-200, 201, 2):
        rp = np.stack([np.interp(t - lag / 1000, ref.time_s, ref.pos[:, j]) for j in range(3)], 1)
        res.append((float(np.abs(lg - rp)[m].max()), lag))
    best = min(res); zero = [e for e, l in res if l == 0][0]
    # onset of motion of the logged reference vs the frsim reference (first time the xy reference leaves its t=0 value by 1 cm)
    def onset(p, tt):
        d = np.linalg.norm(p[:, :2] - p[np.searchsorted(tt, 0.0), :2], axis=1); k = np.where((tt >= 0) & (d > 0.01))[0]; return float(tt[k[0]]) if len(k) else None
    ons_log = onset(lg, t); ons_ref = onset(ref.pos, ref.time_s)
    # z of the logged reference (A8 flies z const?) and x/y scale
    out.append(dict(folder=r['folder'], scenario=sc, max_diff_lag0_m=zero, best_lag_ms=best[1], max_diff_best_m=best[0], onset_logged_s=ons_log, onset_ref_s=ons_ref,
                    t_first_s=float(t[0]), meta=fl.meta))
    print('%-24s lag0 %.4f m  best %.4f m @ %d ms | onset logged %s ref %s | log starts at t=%.3f' % (r['folder'], zero, best[0], best[1], ons_log, ons_ref, t[0]))
json.dump(out, open(HERE / 'results' / 'lag_a8.json', 'w'), indent=1)
