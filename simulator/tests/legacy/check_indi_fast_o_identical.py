#!/usr/bin/env python3
"""Needs the original tuning code (`experiments/tuning/`), which is not part of this branch; the frozen reference table and `test_equivalence.py` do not need it.
[check_indi_fast_o_identical.py, simulator Stage 2, 2026-11-10: derived from experiments/tuning/diff/check_indi_fast_identical.py; fills the gap that plant_core_indi_fast_o.py had no identity test of its own]
plant_core_indi_fast_o.py (fast INDI plant, exact Omar inertia) against plant_core_indi_o.py (standard INDI plant, exact Omar inertia): bit-identical for S1 (indi 3, ns2 0) and S3 (indi 3, ns2 1), tuning cell 0 of
A1 / A4 / A8, variants nominal and md 40, gain sets default and a crashing one, plus one thrust_law-on episode per scenario; every result field that the scoring or ranking reads is compared with ==
(stable; for stable episodes x/y/z_rms_cm, tilt_max_deg, xy_steady_cm, yaw_absmax_deg, tilt_rms_deg, completed, tripped); unstable episodes must be unstable in both.
Does NOT write any flag file. The standard plant writes a trace file per episode: redirected to simulator/_build (not experiments/).
  CFFIRMWARE_DIR=<crazyflie-firmware build> nice -n 19 python3 check_indi_fast_o_identical.py [procs]"""
import json, os, sys, time
from multiprocessing import Pool
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'experiments/tuning'))
import plant_core_indi_o as P0, plant_core_indi_fast_o as P1, core
from run_job import S0_CELLS
P0.OUT = Path(__file__).resolve().parents[2] / "_build/ref_scratch/traces"; (P0.OUT / "stock_inner_traces").mkdir(parents=True, exist_ok=True)
P1.OUT = P0.OUT
FIELDS = ['x_rms_cm', 'y_rms_cm', 'z_rms_cm', 'tilt_max_deg', 'xy_steady_cm', 'yaw_absmax_deg', 'tilt_rms_deg', 'completed', 'tripped']
LAW = (0.003915371498235768, 0.9627260787640555)
TG = {'Kpos_P.x': 7., 'Kpos_P.y': 7., 'Kpos_P.z': 7., 'Kpos_D.x': 4., 'Kpos_D.y': 4., 'Kpos_D.z': 4., 'Kpos_I.z': 1.5, 'KR.x': .007, 'KR.y': .007, 'KR.z': .008, 'Komega.x': .00115, 'Komega.y': .00115, 'Komega.z': .002, 'KI.x': .03, 'KI.y': .03}
TGC = dict(TG, **{'Kpos_D.x': 0.8, 'Kpos_D.y': 0.8, 'Kpos_D.z': 0.8, 'KR.x': 0.0025, 'KR.y': 0.0025, 'Komega.x': 0.0004, 'Komega.y': 0.0004})
def work(a):
    which, cfg = a; cfg = dict(cfg); t = time.time(); r = (P0 if which == 0 else P1).run(cfg); return which, r, time.time() - t
if __name__ == '__main__':
    procs = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    cfgs = []; k = 0
    for sc in ('A1', 'A4', 'A8'):
        kw = S0_CELLS[sc][0][1]
        for v, law in ((dict(), False), (dict(md=40), False), (dict(), True)):
            for gi, tg in enumerate((TG, TGC)):
                if law and gi == 1: continue
                for ind, ns2 in ((3, 0), (3, 1)):
                    if law and ns2 == 1: continue
                    c = core.base_plant(dict(v, J='omar'))
                    c.update(scen=sc, sc=kw, gains={}, tree=True, indi=ind, ns2=ns2, tree_gains=dict(tg), seed=k)
                    if law: c['thrust_law'] = LAW
                    cfgs.append(c); k += 1
    n = len(cfgs); print(n, 'configs', flush=True)
    jobs = [(w, c) for c in cfgs for w in (0, 1)]
    with Pool(procs) as pool: res = pool.map(work, jobs, chunksize=1)
    bad = 0; t0 = t1 = 0.0; nst = nun = 0
    for k in range(n):
        (w0, r0, s0), (w1, r1, s1) = res[2 * k], res[2 * k + 1]; t0 += s0; t1 += s1
        ok = (r0['stable'] == r1['stable'])
        if ok and r0['stable']:
            nst += 1; ok = all(r0[f] == r1[f] or (isinstance(r0[f], float) and np.isnan(r0[f]) and np.isnan(r1[f])) for f in FIELDS)
        elif ok: nun += 1
        if not ok: bad += 1; print('MISMATCH', k, {f: (r0[f], r1[f]) for f in ['stable'] + FIELDS if r0[f] != r1[f]}, flush=True)
    print('episodes %d: stable %d, unstable %d; mismatches %d; wall time standard %.0f s, fast %.0f s -> speed-up %.2fx' % (n, nst, nun, bad, t0, t1, t0 / max(t1, 1e-9)))
    print('IDENTICAL' if bad == 0 else 'NOT identical')
